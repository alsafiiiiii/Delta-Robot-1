/**
 * @file main5.c (ARCHIVED)
 * @brief Trajectory Library Integration (Per-Joint Trapezoid)
 *
 * CHANGES FROM main4.c:
 * - Uses trajectory.h library for smooth motion
 * - Per-joint trapezoidal velocity profiles
 * - Max Vel 300 deg/s, Accel 400 deg/s^2
 * - No buffering - stream directly to trajectory generator
 *
 * ISSUES: Per-joint profiles != coordinated Cartesian motion.
 *         Arms may not all finish at the same time.
 */
#include "driver/mcpwm_prelude.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "nvs_flash.h"
#include <errno.h>
#include <lwip/netdb.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>

// Include the C version of the library
#include "trajectory.h"

static const char *TAG = "delta_c_driver";

// --- CONFIGURATION ---
#define WIFI_SSID "Galaxy S25 Ultra 7880"
#define WIFI_PASS "12345678"
#define PORT 3333
#define NUM_SERVOS 3

// Tuned for STREAMING SMOOTHNESS
// Max Speed: 300 deg/s
// Accel: 400 deg/s^2 (Acts as Low Pass Filter for Jitter)
// Decel: 400 deg/s^2 (Soft Stop)
#define TRAJ_MAX_VEL 300.0f
#define TRAJ_ACCEL 400.0f
#define TRAJ_DECEL 400.0f

#define SERVO_MIN_PULSEWIDTH_US 500
#define SERVO_MAX_PULSEWIDTH_US 2400
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000
#define SERVO_TIMEBASE_PERIOD 20000

static const int SERVO_GPIOS[NUM_SERVOS] = {2, 4, 5};

// --- DATA STRUCTURES ---
typedef struct {
  float angles[5];
} delta_packet_t;

// --- GLOBALS ---
mcpwm_cmpr_handle_t comparators[NUM_SERVOS];
Trajectory_t servo_traj[NUM_SERVOS]; // Array of structs

// Shared Data
delta_packet_t current_target_packet;
SemaphoreHandle_t target_mutex;
bool new_data_available = false;

// --- UTILS ---
static inline uint32_t angle_to_compare(float angle) {
  if (angle < 0)
    angle = 0;
  if (angle > 180)
    angle = 180;
  return (uint32_t)((angle) *
                        (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) /
                        180 +
                    SERVO_MIN_PULSEWIDTH_US);
}

// --- TASK 1: UDP RECEIVER ---
static void udp_server_task(void *pvParameters) {
  delta_packet_t packet;
  struct sockaddr_in dest_addr;
  dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(PORT);

  int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (sock < 0) {
    ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
    vTaskDelete(NULL);
    return;
  }
  bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
  ESP_LOGI(TAG, "UDP Listening (Pure C Mode)...");

  while (1) {
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);

    int len = recvfrom(sock, &packet, sizeof(delta_packet_t), 0,
                       (struct sockaddr *)&source_addr, &socklen);

    if (len == sizeof(delta_packet_t)) {
      xSemaphoreTake(target_mutex, portMAX_DELAY);
      current_target_packet = packet;
      new_data_available = true;
      xSemaphoreGive(target_mutex);
    }
  }
}

// --- TASK 2: MOTION LOOP ---
static void motion_task(void *pvParameters) {
  const int LOOP_RATE_HZ = 250;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1000 / LOOP_RATE_HZ);

  // Initialize Trajectories
  for (int i = 0; i < NUM_SERVOS; i++) {
    traj_init(&servo_traj[i], TRAJ_MAX_VEL, TRAJ_ACCEL, TRAJ_DECEL, 0.1f);
    traj_reset(&servo_traj[i], 90.0f);
  }

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // 1. Check for new target
    if (new_data_available) {
      xSemaphoreTake(target_mutex, portMAX_DELAY);
      delta_packet_t target = current_target_packet;
      new_data_available = false;
      xSemaphoreGive(target_mutex);

      // Update Targets
      for (int i = 0; i < NUM_SERVOS; i++) {
        traj_setTargetPos(&servo_traj[i], target.angles[i]);
      }
    }

    // 2. Update Physics
    for (int i = 0; i < NUM_SERVOS; i++) {
      float angle = traj_update(&servo_traj[i]);
      mcpwm_comparator_set_compare_value(comparators[i],
                                         angle_to_compare(angle));
    }
  }
}

// --- WIFI BOILERPLATE ---
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static void wifi_handler(void *arg, esp_event_base_t base, int32_t id,
                         void *data) {
  if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START)
    esp_wifi_connect();
  else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
    esp_wifi_connect();
}

void app_main(void) {
  target_mutex = xSemaphoreCreateMutex();
  nvs_flash_init();
  s_wifi_event_group = xEventGroupCreate();
  esp_netif_init();
  esp_event_loop_create_default();

  esp_netif_create_default_wifi_sta();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);

  esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                      &wifi_handler, NULL, NULL);
  esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                      &wifi_handler, NULL, NULL);

  wifi_config_t w_cfg = {};
  strcpy((char *)w_cfg.sta.ssid, WIFI_SSID);
  strcpy((char *)w_cfg.sta.password, WIFI_PASS);
  w_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_config(WIFI_IF_STA, &w_cfg);
  esp_wifi_start();
  esp_wifi_set_ps(WIFI_PS_NONE);

  xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE,
                      portMAX_DELAY);

  // MCPWM
  mcpwm_timer_handle_t timer = NULL;
  mcpwm_timer_config_t t_cfg = {.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
                                .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
                                .period_ticks = SERVO_TIMEBASE_PERIOD,
                                .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
                                .group_id = 0};
  ESP_ERROR_CHECK(mcpwm_new_timer(&t_cfg, &timer));

  for (int i = 0; i < NUM_SERVOS; i++) {
    mcpwm_oper_handle_t oper;
    mcpwm_operator_config_t o_cfg = {.group_id = 0};
    ESP_ERROR_CHECK(mcpwm_new_operator(&o_cfg, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    mcpwm_cmpr_handle_t cmpr;
    mcpwm_comparator_config_t c_cfg = {.flags.update_cmp_on_tez = true};
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &c_cfg, &cmpr));
    comparators[i] = cmpr;

    mcpwm_gen_handle_t gen;
    mcpwm_generator_config_t g_cfg = {.gen_gpio_num = SERVO_GPIOS[i]};
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &g_cfg, &gen));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        gen, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                          MCPWM_TIMER_EVENT_EMPTY,
                                          MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        gen, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr,
                                            MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(
        mcpwm_comparator_set_compare_value(cmpr, angle_to_compare(90)));
  }

  ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

  xTaskCreate(motion_task, "motion", 4096, NULL, 10, NULL);
  xTaskCreate(udp_server_task, "udp", 4096, NULL, 5, NULL);
}
