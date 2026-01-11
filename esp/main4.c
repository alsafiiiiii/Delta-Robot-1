#include "driver/mcpwm_prelude.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"
#include <math.h>
#include <string.h>

static const char *TAG = "delta_fast_driver";

// --- CONFIGURATION ---
#define WIFI_SSID "Amith S's Galaxy A53"
#define WIFI_PASS "ycij95136"
#define PORT 3333
#define NUM_SERVOS 3

// --- BUFFER SETTINGS ---
#define BUFFER_SIZE 128 // Increased Size
#define STARTUP_COUNT 1 // START IMMEDIATELY (Low Latency)

// --- TIMING MATH ---
// Input: 50Hz (20ms) from PC
// Output: 250Hz (4ms) loop
#define MOTION_FREQ_HZ 250
#define INTERP_STEP 0.2f // 1.0 / (250/50) = 0.2

static const int SERVO_GPIOS[NUM_SERVOS] = {2, 4, 5};

#define SERVO_MIN_PULSEWIDTH_US 500
#define SERVO_MAX_PULSEWIDTH_US 2400
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000 // 1MHz
#define SERVO_TIMEBASE_PERIOD 20000          // 20ms

typedef struct {
  float angles[5];
} delta_packet_t;

mcpwm_cmpr_handle_t comparators[NUM_SERVOS];
delta_packet_t buffer[BUFFER_SIZE];

// Synchronization
SemaphoreHandle_t buffer_mutex;
volatile int head = 0;
volatile int tail = 0;
volatile int count = 0;

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

  ESP_LOGI(TAG, "UDP Listening (Fast Mode)...");

  while (1) {
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);

    // Blocking Receive
    int len = recvfrom(sock, &packet, sizeof(delta_packet_t), 0,
                       (struct sockaddr *)&source_addr, &socklen);

    if (len == sizeof(delta_packet_t)) {
      xSemaphoreTake(buffer_mutex, portMAX_DELAY);

      // If buffer is full, we DROP the OLDEST to make room (Latest is most
      // important)
      if (count == BUFFER_SIZE) {
        // Drop tail (oldest)
        tail = (tail + 1) % BUFFER_SIZE;
        count--;
        // ESP_LOGW(TAG, "Buffer Full - Dropping Oldest");
      }

      buffer[head] = packet;
      head = (head + 1) % BUFFER_SIZE;
      count++;

      xSemaphoreGive(buffer_mutex);
    }
  }
}

// --- TASK 2: MOTION CONTROLLER ---
static void motion_task(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1000 / MOTION_FREQ_HZ);

  float phase = 0.0f;
  bool active = false;

  delta_packet_t start_pt, end_pt;

  // Initialize to 90 degrees (Home)
  for (int i = 0; i < NUM_SERVOS; i++) {
    start_pt.angles[i] = 90.0;
    end_pt.angles[i] = 90.0;
  }

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency); // Strict Timing

    // --- State Machine ---

    // 1. If not active, try to start
    if (!active) {
      xSemaphoreTake(buffer_mutex, portMAX_DELAY);
      if (count >= STARTUP_COUNT) {
        // Consume one to catch up/start
        start_pt = buffer[tail];
        tail = (tail + 1) % BUFFER_SIZE;
        count--;

        // If we have another, that is our target
        if (count > 0) {
          end_pt = buffer[tail];
          // We do NOT consume end_pt yet, we interpolating TOWARDS it
          // Actually, standard logic:
          // Segment is FROM start TO end.
          // We consume 'end' when we finish the segment.
          // So here we just PEEK 'end' or we consume it?
          // Let's CONSUME it now so we own it.
          tail = (tail + 1) % BUFFER_SIZE;
          count--;

          active = true;
          phase = 0.0f;
          ESP_LOGI(TAG, "Motion Started");
        }
      }
      xSemaphoreGive(buffer_mutex);

      // If still not active, just write start_pt
      if (!active) {
        for (int i = 0; i < NUM_SERVOS; i++) {
          mcpwm_comparator_set_compare_value(
              comparators[i], angle_to_compare(start_pt.angles[i]));
        }
        continue;
      }
    }

    // 2. If active, interpolate
    if (active) {
      float t = phase;

      // Linear Interpolation
      for (int i = 0; i < NUM_SERVOS; i++) {
        float val =
            start_pt.angles[i] + (end_pt.angles[i] - start_pt.angles[i]) * t;
        mcpwm_comparator_set_compare_value(comparators[i],
                                           angle_to_compare(val));
      }

      phase += INTERP_STEP;

      // 3. Segment Complete
      if (phase >= 1.0f) {
        phase -= 1.0f; // Keep fraction for precision? Or just reset? Reset is
                       // safer for now.
        phase = 0.0f;

        // Shift: End becomes Start
        start_pt = end_pt;

        // Fetch Next
        xSemaphoreTake(buffer_mutex, portMAX_DELAY);
        if (count > 0) {
          end_pt = buffer[tail];
          tail = (tail + 1) % BUFFER_SIZE;
          count--;
        } else {
          // BUFFER UNDERRUN
          // CRITICAL FIX: Do NOT stop. Just Hold.
          // Target effectively becomes current point (distance 0)
          end_pt = start_pt;
          // active remains true.
          // We just interpolate from A to A (hold) for next cycle
          // ESP_LOGD(TAG, "Underrun - Holding");
        }
        xSemaphoreGive(buffer_mutex);
      }
    }
  }
}

// --- BOILERPLATE WIFI & SETUP ---
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
  buffer_mutex = xSemaphoreCreateMutex();
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

  wifi_config_t w_cfg = {.sta = {.ssid = WIFI_SSID,
                                 .password = WIFI_PASS,
                                 .threshold.authmode = WIFI_AUTH_WPA2_PSK}};
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_config(WIFI_IF_STA, &w_cfg);
  esp_wifi_start();

  // Power Save OFF for latency
  esp_wifi_set_ps(WIFI_PS_NONE);

  xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE,
                      portMAX_DELAY);

  // MCPWM Init
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

    // Init Center
    ESP_ERROR_CHECK(
        mcpwm_comparator_set_compare_value(cmpr, angle_to_compare(90)));
  }

  ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

  xTaskCreate(motion_task, "motion", 4096, NULL, 10, NULL);
  xTaskCreate(udp_server_task, "udp", 4096, NULL, 5, NULL);
}
