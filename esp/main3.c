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

static const char *TAG = "delta_buffer_driver";

// --- CONFIGURATION ---
#define WIFI_SSID "Galaxy S25 Ultra 7880"
#define WIFI_PASS "12345678"
#define PORT 3333
#define NUM_SERVOS 3 // CHANGED: Only 3 Arm Motors
#define MIN_STEP_DEG 0.40f

// --- BUFFER SETTINGS ---
#define BUFFER_SIZE 64   // Increased from 32 -> 64
#define STARTUP_COUNT 10 // Increased from 2 -> 10 (200ms buffer)

// --- TIMING MATH ---
// --- TIMING MATH ---
#define INTERP_STEP                                                            \
  0.2f // 0.2 step = 5 sub-steps per packet (50Hz input -> 250Hz output)
#define MOTION_FREQ_HZ 250 // 250Hz Motion Loop (4ms tick)

// ... (existing code: pins and definitions) ...

// CHANGED: Only pins 2, 4, 5 for the arms
static const int SERVO_GPIOS[NUM_SERVOS] = {2, 4, 5};

#define SERVO_MIN_PULSEWIDTH_US 500
#define SERVO_MAX_PULSEWIDTH_US 2400
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000 // 1MHz Resolution (1us per tick)
#define SERVO_TIMEBASE_PERIOD                                                  \
  20000 // 20000 ticks = 20ms (50Hz) standard servo frame

typedef struct {
  float angles[5];
} delta_packet_t;
mcpwm_cmpr_handle_t comparators[NUM_SERVOS];
float last_written_angles[NUM_SERVOS] = {90.0, 90.0, 90.0};

// --- RING BUFFER ---
SemaphoreHandle_t buffer_mutex;
delta_packet_t buffer[BUFFER_SIZE];
volatile int head = 0;
volatile int tail = 0;
volatile int count = 0;

static inline uint32_t angle_to_compare(float angle) {
  if (angle < 0)
    angle = 0;
  if (angle > 180)
    angle = 180;
  // Map 0-180 to MIN-MAX
  uint32_t us =
      (angle) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / 180 +
      SERVO_MIN_PULSEWIDTH_US;
  return us; // 1MHz clock = 1 tick per us
}

// --- TASK 1: UDP RECEIVER ---
static void udp_server_task(void *pvParameters) {
  delta_packet_t packet;
  struct sockaddr_in dest_addr;
  dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(PORT);

  int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));

  ESP_LOGI(TAG, "UDP Listening (3 Motors). Buffering packets...");

  while (1) {
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);
    int len = recvfrom(sock, &packet, sizeof(delta_packet_t), 0,
                       (struct sockaddr *)&source_addr, &socklen);

    if (len == sizeof(delta_packet_t)) {
      xSemaphoreTake(buffer_mutex, portMAX_DELAY);
      if (count < BUFFER_SIZE) {
        buffer[head] = packet;
        head = (head + 1) % BUFFER_SIZE;
        count++;
      } else {
        tail = (tail + 1) % BUFFER_SIZE;
        buffer[head] = packet;
        head = (head + 1) % BUFFER_SIZE;
        ESP_LOGW(TAG, "Buffer Overflow! (Dropping oldest)");
      }
      int current_count = count; // Capture for logging to avoid race in LOGI
      xSemaphoreGive(buffer_mutex);

      // Only log randomly or it floods given we want speed, but let's keep it
      // for now ESP_LOGI(TAG, "RX: %6.2f", packet.angles[0]);
    }
  }
}

// --- TASK 2: INTERPOLATOR ---
static void motion_task(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  TickType_t xFrequency = pdMS_TO_TICKS(1000 / MOTION_FREQ_HZ);
  if (xFrequency == 0)
    xFrequency = 1; // Safety: Minimum 1 tick to prevent crash

  float phase = 0.0f;
  bool active = false;
  delta_packet_t start_pt, end_pt;

  // Init to home
  for (int i = 0; i < NUM_SERVOS; i++) {
    start_pt.angles[i] = 90.0;
    end_pt.angles[i] = 90.0;
  }

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    if (!active) {
      bool ready = false;
      xSemaphoreTake(buffer_mutex, portMAX_DELAY);
      if (count >= STARTUP_COUNT) {
        start_pt = buffer[tail];
        tail = (tail + 1) % BUFFER_SIZE;
        count--;

        if (count > 0) {
          end_pt = buffer[tail];
          ready = true;
        }
      }
      xSemaphoreGive(buffer_mutex);

      if (ready) {
        active = true;
        phase = 0.0f;
        // ESP_LOGI(TAG, ">>> STARTING MOTION");
      }
    } else {
      // --- LINEAR INTERPOLATION (Better for streaming) ---
      // S-Curve removed because it causes stop-start jerk between streaming
      // points
      float t = phase;

      for (int i = 0; i < NUM_SERVOS; i++) {
        float ideal_angle =
            start_pt.angles[i] + (end_pt.angles[i] - start_pt.angles[i]) * t;

        // Always write to ensure smoothness, MCPWM is fast enough
        mcpwm_comparator_set_compare_value(comparators[i],
                                           angle_to_compare(ideal_angle));
      }

      phase += INTERP_STEP;

      if (phase >= 1.0f) {
        phase -= 1.0f;
        start_pt = end_pt;

        bool has_next = false;
        xSemaphoreTake(buffer_mutex, portMAX_DELAY);
        // We DON'T pop 'tail' here because 'end_pt' was just a peek at the next
        // one? Actually in the previous logic, we weren't peeking, we were
        // consuming. Let's correct the logic:
        // 1. We just finished the segment from start_pt to end_pt.
        // 2. So now end_pt becomes the new start_pt.
        // 3. We need to fetch a NEW end_pt from the buffer.
        // 4. The previous 'end_pt' was already consumed from the buffer in the
        // sense that 'tail' pointed to it? Wait, the previous logic was:
        //   start_pt = buffer[tail]; count--; tail++;
        //   if (count > 0) end_pt = buffer[tail]; (This is PEEKING at the new
        //   tail)
        // correct.

        // So now, we need to CONSUME the one we were peeking at, to make it the
        // new start? actually, let's keep it simple: start_pt is implicitly
        // end_pt. We just need to pop one more from the buffer to be the next
        // end_pt.

        tail = (tail + 1) % BUFFER_SIZE;
        count--; // Consume the one that was 'end_pt'

        if (count > 0) {
          end_pt = buffer[tail]; // Peek at next
          has_next = true;
        }
        xSemaphoreGive(buffer_mutex);

        if (!has_next) {
          active = false;
          ESP_LOGW(TAG, "!!! BUFFER EMPTY (Underrun) !!!");
        }
      }
    }
  }
}

// (Standard WiFi Boilerplate)
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
  buffer_mutex = xSemaphoreCreateMutex(); // Create Mutex
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
  wifi_config_t w_cfg = {.sta = {.ssid = WIFI_SSID, .password = WIFI_PASS}};
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_config(WIFI_IF_STA, &w_cfg);
  esp_wifi_start();
  esp_wifi_set_ps(WIFI_PS_NONE); // CRITICAL: Disable power save for low latency
  xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE,
                      portMAX_DELAY);

  // MCPWM Setup (Simpler now - Just Group 0)
  mcpwm_timer_handle_t timer = NULL;
  mcpwm_timer_config_t t_cfg = {.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
                                .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
                                .period_ticks = SERVO_TIMEBASE_PERIOD,
                                .count_mode = MCPWM_TIMER_COUNT_MODE_UP};
  t_cfg.group_id = 0;
  ESP_ERROR_CHECK(mcpwm_new_timer(&t_cfg, &timer));

  for (int i = 0; i < NUM_SERVOS; i++) {
    mcpwm_oper_handle_t oper;
    mcpwm_operator_config_t o_cfg = {.group_id = 0}; // All in Group 0
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