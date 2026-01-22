#include "driver/mcpwm_prelude.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

static const char *TAG = "DeltaController";

// --- CONFIGURATION ---
#define NUM_SERVOS 3
const int SERVO_PINS[NUM_SERVOS] = {2, 4, 5};

#define SERVO_MIN_US 550
#define SERVO_MAX_US 2400
#define SERVO_CENTER_US 1500

// 8-bit encoder resolution
#define ENCODER_RESOLUTION_US 7.0f

// Control Loop
#define CONTROL_FREQ_HZ 200

// Motion Profile Settings
// Servo at 7.5V: ~0.12sec/60° = 500°/sec = ~5000µs/sec
#define MAX_SPEED_US_PER_SEC 4000.0f // Max servo speed (slightly conservative)
#define MIN_MOTION_TIME_MS 100 // Minimum motion duration for S-curve benefit

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Quantize to encoder resolution
static inline float quantize_us(float us) {
  return roundf(us / ENCODER_RESOLUTION_US) * ENCODER_RESOLUTION_US;
}

// S-curve function: smooth acceleration/deceleration
// progress: 0.0 to 1.0, returns: 0.0 to 1.0
static inline float scurve(float t) {
  if (t <= 0.0f)
    return 0.0f;
  if (t >= 1.0f)
    return 1.0f;
  return (1.0f - cosf(t * M_PI)) * 0.5f;
}

// --- DATA STRUCTURES ---
typedef struct {
  float start_us;        // Where motion started
  float target_us;       // Where we want to go
  float current_us;      // Current interpolated position
  int64_t start_time_us; // When motion started
  int64_t duration_us;   // How long motion should take
  bool moving;           // Is servo in motion
  mcpwm_cmpr_handle_t comparator;
} ServoState;

uint32_t last_written_ticks[NUM_SERVOS] = {0, 0, 0};
ServoState servos[NUM_SERVOS];
SemaphoreHandle_t motion_mutex;

// --- MOTION LOGIC ---
// S-curve interpolation based on distance and time
static void motion_timer_callback(void *arg) {
  int64_t now_us = esp_timer_get_time();

  if (xSemaphoreTake(motion_mutex, 0) == pdTRUE) {
    for (int i = 0; i < NUM_SERVOS; i++) {

      if (servos[i].moving) {
        int64_t elapsed = now_us - servos[i].start_time_us;

        if (elapsed >= servos[i].duration_us) {
          // Motion complete
          servos[i].current_us = servos[i].target_us;
          servos[i].moving = false;
        } else {
          // Interpolate using S-curve
          float progress = (float)elapsed / (float)servos[i].duration_us;
          float s = scurve(progress);
          float delta = servos[i].target_us - servos[i].start_us;
          servos[i].current_us = servos[i].start_us + delta * s;
        }
      }

      // Quantize and write
      uint32_t new_ticks = (uint32_t)quantize_us(servos[i].current_us);
      if (new_ticks != last_written_ticks[i]) {
        mcpwm_comparator_set_compare_value(servos[i].comparator, new_ticks);
        last_written_ticks[i] = new_ticks;
      }
    }
    xSemaphoreGive(motion_mutex);
  }
}

// Start a new motion command
static void start_motion(int idx, float new_target) {
  float distance = fabsf(new_target - servos[idx].current_us);

  // Skip tiny movements
  if (distance < ENCODER_RESOLUTION_US) {
    servos[idx].target_us = new_target;
    return;
  }

  // Calculate duration based on distance and max speed
  float duration_sec = distance / MAX_SPEED_US_PER_SEC;
  int64_t duration_us = (int64_t)(duration_sec * 1000000.0f);

  // Apply minimum duration for smooth S-curve
  if (duration_us < MIN_MOTION_TIME_MS * 1000) {
    duration_us = MIN_MOTION_TIME_MS * 1000;
  }

  servos[idx].start_us = servos[idx].current_us;
  servos[idx].target_us = new_target;
  servos[idx].start_time_us = esp_timer_get_time();
  servos[idx].duration_us = duration_us;
  servos[idx].moving = true;
}

// --- HARDWARE SETUP ---
static void setup_mcpwm() {
  ESP_LOGI(TAG, "Init MCPWM...");
  mcpwm_timer_handle_t timer = NULL;
  mcpwm_timer_config_t timer_config = {
      .group_id = 0,
      .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
      .resolution_hz = 1000000,
      .period_ticks = 20000,
      .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
  };
  ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

  mcpwm_oper_handle_t oper = NULL;
  mcpwm_operator_config_t oper_config = {.group_id = 0};

  for (int i = 0; i < NUM_SERVOS; i++) {
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    mcpwm_comparator_config_t cmp_config = {.flags.update_cmp_on_tez = true};
    ESP_ERROR_CHECK(
        mcpwm_new_comparator(oper, &cmp_config, &servos[i].comparator));

    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t gen_config = {.gen_gpio_num = SERVO_PINS[i]};
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_config, &generator));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                MCPWM_TIMER_EVENT_EMPTY,
                                                MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        generator, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                  servos[i].comparator,
                                                  MCPWM_GEN_ACTION_LOW)));

    servos[i].current_us = SERVO_CENTER_US;
    servos[i].target_us = SERVO_CENTER_US;
    servos[i].start_us = SERVO_CENTER_US;
    servos[i].moving = false;
    last_written_ticks[i] = SERVO_CENTER_US;
  }

  ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

void app_main(void) {
  motion_mutex = xSemaphoreCreateMutex();
  setup_mcpwm();

  const esp_timer_create_args_t periodic_timer_args = {
      .callback = &motion_timer_callback, .name = "motion_loop"};
  esp_timer_handle_t periodic_timer;
  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
  ESP_ERROR_CHECK(
      esp_timer_start_periodic(periodic_timer, 1000000 / CONTROL_FREQ_HZ));

  uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  uart_driver_install(UART_NUM_0, 2048, 0, 0, NULL, 0);
  uart_param_config(UART_NUM_0, &uart_config);

  ESP_LOGI(TAG, "S-Curve Motion Ready. A,us1,us2,us3");

  uint8_t *data = (uint8_t *)malloc(128);
  char line[64];
  int line_idx = 0;

  while (1) {
    int len = uart_read_bytes(UART_NUM_0, data, 128, pdMS_TO_TICKS(100));
    for (int i = 0; i < len; i++) {
      char c = (char)data[i];
      if (c == '\n') {
        line[line_idx] = 0;
        if (line[0] == 'A') {
          float t1, t2, t3;
          if (sscanf(line, "A,%f,%f,%f", &t1, &t2, &t3) == 3) {
            xSemaphoreTake(motion_mutex, portMAX_DELAY);

            // Safety clamps
            if (t1 < SERVO_MIN_US)
              t1 = SERVO_MIN_US;
            if (t1 > SERVO_MAX_US)
              t1 = SERVO_MAX_US;
            if (t2 < SERVO_MIN_US)
              t2 = SERVO_MIN_US;
            if (t2 > SERVO_MAX_US)
              t2 = SERVO_MAX_US;
            if (t3 < SERVO_MIN_US)
              t3 = SERVO_MIN_US;
            if (t3 > SERVO_MAX_US)
              t3 = SERVO_MAX_US;

            // Start S-curve motion for each servo
            start_motion(0, t1);
            start_motion(1, t2);
            start_motion(2, t3);

            xSemaphoreGive(motion_mutex);
          }
        }
        line_idx = 0;
      } else {
        if (line_idx < 63)
          line[line_idx++] = c;
      }
    }
  }
}
