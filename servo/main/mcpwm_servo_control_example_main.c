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
const int SERVO_PINS[NUM_SERVOS] = {2, 4, 5}; // GPIO Pins

// Servo Physical Calibration
#define SERVO_MIN_US 550
#define SERVO_MAX_US 2400
#define SERVO_CENTER_US 1500

// 8-bit encoder: (2400-550)/256 ≈ 7.2µs per tick
#define ENCODER_RESOLUTION_US 7.0f

// Control Loop Settings
#define CONTROL_FREQ_HZ 200 // Update servos 200 times/sec

// Smoothing filter coefficient (0.0 = no smoothing, 1.0 = infinite smoothing)
// Increased to reduce current spikes with limited 2.3A PSU
// Light smoothing: 0.15f (Reduced from 0.5f because Python now sends smooth
// Quintic paths)
#define SMOOTHING_FACTOR 0.15f

// Rate limit: max microseconds change per update cycle
// At 200Hz, 15µs/update = 3000µs/sec max velocity
// This caps acceleration to prevent current spikes
#define MAX_STEP_US 15.0f

// Deadband in microseconds
#define DEADBAND_US 4.0f

// Quantize to encoder resolution
static inline float quantize_us(float us) {
  return roundf(us / ENCODER_RESOLUTION_US) * ENCODER_RESOLUTION_US;
}

// --- DATA STRUCTURES ---
typedef struct {
  float current_us;               // Current smooth position
  float target_us;                // Where we want to go (from serial)
  mcpwm_cmpr_handle_t comparator; // Handle to hardware
} ServoState;

uint32_t last_written_ticks[NUM_SERVOS] = {0, 0, 0};
ServoState servos[NUM_SERVOS];

// Mutex to protect data between UART and Motion Loop
SemaphoreHandle_t motion_mutex;

// --- MOTION LOGIC ---
// Simple exponential smoothing filter
// Runs at 200Hz
static void motion_timer_callback(void *arg) {

  if (xSemaphoreTake(motion_mutex, 0) == pdTRUE) {

    for (int i = 0; i < NUM_SERVOS; i++) {
      float error = servos[i].target_us - servos[i].current_us;

      // If within deadband, snap to target
      if (fabsf(error) < DEADBAND_US) {
        servos[i].current_us = servos[i].target_us;
      } else {
        // Exponential smoothing + rate limiting
        float smoothed = servos[i].current_us * SMOOTHING_FACTOR +
                         servos[i].target_us * (1.0f - SMOOTHING_FACTOR);

        // Rate limit: clamp step size to prevent current spikes
        float step = smoothed - servos[i].current_us;
        if (step > MAX_STEP_US)
          step = MAX_STEP_US;
        if (step < -MAX_STEP_US)
          step = -MAX_STEP_US;

        servos[i].current_us += step;
      }

      // Use full float resolution cast to int (1us precision with 1MHz timer)
      uint32_t new_ticks = (uint32_t)servos[i].current_us;

      if (new_ticks != last_written_ticks[i]) {
        mcpwm_comparator_set_compare_value(servos[i].comparator, new_ticks);
        last_written_ticks[i] = new_ticks;
      }
    }
    xSemaphoreGive(motion_mutex);
  }
}

// --- HARDWARE SETUP (MCPWM) ---
static void setup_mcpwm() {
  ESP_LOGI(TAG, "Init MCPWM...");
  mcpwm_timer_handle_t timer = NULL;
  mcpwm_timer_config_t timer_config = {
      .group_id = 0,
      .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
      .resolution_hz = 1000000, // 1MHz = 1us per tick
      .period_ticks = 5000,     // 200Hz PWM (5ms period) to match Control Loop
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

    // Init State
    servos[i].current_us = SERVO_CENTER_US;
    servos[i].target_us = SERVO_CENTER_US;
    last_written_ticks[i] = SERVO_CENTER_US;
  }

  ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

// --- MAIN TASK ---
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
  uart_driver_install(UART_NUM_0, 1024 * 2, 0, 0, NULL, 0);
  uart_param_config(UART_NUM_0, &uart_config);

  ESP_LOGI(TAG, "Smooth Filter Ready. Send: A,1500,1500,1500 (us)");

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

            // Just update targets - smoothing happens in timer callback
            servos[0].target_us = t1;
            servos[1].target_us = t2;
            servos[2].target_us = t3;

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
