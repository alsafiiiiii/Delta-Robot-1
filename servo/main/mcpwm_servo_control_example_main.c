/**
 * Interpolating Servo Controller for ESP32
 *
 * Implements "Fire and Forget" command protocol:
 * T<idx>:<degrees> D:<duration_ms>
 *
 * The ESP32 handles smooth linear interpolation between the current position
 * and the target position over the specified duration.
 *
 * PWM Frequency: 50Hz (Standard for digital servos)
 * Control Loop: 50Hz
 */

#include "driver/mcpwm_prelude.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

static const char *TAG = "InterpServo";

// --- CONFIGURATION ---
#define NUM_SERVOS 3
const int SERVO_PINS[NUM_SERVOS] = {2, 4, 5};

// Servo Calibration
#define SERVO_MIN_US 550
#define SERVO_MAX_US 2400
#define SERVO_MIN_DEG 0.0f
#define SERVO_MAX_DEG 180.0f

// Stability Tuning
// Mix Factor: 0.0=Linear(Vibration-Free), 1.0=Cubic(Smooth-Jerk)
// 0.5 blends them: Start/End Velocity is 50% of cruise (Not zero)
#define HYBRID_FACTOR 0.8f

// UART
#define UART_NUM UART_NUM_0
#define BUF_SIZE 1024

// Control Loop
#define CONTROL_FREQ_HZ 50

// --- DATA STRUCTURES ---
typedef struct {
  float current_degree; // Actual Output Position
  float start_degree;   // Position at start of move
  float target_degree;  // Final destination
  int current_tick;     // Current progress (0 to total_ticks)
  int total_ticks;      // Duration of move in ticks
  mcpwm_cmpr_handle_t comparator;
} ServoState;

ServoState servos[NUM_SERVOS];

// --- HELPERS ---
float deg_to_us(float deg) {
  deg = fmaxf(SERVO_MIN_DEG, fminf(SERVO_MAX_DEG, deg));
  return (deg - SERVO_MIN_DEG) * (SERVO_MAX_US - SERVO_MIN_US) /
             (SERVO_MAX_DEG - SERVO_MIN_DEG) +
         SERVO_MIN_US;
}

// --- MOTION TIMER ---
// --- MOTION TIMER ---
// --- MOTION TIMER ---
static void motion_timer_callback(void *arg) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (servos[i].current_tick < servos[i].total_ticks) {
      servos[i].current_tick++;

      // Normalized Time (0.0 to 1.0)
      float t = (float)servos[i].current_tick / (float)servos[i].total_ticks;

      // Cubic Ease-In/Ease-Out: 3t^2 - 2t^3
      // Forces zero velocity at start and end
      float cubic = (3 * t * t) - (2 * t * t * t);

      // Linear: t
      float linear = t;

      // Hybrid Blend
      float ease = (1.0f - HYBRID_FACTOR) * linear + (HYBRID_FACTOR)*cubic;

      // Interpolate
      float spread = servos[i].target_degree - servos[i].start_degree;
      servos[i].current_degree = servos[i].start_degree + (spread * ease);

    } else {
      // Ensure we hold exactly at target
      servos[i].current_degree = servos[i].target_degree;
    }

    // Write to Hardware
    uint32_t us = (uint32_t)deg_to_us(servos[i].current_degree);
    mcpwm_comparator_set_compare_value(servos[i].comparator, us);
  }
}

// --- COMMAND PARSER ---
// Format: "T0:90.5 D:1000" (Servo 0 to 90.5 deg in 1000ms)
void process_command(char *line) {
  int servo_idx;
  float target_deg;
  int duration_ms;

  // Safety check line length
  if (strlen(line) > 64)
    return;

  // Parse "T<idx>:<deg> D:<ms>"
  char *t_ptr = strstr(line, "T");
  char *d_ptr = strstr(line, "D");

  if (t_ptr && d_ptr) {
    // Parse T part
    if (sscanf(t_ptr, "T%d:%f", &servo_idx, &target_deg) == 2) {
      // Parse D part
      if (sscanf(d_ptr, "D:%d", &duration_ms) == 1) {

        // Validate inputs
        if (servo_idx >= 0 && servo_idx < NUM_SERVOS) {
          // Clamp target
          target_deg = fmaxf(SERVO_MIN_DEG, fminf(SERVO_MAX_DEG, target_deg));

          // Clamp duration (min 20ms to avoid divide by zero)
          if (duration_ms < 20)
            duration_ms = 20;

          // Calculate Steps for S-Curve
          int ticks = duration_ms * CONTROL_FREQ_HZ / 1000;

          servos[servo_idx].start_degree = servos[servo_idx].current_degree;
          servos[servo_idx].target_degree = target_deg;
          servos[servo_idx].total_ticks = ticks;
          servos[servo_idx].current_tick = 0;

          // Debug Log
          // ESP_LOGI(TAG, "S%d -> %.1f deg in %d ms", servo_idx, target_deg,
          // duration_ms);
        }
      }
    }
  }
}

// --- UART TASK ---
static void rx_task(void *arg) {
  uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
  char line_buffer[128];
  int line_pos = 0;

  while (1) {
    int len =
        uart_read_bytes(UART_NUM, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);
    if (len > 0) {
      for (int i = 0; i < len; i++) {
        char c = (char)data[i];
        if (c == '\n' || c == '\r') {
          if (line_pos > 0) {
            line_buffer[line_pos] = '\0';
            process_command(line_buffer);
            line_pos = 0;
          }
        } else {
          if (line_pos < sizeof(line_buffer) - 1) {
            line_buffer[line_pos++] = c;
          }
        }
      }
    }
  }
  free(data);
}

// --- HARDWARE SETUP ---
static void setup_mcpwm() {
  ESP_LOGI(TAG, "Initializing MCPWM...");

  mcpwm_timer_handle_t timer = NULL;
  mcpwm_timer_config_t timer_config = {
      .group_id = 0,
      .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
      .resolution_hz = 1000000,
      .period_ticks = 20000, // 50Hz
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

    // Initial State
    servos[i].current_degree = 90.0f; // Assume center start
    servos[i].start_degree = 90.0f;
    servos[i].target_degree = 90.0f;
    servos[i].total_ticks = 0;
    servos[i].current_tick = 0;

    mcpwm_comparator_set_compare_value(servos[i].comparator,
                                       (uint32_t)deg_to_us(90.0f));
  }

  ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

static void setup_uart() {
  const uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
  uart_param_config(UART_NUM, &uart_config);
  uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

// --- MAIN ---
void app_main(void) {
  setup_mcpwm();
  setup_uart();

  // Start motion timer
  const esp_timer_create_args_t timer_args = {
      .callback = &motion_timer_callback, .name = "motion_loop"};
  esp_timer_handle_t motion_timer;
  ESP_ERROR_CHECK(esp_timer_create(&timer_args, &motion_timer));
  ESP_ERROR_CHECK(
      esp_timer_start_periodic(motion_timer, 1000000 / CONTROL_FREQ_HZ));

  xTaskCreate(rx_task, "uart_rx_task", 4096, NULL, configMAX_PRIORITIES - 1,
              NULL);

  ESP_LOGI(TAG, "Interpolation Controller Started. 50Hz Control.");
}
