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

// Servo Physical Calibration (TUNE THESE!)
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2400

// Control Loop Settings
#define CONTROL_FREQ_HZ 50 // Update servos 200 times/sec
#define DEFAULT_SPEED 2800.0f
// --- DATA STRUCTURES ---
typedef struct {
  float current_us;               // Current position (float for precision)
  float target_us;                // Where we want to go
  float speed;                    // Speed in US/sec
  mcpwm_cmpr_handle_t comparator; // Handle to hardware
} ServoState;
uint32_t last_written_ticks[NUM_SERVOS] = {0, 0, 0};
ServoState servos[NUM_SERVOS];

// Mutex to protect data between UART (Core 0) and Motion Loop (Core 1)
SemaphoreHandle_t motion_mutex;

// --- MOTION LOGIC (The "Ported" bit) ---
// This runs inside a high-priority timer callback
static void motion_timer_callback(void* arg) {
    const float dt = 1.0f / CONTROL_FREQ_HZ;

    if (xSemaphoreTake(motion_mutex, 0) == pdTRUE) {
        
        for (int i = 0; i < NUM_SERVOS; i++) {
            float err = servos[i].target_us - servos[i].current_us;
            
            // 1. DEADBAND CHECK (Logic Logic)
            if (fabsf(err) < 8.0f) {
                // We are close enough. Force internal state to target 
                // to stop float drift.
                servos[i].current_us = servos[i].target_us;
            } 
            else {
                // Move logic
                float max_step = servos[i].speed * dt;
                if (max_step > fabsf(err)) {
                    servos[i].current_us = servos[i].target_us;
                } else {
                    if (err > 0) servos[i].current_us += max_step;
                    else         servos[i].current_us -= max_step;
                }
            }

            // 2. WRITE FILTER (Anti-Jitter Logic)
            // Only talk to the hardware if the value CHANGED.
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

    // Set PWM Actions: High on Zero, Low on Compare
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                MCPWM_TIMER_EVENT_EMPTY,
                                                MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        generator, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                  servos[i].comparator,
                                                  MCPWM_GEN_ACTION_LOW)));

    // Init State
    servos[i].current_us = 1500; // Center
    servos[i].target_us = 1500;
    servos[i].speed = DEFAULT_SPEED;
  }

  ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

// --- MAIN TASK ---
void app_main(void) {
  motion_mutex = xSemaphoreCreateMutex();
  setup_mcpwm();

  // 1. Create a High Precision Timer for the Motion Loop
  // This replaces the "Loop()" delay in Arduino
  const esp_timer_create_args_t periodic_timer_args = {
      .callback = &motion_timer_callback, .name = "motion_loop"};
  esp_timer_handle_t periodic_timer;
  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
  // Start timer (microseconds) -> 200Hz = 5000us interval
  ESP_ERROR_CHECK(
      esp_timer_start_periodic(periodic_timer, 1000000 / CONTROL_FREQ_HZ));

  // 2. Setup UART (Interrupt Driven)
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

  ESP_LOGI(TAG, "Ready. Send: A,1500,1500,1500 (us)");

  uint8_t *data = (uint8_t *)malloc(128);
  char line[64];
  int line_idx = 0;

  while (1) {
    // Blocks until data arrives
    int len = uart_read_bytes(UART_NUM_0, data, 128, pdMS_TO_TICKS(100));

    for (int i = 0; i < len; i++) {
      char c = (char)data[i];
      if (c == '\n') {
        line[line_idx] = 0;
        // Parse "A,1000,1500,2000"
        if (line[0] == 'A') {
          float t1, t2, t3;
          if (sscanf(line, "A,%f,%f,%f", &t1, &t2, &t3) == 3) {
            xSemaphoreTake(motion_mutex, portMAX_DELAY);

            // --- SAFETY CLAMPS (ADD THIS BACK) ---
            // Protect your servo gears!
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
            // -------------------------------------

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
