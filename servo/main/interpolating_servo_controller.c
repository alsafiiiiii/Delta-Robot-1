/**
 * Interpolating Servo Controller
 *
 * Receives target angles + speed over Serial.
 * Generates smooth quintic trajectory internally.
 * Updates servos at 200Hz for jitter-free motion.
 *
 * Protocol:
 *   M<j1>,<j2>,<j3>,<speed>\n   Move to angles at speed (deg/sec)
 *   H\n                         Home (go to 45,45,45)
 *   S\n                         Stop (hold position)
 */

#include "driver/mcpwm_prelude.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "InterpServo";

// --- CONFIGURATION ---
#define NUM_SERVOS 3
#define UART_NUM UART_NUM_0
#define BUF_SIZE 256

const int SERVO_PINS[NUM_SERVOS] = {2, 4, 5};

// Servo Calibration
#define SERVO_MIN_US 550
#define SERVO_MAX_US 2400
#define SERVO_MIN_DEG 0.0f
#define SERVO_MAX_DEG 180.0f

// Motion Settings
#define CONTROL_FREQ_HZ 200
#define DEFAULT_SPEED 30.0f // deg/sec if not specified
#define HOME_ANGLE 45.0f

// --- DATA STRUCTURES ---
typedef struct {
  float start[NUM_SERVOS];
  float end[NUM_SERVOS];
  float duration; // seconds
  float elapsed;  // seconds
  bool active;
} Motion;

typedef struct {
  float current_us;
  mcpwm_cmpr_handle_t comparator;
} ServoState;

// --- GLOBALS ---
ServoState servos[NUM_SERVOS];
Motion current_motion = {0};
SemaphoreHandle_t motion_mutex;

// --- HELPERS ---
float deg_to_us(float deg) {
  deg = fmaxf(SERVO_MIN_DEG, fminf(SERVO_MAX_DEG, deg));
  return (deg - SERVO_MIN_DEG) * (SERVO_MAX_US - SERVO_MIN_US) /
             (SERVO_MAX_DEG - SERVO_MIN_DEG) +
         SERVO_MIN_US;
}

float us_to_deg(float us) {
  us = fmaxf(SERVO_MIN_US, fminf(SERVO_MAX_US, us));
  return (us - SERVO_MIN_US) * (SERVO_MAX_DEG - SERVO_MIN_DEG) /
             (SERVO_MAX_US - SERVO_MIN_US) +
         SERVO_MIN_DEG;
}

// Quintic interpolation (0 vel, 0 accel at endpoints)
float quintic(float t) {
  // t is normalized [0, 1]
  // Returns position [0, 1]
  return 10 * t * t * t - 15 * t * t * t * t + 6 * t * t * t * t * t;
}

// --- COMMAND PARSER ---
void start_move(float j1, float j2, float j3, float speed) {
  xSemaphoreTake(motion_mutex, portMAX_DELAY);

  // Record current positions as start
  current_motion.start[0] = us_to_deg(servos[0].current_us);
  current_motion.start[1] = us_to_deg(servos[1].current_us);
  current_motion.start[2] = us_to_deg(servos[2].current_us);

  // Set targets
  current_motion.end[0] = j1;
  current_motion.end[1] = j2;
  current_motion.end[2] = j3;

  // Calculate duration based on speed and max delta
  float max_delta = 0.0f;
  for (int i = 0; i < NUM_SERVOS; i++) {
    float delta = fabsf(current_motion.end[i] - current_motion.start[i]);
    if (delta > max_delta)
      max_delta = delta;
  }

  current_motion.duration = (speed > 0.1f) ? (max_delta / speed) : 0.1f;
  if (current_motion.duration < 0.05f)
    current_motion.duration = 0.05f; // Min 50ms

  current_motion.elapsed = 0.0f;
  current_motion.active = true;

  ESP_LOGI(TAG,
           "Move: [%.1f,%.1f,%.1f] -> [%.1f,%.1f,%.1f] @ %.1f deg/s (%.2fs)",
           current_motion.start[0], current_motion.start[1],
           current_motion.start[2], j1, j2, j3, speed, current_motion.duration);

  xSemaphoreGive(motion_mutex);
}

void parse_command(char *cmd) {
  // Remove newline
  char *nl = strchr(cmd, '\n');
  if (nl)
    *nl = '\0';

  if (cmd[0] == 'M' || cmd[0] == 'm') {
    float j1, j2, j3, speed = DEFAULT_SPEED;
    int parsed = sscanf(cmd + 1, "%f,%f,%f,%f", &j1, &j2, &j3, &speed);
    if (parsed >= 3) {
      start_move(j1, j2, j3, speed);
    } else {
      ESP_LOGW(TAG, "Invalid M command: %s", cmd);
    }
  } else if (cmd[0] == 'H' || cmd[0] == 'h') {
    start_move(HOME_ANGLE, HOME_ANGLE, HOME_ANGLE, DEFAULT_SPEED);
  } else if (cmd[0] == 'S' || cmd[0] == 's') {
    xSemaphoreTake(motion_mutex, portMAX_DELAY);
    current_motion.active = false;
    xSemaphoreGive(motion_mutex);
    ESP_LOGI(TAG, "Stop");
  }
}

// --- MOTION TIMER (200Hz) ---
static void motion_timer_callback(void *arg) {
  float dt = 1.0f / CONTROL_FREQ_HZ;

  xSemaphoreTake(motion_mutex, portMAX_DELAY);

  if (current_motion.active) {
    current_motion.elapsed += dt;

    // Normalize time
    float t = current_motion.elapsed / current_motion.duration;
    if (t >= 1.0f) {
      t = 1.0f;
      current_motion.active = false;
    }

    // Quintic interpolation
    float s = quintic(t);

    // Calculate and apply positions
    for (int i = 0; i < NUM_SERVOS; i++) {
      float angle = current_motion.start[i] +
                    (current_motion.end[i] - current_motion.start[i]) * s;
      servos[i].current_us = deg_to_us(angle);
      mcpwm_comparator_set_compare_value(servos[i].comparator,
                                         (uint32_t)servos[i].current_us);
    }
  }

  xSemaphoreGive(motion_mutex);
}

// --- SERIAL TASK ---
static void serial_task(void *pvParameters) {
  char buf[BUF_SIZE];
  int buf_pos = 0;

  while (1) {
    uint8_t byte;
    int len = uart_read_bytes(UART_NUM, &byte, 1, pdMS_TO_TICKS(10));

    if (len > 0) {
      if (byte == '\n' || byte == '\r') {
        if (buf_pos > 0) {
          buf[buf_pos] = '\0';
          parse_command(buf);
          buf_pos = 0;
        }
      } else if (buf_pos < BUF_SIZE - 1) {
        buf[buf_pos++] = byte;
      }
    }
  }
}

// --- HARDWARE SETUP ---
static void setup_mcpwm() {
  ESP_LOGI(TAG, "Initializing MCPWM...");

  mcpwm_timer_handle_t timer = NULL;
  mcpwm_timer_config_t timer_config = {
      .group_id = 0,
      .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
      .resolution_hz = 1000000,
      .period_ticks = 20000, // 50Hz PWM
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

    // Initialize to home angle
    servos[i].current_us = deg_to_us(HOME_ANGLE);
    mcpwm_comparator_set_compare_value(servos[i].comparator,
                                       (uint32_t)servos[i].current_us);
  }

  ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

static void setup_uart() {
  uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  };
  ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
}

// --- MAIN ---
void app_main(void) {
  motion_mutex = xSemaphoreCreateMutex();

  setup_uart();
  setup_mcpwm();

  // Start motion timer (200Hz)
  const esp_timer_create_args_t timer_args = {
      .callback = &motion_timer_callback, .name = "motion_loop"};
  esp_timer_handle_t motion_timer;
  ESP_ERROR_CHECK(esp_timer_create(&timer_args, &motion_timer));
  ESP_ERROR_CHECK(
      esp_timer_start_periodic(motion_timer, 1000000 / CONTROL_FREQ_HZ));

  // Start serial reader
  xTaskCreate(serial_task, "serial", 4096, NULL, 10, NULL);

  ESP_LOGI(TAG, "Interpolating Servo Controller Ready");
  ESP_LOGI(TAG, "Commands: M<j1>,<j2>,<j3>,<speed> | H (home) | S (stop)");
}
