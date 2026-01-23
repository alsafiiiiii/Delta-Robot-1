/**
 * Standalone Star Test v2 - Dense Precomputed Trajectory
 *
 * Uses 262 points with Quintic velocity profile along linear Cartesian paths.
 * NO serial communication - pure hardware test.
 */

#include "driver/mcpwm_prelude.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "StarTestV2";

// Include precomputed trajectory
#include "trajectory_data.h"

// --- CONFIGURATION ---
#define NUM_SERVOS 3
const int SERVO_PINS[NUM_SERVOS] = {2, 4, 5};

// Servo Calibration (0-180 deg = 550-2400 us)
#define SERVO_MIN_US 550
#define SERVO_MAX_US 2400
#define SERVO_MIN_DEG 0.0f
#define SERVO_MAX_DEG 180.0f

// --- DATA STRUCTURES ---
typedef struct {
  float current_us;
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

// --- HARDWARE SETUP ---
static void setup_mcpwm() {
  ESP_LOGI(TAG, "Initializing MCPWM...");

  mcpwm_timer_handle_t timer = NULL;
  mcpwm_timer_config_t timer_config = {
      .group_id = 0,
      .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
      .resolution_hz = 1000000,
      .period_ticks = 5000, // 50Hz standard
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

    servos[i].current_us = 1500;
  }

  ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

// --- MAIN ---
void app_main(void) {
  setup_mcpwm();

  ESP_LOGI(TAG, "Loaded trajectory with %d points", TRAJECTORY_POINTS);
  ESP_LOGI(TAG, "Point delay: %d ms, Total cycle: %.1f sec", POINT_DELAY_MS,
           (float)TRAJECTORY_POINTS * POINT_DELAY_MS / 1000.0f);
  ESP_LOGI(TAG, "Starting dense trajectory playback...");

  int point_index = 0;

  while (1) {
    // Get target angles from precomputed trajectory
    float deg0 = trajectory[point_index][0];
    float deg1 = trajectory[point_index][1];
    float deg2 = trajectory[point_index][2];

    // Convert to microseconds and write directly
    uint32_t us0 = (uint32_t)deg_to_us(deg0);
    uint32_t us1 = (uint32_t)deg_to_us(deg1);
    uint32_t us2 = (uint32_t)deg_to_us(deg2);

    mcpwm_comparator_set_compare_value(servos[0].comparator, us0);
    mcpwm_comparator_set_compare_value(servos[1].comparator, us1);
    mcpwm_comparator_set_compare_value(servos[2].comparator, us2);

    // Next point
    point_index = (point_index + 1) % TRAJECTORY_POINTS;

    // Wait before next update
    vTaskDelay(pdMS_TO_TICKS(POINT_DELAY_MS));
  }
}
