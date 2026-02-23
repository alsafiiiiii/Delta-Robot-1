#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

// Define Pins
#define MOTOR_ENA 4
#define MOTOR_IN1 5
#define MOTOR_IN2 6

// PWM Settings
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // 0 to 8191
#define LEDC_FREQUENCY (5000)           // 5 kHz PWM

void init_motor() {
  // Configure Direction Pins
  gpio_reset_pin(MOTOR_IN1);
  gpio_set_direction(MOTOR_IN1, GPIO_MODE_OUTPUT);
  gpio_reset_pin(MOTOR_IN2);
  gpio_set_direction(MOTOR_IN2, GPIO_MODE_OUTPUT);

  // Configure PWM Timer
  ledc_timer_config_t ledc_timer = {.speed_mode = LEDC_MODE,
                                    .timer_num = LEDC_TIMER,
                                    .duty_resolution = LEDC_DUTY_RES,
                                    .freq_hz = LEDC_FREQUENCY,
                                    .clk_cfg = LEDC_AUTO_CLK};
  ledc_timer_config(&ledc_timer);

  // Configure PWM Channel
  ledc_channel_config_t ledc_channel = {.speed_mode = LEDC_MODE,
                                        .channel = LEDC_CHANNEL,
                                        .timer_sel = LEDC_TIMER,
                                        .intr_type = LEDC_INTR_DISABLE,
                                        .gpio_num = MOTOR_ENA,
                                        .duty = 0,
                                        .hpoint = 0};
  ledc_channel_config(&ledc_channel);
}

void set_motor_speed(int speed) {
  if (speed > 0) {
    gpio_set_level(MOTOR_IN1, 1);
    gpio_set_level(MOTOR_IN2, 0);
  } else if (speed < 0) {
    gpio_set_level(MOTOR_IN1, 0);
    gpio_set_level(MOTOR_IN2, 1);
    speed = -speed; // Make positive for PWM
  } else {
    gpio_set_level(MOTOR_IN1, 0);
    gpio_set_level(MOTOR_IN2, 0);
  }

  ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, speed);
  ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

void app_main(void) {
  init_motor();

  while (1) {
    printf("Moving Forward at 50%% speed...\n");
    set_motor_speed(4000); // ~50% of 8191
    vTaskDelay(pdMS_TO_TICKS(2000));

    printf("Stopping...\n");
    set_motor_speed(0);
    vTaskDelay(pdMS_TO_TICKS(1000));

    printf("Moving Backward at 75%% speed...\n");
    set_motor_speed(-6000);
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}