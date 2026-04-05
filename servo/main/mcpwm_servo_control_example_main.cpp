#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "ServoPWM";

// --- EXPANDED TO 5 SERVOS ---
#define NUM_SERVOS 5
const int SERVO_PINS[NUM_SERVOS] = {2, 4, 5, 18, 19}; 

#define SERVO_MIN_US 550
#define SERVO_MAX_US 2400
#define SERVO_MIN_DEG 0.0f
#define SERVO_MAX_DEG 180.0f

#define UART_NUM UART_NUM_0
#define BUF_SIZE 256

typedef struct {
    float current_deg;
    float target_deg;
    mcpwm_cmpr_handle_t comparator;
} ServoState;

ServoState servos[NUM_SERVOS];

float deg_to_us(float deg) {
    deg = fmaxf(SERVO_MIN_DEG, fminf(SERVO_MAX_DEG, deg));
    return (deg - SERVO_MIN_DEG) * (SERVO_MAX_US - SERVO_MIN_US) /
           (SERVO_MAX_DEG - SERVO_MIN_DEG) + SERVO_MIN_US;
}

// S-curve (smoothstep) interpolation for smooth servo movement
float s_curve_interp(float t) {
    // Clamp t to [0,1]
    t = fmaxf(0.0f, fminf(1.0f, t));
    // Smoothstep: 3t^2 - 2t^3
    return t * t * (3.0f - 2.0f * t);
}

void update_servos() {
    const float alpha = 0.15f; // Lower alpha for smoother motion
    for (int i = 0; i < NUM_SERVOS; i++) {
        float delta = servos[i].target_deg - servos[i].current_deg;
        if (fabsf(delta) < 0.01f) continue; // Already at target
        float t = alpha;
        float s = s_curve_interp(t);
        servos[i].current_deg += delta * s;
        uint32_t us = (uint32_t)deg_to_us(servos[i].current_deg);
        mcpwm_comparator_set_compare_value(servos[i].comparator, us);
    }
}

// Parse expanded command: POS:deg0,deg1,deg2,deg3,deg4\n
void process_command(const char *line) {
    if (strncmp(line, "POS:", 4) == 0) {
        float degs[NUM_SERVOS];
        // Read 5 values now
        if (sscanf(line+4, "%f,%f,%f,%f,%f", &degs[0], &degs[1], &degs[2], &degs[3], &degs[4]) == 5) {
            for (int i = 0; i < NUM_SERVOS; i++) {
                servos[i].target_deg = fmaxf(SERVO_MIN_DEG, fminf(SERVO_MAX_DEG, degs[i]));
            }
        }
    }
}

void uart_rx_task(void *arg) {
    uint8_t buf[BUF_SIZE];
    char line[128]; // Slightly larger buffer for longer POS string
    int pos = 0;
    while (1) {
        int len = uart_read_bytes(UART_NUM, buf, BUF_SIZE, 10 / portTICK_PERIOD_MS);
        for (int i = 0; i < len; i++) {
            char c = (char)buf[i];
            if (c == '\n' || c == '\r') {
                if (pos > 0) {
                    line[pos] = '\0';
                    process_command(line);
                    pos = 0;
                }
            } else if (pos < (int)sizeof(line)-1) {
                line[pos++] = c;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void setup_mcpwm() {
    // We need 2 timers because an ESP32 MCPWM group only holds 3 operators.
    mcpwm_timer_handle_t timers[2] = {NULL, NULL};
    
    for (int i = 0; i < NUM_SERVOS; i++) {
        // Group 0 for pins 2, 4, 5. Group 1 for pins 18, 19.
        int group_id = i / 3; 

        if (timers[group_id] == NULL) {
            mcpwm_timer_config_t timer_config = {};
            timer_config.group_id = group_id;
            timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
            timer_config.resolution_hz = 1000000;
            timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
            timer_config.period_ticks = 20000; // 50Hz
            ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timers[group_id]));
        }

        mcpwm_oper_handle_t oper = NULL;
        mcpwm_operator_config_t oper_config = {};
        oper_config.group_id = group_id;
        ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &oper));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timers[group_id]));

        mcpwm_comparator_config_t cmp_config = {};
        cmp_config.flags.update_cmp_on_tez = true;
        ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmp_config, &servos[i].comparator));

        mcpwm_gen_handle_t generator = NULL;
        mcpwm_generator_config_t gen_config = {};
        gen_config.gen_gpio_num = SERVO_PINS[i];
        ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_config, &generator));

        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
            generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                    MCPWM_TIMER_EVENT_EMPTY,
                                                    MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
            generator, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                      servos[i].comparator,
                                                      MCPWM_GEN_ACTION_LOW)));

        servos[i].current_deg = 90.0f;
        servos[i].target_deg = 90.0f;
        mcpwm_comparator_set_compare_value(servos[i].comparator, (uint32_t)deg_to_us(90.0f));
    }

    // Enable and start both timers
    for (int g = 0; g < 2; g++) {
        if (timers[g] != NULL) {
            ESP_ERROR_CHECK(mcpwm_timer_enable(timers[g]));
            ESP_ERROR_CHECK(mcpwm_timer_start_stop(timers[g], MCPWM_TIMER_START_NO_STOP));
        }
    }
}

void setup_uart() {
    uart_config_t uart_config = {};
    uart_config.baud_rate = 250000; 
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void servo_update_task(void *arg) {
    while (1) {
        update_servos();
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz
    }
}

extern "C" void app_main(void) {
    setup_mcpwm();
    setup_uart();
    xTaskCreate(uart_rx_task, "uart_rx_task", 2048, NULL, 10, NULL);
    xTaskCreate(servo_update_task, "servo_update_task", 2048, NULL, 10, NULL);
    
    ESP_LOGI(TAG, "Servo PWM UART Controller started. 5 Servos active.");
    
    // --- SEND HANDSHAKE TO PYTHON SCRIPT ---
    const char* ready_msg = "READY\n";
    uart_write_bytes(UART_NUM, ready_msg, strlen(ready_msg));
}