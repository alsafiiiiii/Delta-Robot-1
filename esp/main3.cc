#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/sockets.h"

static const char *TAG = "delta_physics_driver";

// --- CONFIGURATION ---
#define WIFI_SSID      "Galaxy S25 Ultra 7880"
#define WIFI_PASS      "12345678"
#define PORT           3333
#define NUM_SERVOS     3 
#define MIN_STEP_DEG   0.02f // Finer control for physics

// --- PHYSICS CONSTANTS ---
// Adjust OMEGA to change speed/stiffness.
// 10.0 = Slow/Heavy, 25.0 = Fast/Snappy
#define OMEGA          15.0f 
#define ZETA           1.0f   // 1.0 = Critical Damping (No Overshoot)
#define MAX_VEL        300.0f // Degrees per second limit

// --- BUFFER SETTINGS ---
#define BUFFER_SIZE     8  // Smaller buffer needed for real-time physics 
#define STARTUP_COUNT   2  

// --- TIMING ---
#define MOTION_FREQ_HZ  50    
#define DT              (1.0f / MOTION_FREQ_HZ)

static const int SERVO_GPIOS[NUM_SERVOS] = {2, 4, 5}; 

#define SERVO_MIN_PULSEWIDTH_US 500
#define SERVO_MAX_PULSEWIDTH_US 2400
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000 
#define SERVO_TIMEBASE_PERIOD        20000

typedef struct { float angles[5]; } delta_packet_t;
mcpwm_cmpr_handle_t comparators[NUM_SERVOS];

// Struct for Physics State
typedef struct {
    float pos;
    float vel;
} servo_state_t;

servo_state_t servos[NUM_SERVOS];
float target_angles[NUM_SERVOS];

// --- RING BUFFER ---
delta_packet_t buffer[BUFFER_SIZE];
volatile int head = 0;
volatile int tail = 0;
volatile int count = 0;

static inline uint32_t angle_to_compare(float angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    uint32_t us = (angle) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / 180 + SERVO_MIN_PULSEWIDTH_US;
    return us; // 1MHz = 1 tick per us
}

static float clamp(float v, float min, float max) {
    if (v < min) return min;
    if (v > max) return max;
    return v;
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
    
    ESP_LOGI(TAG, "Physics Engine Listening...");

    while (1) {
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(sock, &packet, sizeof(delta_packet_t), 0, (struct sockaddr *)&source_addr, &socklen);
        
        if (len == sizeof(delta_packet_t)) {
            // For physics, we want the LATEST target mostly.
            // But buffering smoothens jitter.
            if (count < BUFFER_SIZE) {
                buffer[head] = packet;
                head = (head + 1) % BUFFER_SIZE;
                count++;
            } else {
                // Drop oldest if full (Always keep fresh data)
                tail = (tail + 1) % BUFFER_SIZE;
                buffer[head] = packet;
                head = (head + 1) % BUFFER_SIZE;
            }
        }
    }
}

// --- TASK 2: PHYSICS ENGINE ---
static void motion_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / MOTION_FREQ_HZ); 

    // Init State
    for(int i=0; i<NUM_SERVOS; i++) { 
        servos[i].pos = 90.0f;
        servos[i].vel = 0.0f;
        target_angles[i] = 90.0f;
    }
    
    bool active = false;

    while(1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency); 

        // 1. Fetch Target
        if (!active) {
            if (count >= STARTUP_COUNT) {
                 active = true;
            }
        }
        
        if (active && count > 0) {
            // Unspool buffer
            // In linear interp, we popped one by one.
            // In physics, we update the target. Ideally we process 1 packet per tick if available.
            delta_packet_t pkt = buffer[tail];
            tail = (tail + 1) % BUFFER_SIZE;
            count--;
            
            for(int i=0; i<NUM_SERVOS; i++) target_angles[i] = pkt.angles[i];
            
            ESP_LOGD(TAG, "New Target: %.2f %.2f %.2f", target_angles[0], target_angles[1], target_angles[2]);
        }

        // 2. Physics Update Step (Mass-Spring-Damper)
        // F = -k*x - c*v
        // a = F / m
        // k (spring) ~ omega^2
        // c (damper) ~ 2 * zeta * omega
        
        float spring_k = OMEGA * OMEGA;
        float damper_c = 2.0f * ZETA * OMEGA;

        for (int i=0; i<NUM_SERVOS; i++) {
            float err = target_angles[i] - servos[i].pos;
            
            // Force = Spring Force - Damping Force
            float accel = (err * spring_k) - (servos[i].vel * damper_c);
            
            // Integrate Velocity
            servos[i].vel += accel * DT;
            
            // Clamp Velocity (Safety)
            servos[i].vel = clamp(servos[i].vel, -MAX_VEL, MAX_VEL);
            
            // Integrate Position
            servos[i].pos += servos[i].vel * DT;
            
            // Write to Servo
            mcpwm_comparator_set_compare_value(comparators[i], angle_to_compare(servos[i].pos));
        }
    }
}

// (Standard WiFi Boilerplate)
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
static void wifi_handler(void* arg, esp_event_base_t base, int32_t id, void* data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) esp_wifi_connect();
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) esp_wifi_connect();
}

void app_main(void) {
    nvs_flash_init();
    s_wifi_event_group = xEventGroupCreate();
    esp_netif_init(); esp_event_loop_create_default(); esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); esp_wifi_init(&cfg);
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_handler, NULL, NULL);
    wifi_config_t w_cfg = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS } };
    esp_wifi_set_mode(WIFI_MODE_STA); esp_wifi_set_config(WIFI_IF_STA, &w_cfg); esp_wifi_start();
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    // MCPWM Setup
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t t_cfg = { 
        .clk_src=MCPWM_TIMER_CLK_SRC_DEFAULT, 
        .resolution_hz=1000000,  // 1MHz (Standard)
        .period_ticks=20000,     // 20ms Period (50Hz)
        .count_mode=MCPWM_TIMER_COUNT_MODE_UP 
    };
    t_cfg.group_id=0; 
    ESP_ERROR_CHECK(mcpwm_new_timer(&t_cfg, &timer));

    for (int i=0; i<NUM_SERVOS; i++) {
        mcpwm_oper_handle_t oper; 
        mcpwm_operator_config_t o_cfg = { .group_id=0 }; 
        ESP_ERROR_CHECK(mcpwm_new_operator(&o_cfg, &oper)); 
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));
        
        mcpwm_cmpr_handle_t cmpr; 
        mcpwm_comparator_config_t c_cfg = { .flags.update_cmp_on_tez=true };
        ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &c_cfg, &cmpr)); 
        comparators[i]=cmpr;
        
        mcpwm_gen_handle_t gen; 
        mcpwm_generator_config_t g_cfg = { .gen_gpio_num=SERVO_GPIOS[i] };
        ESP_ERROR_CHECK(mcpwm_new_generator(oper, &g_cfg, &gen));
        
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr, MCPWM_GEN_ACTION_LOW)));
        
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpr, angle_to_compare(90)));
    }
    
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer)); 
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    // Higher stack size for physics calculations if needed
    xTaskCreate(motion_task, "physics", 4096, NULL, 10, NULL);
    xTaskCreate(udp_server_task, "udp", 4096, NULL, 5, NULL);
}
