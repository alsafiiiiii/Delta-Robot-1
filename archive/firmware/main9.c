/**
 * @file main9.c (ARCHIVED - BROKEN)
 * @brief Slew Rate Limiter Attempt (Missing Declarations)
 *
 * CHANGES FROM main8.c:
 * - Replaced EMA with Slew Rate Limiter (0.6 m/s max)
 * - Linear approach to target at constant speed
 * - Corrected dt = 0.02f
 *
 * BUG: Missing 'static cartesian_packet_t smoothed_target' and
 *      'static bool first_run' declarations. WILL NOT COMPILE.
 *      Use mcpwm_servo_control_example_main.c for fixed version.
 */
#include "driver/mcpwm_prelude.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "nvs_flash.h"
#include <errno.h>
#include <lwip/netdb.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "delta_ik_driver";

// --- CONFIGURATION ---
#define WIFI_SSID "Galaxy S25 Ultra 7880"
#define WIFI_PASS "12345678"
#define PORT 3333
#define NUM_SERVOS 3

// --- ROBOT GEOMETRY (Meters) ---
// Based on: RobotDelta(np.array([0.104, 0.040, 0.105, 0.205]))
#define R_BASE 0.104f
#define R_END 0.040f
#define L_UPPER 0.105f
#define L_LOWER 0.205f

// --- MOTION SETTINGS ---
// Tuned for CARTESIAN STREAMING
// --- MOTION SETTINGS ---
// Tuned for CARTESIAN STREAMING
#define TRAJ_MAX_VEL 3.5f // m/s (Increased for responsive motion)
#define TRAJ_ACCEL 8.0f   // m/s^2 (More responsive)
#define TRAJ_DECEL 8.0f   // m/s^2

// ...

// (Stray code removed)

#define SERVO_MIN_PULSEWIDTH_US 500
#define SERVO_MAX_PULSEWIDTH_US 2400
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000
#define SERVO_TIMEBASE_PERIOD 20000

static const int SERVO_GPIOS[NUM_SERVOS] = {2, 4, 5};

// --- DATA STRUCTURES ---
// New Packet: Cartesian Target
typedef struct {
  float x;
  float y;
  float z;
  uint8_t mode; // 0 = Linear (Cartesian Interp), 1 = Joint (Angle Interp)
} __attribute__((packed)) cartesian_packet_t;

// --- GLOBALS ---
mcpwm_cmpr_handle_t comparators[NUM_SERVOS];
SemaphoreHandle_t target_mutex;

// Shared Data
cartesian_packet_t current_target = {0.0f, 0.0f, -0.22f}; // Home Z
bool new_data_available = false;

// --- INVERSE KINEMATICS (Visual Kinematics Port) ---
// Exact match of RobotDelta from Python library
// Params derived from dump_vk_params.py
static const float L1 = 0.105f;
static const float L2 = 0.205f;
static const float R1 = 0.104f;

// Attachment Points (AP) - X, Y coordinates (Z is 0)
// Columns relate to Arms 0, 1, 2
static const float AP_X[3] = {-0.04f, 0.02f, 0.02f};
static const float AP_Y[3] = {0.0f, -0.03464102f, 0.03464102f};

// Precomputed Cos/Sin Phi
static const float COS_PHI[3] = {1.0f, -0.5f, -0.5f};
static const float SIN_PHI[3] = {0.0f, 0.866025403f, -0.866025403f};

static float simplify_angle(float angle) {
  while (angle <= -M_PI)
    angle += 2 * M_PI;
  while (angle > M_PI)
    angle -= 2 * M_PI;
  return angle * 180.0f / M_PI; // Return Degrees
}

int delta_calcInverse(float x, float y, float z, float *t1, float *t2,
                      float *t3) {
  // Solve for each arm
  float theta[3];

  for (int i = 0; i < 3; i++) {
    // oa = op - ap_i
    // op is target(x,y,z)
    float oa_x = x - AP_X[i];
    float oa_y = y - AP_Y[i];
    float oa_z = z - 0.0f;

    float norm_oa_sq = oa_x * oa_x + oa_y * oa_y + oa_z * oa_z;

    // a = 2*l1*z
    float a = 2.0f * L1 * z;

    // b = ...
    float cp = COS_PHI[i];
    float sp = SIN_PHI[i];

    float term1 = (R1 * cp) - oa_x;
    float term2 = (R1 * sp) - oa_y;

    float b = 2.0f * L1 * (cp * term1 + sp * term2);

    // c = ...
    float c = (L2 * L2) - (L1 * L1) - norm_oa_sq - (R1 * R1) +
              2.0f * R1 * (cp * oa_x + sp * oa_y);

    // Solve a*sin + b*cos = c
    // theta = atan2(c, -sqrt(a*a+b*b-c*c)) - atan2(b, a)

    float disc = a * a + b * b - c * c;
    if (disc < 0) {
      return -1; // Unreachable
    }

    float val = atan2f(c, -sqrtf(disc)) - atan2f(b, a);
    theta[i] = simplify_angle(val);
  }

  *t1 = theta[0];
  *t2 = theta[1];
  *t3 = theta[2];

  return 0;
}

// --- UTILS ---
static inline uint32_t angle_to_compare(float angle) {
  // Servo Mounting Correction
  // VK Output: 25-45 deg (Down from Horiz? Up?)
  // User system Home: 90 deg = Active Arm Horiz? Or Vert?
  // User verified in sim: 42 deg.
  // Standard servo: 90 deg is center.
  // User says: "Zero degrees is when its flat pointing outward"
  // IK Output: 0 deg = Flat.
  // Servo Input: 0 deg = Flat (500us).
  // Logic: Direct Mapping (No Offset).

  float servo_ang = angle + 0.0f; // Was + 90.0f
  // CLAMPING:
  if (servo_ang < 0.0f)
    servo_ang = 0.0f;
  if (servo_ang > 180.0f)
    servo_ang = 180.0f;

  return (uint32_t)((servo_ang) *
                        (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) /
                        180 +
                    SERVO_MIN_PULSEWIDTH_US);
}

// --- TASK 1: UDP RECEIVER ---
static void udp_server_task(void *pvParameters) {
  cartesian_packet_t packet;
  struct sockaddr_in dest_addr;
  dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(PORT);

  int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
  ESP_LOGI(TAG, "UDP Listening (IK Mode: X,Y,Z)...");

  while (1) {
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);

    // Blocking Receive (X, Y, Z floats)
    int len = recvfrom(sock, &packet, sizeof(cartesian_packet_t), 0,
                       (struct sockaddr *)&source_addr, &socklen);

    if (len == sizeof(cartesian_packet_t)) {
      xSemaphoreTake(target_mutex, portMAX_DELAY);
      current_target = packet;
      new_data_available = true;
      xSemaphoreGive(target_mutex);
    }
  }
}

// --- TASK 2: MOTION LOOP ---
static void motion_task(void *pvParameters) {
  // 50Hz Loop (Matches Servo PWM Period)
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20);

  cartesian_packet_t actual = {0.0f, 0.0f, -0.22f}; // Current smoothed pos

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // 1. Get Target
    cartesian_packet_t target;
    bool valid_data = false;

    xSemaphoreTake(target_mutex, portMAX_DELAY);
    if (new_data_available) {
      target = current_target;
      new_data_available =
          false; // Consumption flag (optional, allows timeout logic)
      valid_data = true;
    } else {
      // Keep using last target, but check for timeout?
      // Actually, let's track last_packet_time in a cleaner way.
      target = current_target;
    }
    xSemaphoreGive(target_mutex);

    // SAFETY: Connection Timeout
    // If we haven't received a UDP packet in >500ms, stop interpolation to
    // avoid runaway (Note: Requires a timestamp tracker in UDP task. For now,
    // simple logic:) If no new data, we just hold position. This IS safe for a
    // position-controlled robot. Optimization: Just ensure we don't drift.

    // 2. Interpolate (Linear or Joint)
    // dt = 0.02s (20ms)
    float dt = 0.02f;
    float max_step = TRAJ_MAX_VEL * dt;
    float dist = 0.0f; // Shared for logging

    // --- SLEW RATE LIMITER (Constant Velocity Smoothing) ---
    // Instead of EMA (curved approach), we move linearly towards target at
    // fixed speed. This hides UDP jitter by enforcing a steady pace.

    // Servo Limit: 0.6 m/s (Do not exceed)
    float slew_speed = 0.6f;
    float max_slew_step = slew_speed * dt;

    // Calculate vector to target
    float dx = target.x - smoothed_target.x;
    float dy = target.y - smoothed_target.y;
    float dz = target.z - smoothed_target.z;
    float dist_to_target = sqrtf(dx * dx + dy * dy + dz * dz);

    // 1. Detect large jump - Reset Logic
    if (first_run || dist_to_target > 0.05f) { // 50mm jump = reset
      smoothed_target = target;
      first_run = false;
      ESP_LOGI(TAG, "Slew Reset (jump: %.3fm)", dist_to_target);
    } else {
      // 2. Apply Slew Rate Limit
      if (dist_to_target > max_slew_step) {
        // Move towards target by max_slew_step
        float ratio = max_slew_step / dist_to_target;
        smoothed_target.x += dx * ratio;
        smoothed_target.y += dy * ratio;
        smoothed_target.z += dz * ratio;
      } else {
        // Close enough - snap to target
        smoothed_target = target;
      }
    }

    // Use smoothed_target DIRECTLY (no ESP32 interpolation)
    // Python controller handles linear interpolation at safe speed (0.15 m/s)
    // ESP32 just executes: smooth input → IK → servo
    actual = smoothed_target;

    // 3. Solve IK
    float t1, t2, t3;
    if (delta_calcInverse(actual.x, actual.y, actual.z, &t1, &t2, &t3) == 0) {
      // 4. Drive Servos Directly (No Per-Axis Ramping)
      // Cartesian interpolation already handles smoothness.
      // Coordinated 3-axis motion → no jerk
      mcpwm_comparator_set_compare_value(comparators[0], angle_to_compare(t1));
      mcpwm_comparator_set_compare_value(comparators[1], angle_to_compare(t2));
      mcpwm_comparator_set_compare_value(comparators[2], angle_to_compare(t3));

      // 5. Smart Logging (Only on Change)
      // Calculate instantaneous speed
      float current_speed = 0.0f;
      if (dist > 1e-6) {
        current_speed = (dist > max_step) ? TRAJ_MAX_VEL : (dist / dt);
      }

      static cartesian_packet_t last_logged_pos = {0.0f, 0.0f, -0.22f};
      float chg_x = actual.x - last_logged_pos.x;
      float chg_y = actual.y - last_logged_pos.y;
      float chg_z = actual.z - last_logged_pos.z;
      float change_mag = sqrtf(chg_x * chg_x + chg_y * chg_y + chg_z * chg_z);

      // Log if moved > 1mm OR every ~5s heartbeat
      static int heartbeat = 0;
      if (change_mag > 0.001f || ++heartbeat > 250) {
        ESP_LOGI(TAG,
                 "Mov[50Hz]: Pos(%.3f, %.3f, %.3f) | Spd: %.2fm/s | Ang(%.1f, "
                 "%.1f, %.1f)",
                 actual.x, actual.y, actual.z, current_speed, t1, t2, t3);
        last_logged_pos = actual;
        heartbeat = 0;
      }
    } else {
      // Out of workspace? Ignore?
      // ESP_LOGW(TAG, "IK Fail");
    }
  }
}

// --- BOILERPLATE WIFI & SETUP ---
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
  target_mutex = xSemaphoreCreateMutex();
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

  wifi_config_t w_cfg = {};
  strcpy((char *)w_cfg.sta.ssid, WIFI_SSID);
  strcpy((char *)w_cfg.sta.password, WIFI_PASS);
  w_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_config(WIFI_IF_STA, &w_cfg);
  esp_wifi_start();
  esp_wifi_set_ps(WIFI_PS_NONE);

  xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE,
                      portMAX_DELAY);

  // MCPWM Init
  mcpwm_timer_handle_t timer = NULL;
  mcpwm_timer_config_t t_cfg = {.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
                                .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
                                .period_ticks = SERVO_TIMEBASE_PERIOD,
                                .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
                                .group_id = 0};
  ESP_ERROR_CHECK(mcpwm_new_timer(&t_cfg, &timer));

  for (int i = 0; i < NUM_SERVOS; i++) {
    mcpwm_oper_handle_t oper;
    mcpwm_operator_config_t o_cfg = {.group_id = 0};
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

    // Init Center
    ESP_ERROR_CHECK(
        mcpwm_comparator_set_compare_value(cmpr, angle_to_compare(0)));
  }

  ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

  // Priority Boost: High (above WiFi/IP which are usually 18-23 on ESP32,
  // but configMAX_PRIORITIES is often 25. Let's set to safe High value).
  // Standard FreeRTOS max is often 25.
  xTaskCreate(motion_task, "motion", 4096, NULL, 20, NULL);
  xTaskCreate(udp_server_task, "udp", 4096, NULL, 5, NULL);
}
