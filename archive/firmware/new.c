/*
 * MERGED CONTROLLER: 3x High-Speed Sharp Sensors + Interpolated Servos
 * * HARDWARE MAPPING:
 * - Sharp Sensor 0: GPIO 34 (ADC1_CH6)
 * - Sharp Sensor 1: GPIO 35 (ADC1_CH7)
 * - Sharp Sensor 2: GPIO 32 (ADC1_CH4)
 * - Servos: GPIO 2, 4, 5
 * - UART: Default (TX=1, RX=3)
 */

#include "driver/mcpwm_prelude.h"
#include "driver/uart.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "RobotMain";

// =============================================================================
// --- CONFIGURATION: SENSORS ---
// =============================================================================
#define CALIBRATION_MODE 0 // 0 = Distance (mm), 1 = Raw Voltage Debug
#define NUM_SENSORS 3
#define FILTER_SIZE 50

// Sensor Pins & Channels (Modify based on your ESP32 board)
// NOTE: ADC1 is preferred for Wi-Fi coexistence if needed later.
const adc_channel_t SENSOR_CHANNELS[NUM_SENSORS] = {
    ADC_CHANNEL_6, // GPIO 34
    ADC_CHANNEL_7, // GPIO 35
    ADC_CHANNEL_4  // GPIO 32
};

#define ADC_UNIT ADC_UNIT_1
#define ADC_ATTEN ADC_ATTEN_DB_12

// =============================================================================
// --- CONFIGURATION: SERVOS ---
// =============================================================================
#define NUM_SERVOS 3
const int SERVO_PINS[NUM_SERVOS] = {2, 4, 5};
#define SERVO_MIN_US 550
#define SERVO_MAX_US 2400
#define SERVO_MIN_DEG 0.0f
#define SERVO_MAX_DEG 180.0f
#define HYBRID_FACTOR 0.8f
#define CONTROL_FREQ_HZ 50

// =============================================================================
// --- CONFIGURATION: UART ---
// =============================================================================
#define UART_NUM UART_NUM_0
#define BUF_SIZE 1024

// -----------------------------------------------------------------------------
// PART 1: SENSOR LOGIC
// -----------------------------------------------------------------------------
static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t cali_handle = NULL;
static QueueHandle_t sensor_queue = NULL;

// Lookup Tables for 3 sensors
static float lut_mv_to_mm[NUM_SENSORS][3301];

typedef struct {
  int voltage_mv[NUM_SENSORS];
  float distance_mm[NUM_SENSORS];
} sensor_packet_t;

// --- INDEPENDENT POLYNOMIALS PER SENSOR ---
// You can tweak these curves individually based on physical calibration
static float poly_sensor_0(double v) {
  // Example: Standard GP2Y0A21YK0F
  return (float)((-6.56E-08 * v * v * v) + (3.73E-04 * v * v) + (-0.774 * v) +
                 689.0);
}

static float poly_sensor_1(double v) {
  // Example: Slightly different calibration
  return (float)((-6.20E-08 * v * v * v) + (3.60E-04 * v * v) + (-0.750 * v) +
                 680.0);
}

static float poly_sensor_2(double v) {
  // Example: Long range sensor or different batch
  return (float)((-6.80E-08 * v * v * v) + (3.85E-04 * v * v) + (-0.800 * v) +
                 700.0);
}

// Function pointer array for clean code
typedef float (*poly_func_t)(double);
poly_func_t poly_funcs[NUM_SENSORS] = {poly_sensor_0, poly_sensor_1,
                                       poly_sensor_2};

void generate_luts() {
  ESP_LOGI(TAG, "Generating LUTs for %d sensors...", NUM_SENSORS);
  for (int s = 0; s < NUM_SENSORS; s++) {
    for (int mv = 0; mv <= 3300; mv++) {
      // Clamp voltages outside working range of Sharp sensors
      if (mv < 300)
        lut_mv_to_mm[s][mv] = 800.0f; // Max valid dist
      else if (mv > 3100)
        lut_mv_to_mm[s][mv] = 60.0f; // Min valid dist
      else
        lut_mv_to_mm[s][mv] = poly_funcs[s]((double)mv);
    }
  }
}

static bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten,
                                 adc_cali_handle_t *out_handle) {
  adc_cali_handle_t handle = NULL;
  esp_err_t ret = ESP_FAIL;
  bool calibrated = false;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  adc_cali_curve_fitting_config_t cali_config = {
      .unit_id = unit,
      .atten = atten,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
  if (ret == ESP_OK)
    calibrated = true;
#endif
  *out_handle = handle;
  return calibrated;
}

void sensor_task(void *pvParameters) {
  int raw_val;
  int cur_voltage;

  // Buffers for moving average
  int buffers[NUM_SENSORS][FILTER_SIZE];
  long sums[NUM_SENSORS] = {0};
  int buf_idx = 0;

  sensor_packet_t packet;
  int safety_counter = 0;

  // Zero out buffers
  memset(buffers, 0, sizeof(buffers));

  // Pre-fill buffers to avoid startup glitches
  for (int s = 0; s < NUM_SENSORS; s++) {
    adc_oneshot_read(adc_handle, SENSOR_CHANNELS[s], &raw_val);
    if (cali_handle)
      adc_cali_raw_to_voltage(cali_handle, raw_val, &cur_voltage);
    else
      cur_voltage = raw_val * 3300 / 4095;

    for (int i = 0; i < FILTER_SIZE; i++) {
      buffers[s][i] = cur_voltage;
      sums[s] += cur_voltage;
    }
  }

  while (1) {
    // Read all sensors in one pass
    for (int s = 0; s < NUM_SENSORS; s++) {
      ESP_ERROR_CHECK(
          adc_oneshot_read(adc_handle, SENSOR_CHANNELS[s], &raw_val));

      if (cali_handle)
        adc_cali_raw_to_voltage(cali_handle, raw_val, &cur_voltage);
      else
        cur_voltage = raw_val * 3300 / 4095;

      // Update Moving Average
      sums[s] -= buffers[s][buf_idx];
      buffers[s][buf_idx] = cur_voltage;
      sums[s] += cur_voltage;

      // Calculate Result
      packet.voltage_mv[s] = sums[s] / FILTER_SIZE;

      // Safety Clamp
      if (packet.voltage_mv[s] > 3300)
        packet.voltage_mv[s] = 3300;
      if (packet.voltage_mv[s] < 0)
        packet.voltage_mv[s] = 0;

      // Map to Distance using specific LUT
      packet.distance_mm[s] = lut_mv_to_mm[s][packet.voltage_mv[s]];
    }

    // Increment circular buffer index once per full sweep
    buf_idx++;
    if (buf_idx >= FILTER_SIZE)
      buf_idx = 0;

    // Send consolidated packet
    xQueueOverwrite(sensor_queue, &packet);

    // Yield occasionally to prevent watchdog triggers
    safety_counter++;
    if (safety_counter >= 500) {
      vTaskDelay(1);
      safety_counter = 0;
    }
  }
}

// -----------------------------------------------------------------------------
// PART 2: SERVO LOGIC (Unchanged)
// -----------------------------------------------------------------------------
typedef struct {
  float current_degree;
  float start_degree;
  float target_degree;
  int current_tick;
  int total_ticks;
  mcpwm_cmpr_handle_t comparator;
} ServoState;

ServoState servos[NUM_SERVOS];

float deg_to_us(float deg) {
  deg = fmaxf(SERVO_MIN_DEG, fminf(SERVO_MAX_DEG, deg));
  return (deg - SERVO_MIN_DEG) * (SERVO_MAX_US - SERVO_MIN_US) /
             (SERVO_MAX_DEG - SERVO_MIN_DEG) +
         SERVO_MIN_US;
}

static void motion_timer_callback(void *arg) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (servos[i].current_tick < servos[i].total_ticks) {
      servos[i].current_tick++;
      float t = (float)servos[i].current_tick / (float)servos[i].total_ticks;
      float cubic = (3 * t * t) - (2 * t * t * t);
      float ease = (1.0f - HYBRID_FACTOR) * t + (HYBRID_FACTOR)*cubic;
      servos[i].current_degree =
          servos[i].start_degree +
          ((servos[i].target_degree - servos[i].start_degree) * ease);
    } else {
      servos[i].current_degree = servos[i].target_degree;
    }
    uint32_t us = (uint32_t)deg_to_us(servos[i].current_degree);
    mcpwm_comparator_set_compare_value(servos[i].comparator, us);
  }
}

void process_command(char *line) {
  int servo_idx, duration_ms;
  float target_deg;
  char *t_ptr = strstr(line, "T");
  char *d_ptr = strstr(line, "D");

  if (t_ptr && d_ptr) {
    if (sscanf(t_ptr, "T%d:%f", &servo_idx, &target_deg) == 2) {
      if (sscanf(d_ptr, "D:%d", &duration_ms) == 1) {
        if (servo_idx >= 0 && servo_idx < NUM_SERVOS) {
          if (duration_ms < 20)
            duration_ms = 20;
          servos[servo_idx].start_degree = servos[servo_idx].current_degree;
          servos[servo_idx].target_degree = target_deg;
          servos[servo_idx].total_ticks = duration_ms * CONTROL_FREQ_HZ / 1000;
          servos[servo_idx].current_tick = 0;
        }
      }
    }
  }
}

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
        } else if (line_pos < sizeof(line_buffer) - 1) {
          line_buffer[line_pos++] = c;
        }
      }
    }
  }
  free(data);
}

// -----------------------------------------------------------------------------
// PART 3: SETUP FUNCTIONS
// -----------------------------------------------------------------------------
void setup_sensor_hardware() {
#if CALIBRATION_MODE == 0
  generate_luts();
#endif

  adc_oneshot_unit_init_cfg_t init_config = {.unit_id = ADC_UNIT};
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

  adc_oneshot_chan_cfg_t config = {.bitwidth = ADC_BITWIDTH_DEFAULT,
                                   .atten = ADC_ATTEN};

  // Configure all sensor channels
  for (int i = 0; i < NUM_SENSORS; i++) {
    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(adc_handle, SENSOR_CHANNELS[i], &config));
  }

  adc_calibration_init(ADC_UNIT, ADC_ATTEN, &cali_handle);
  sensor_queue = xQueueCreate(1, sizeof(sensor_packet_t));
  xTaskCreatePinnedToCore(sensor_task, "FastSensor", 4096, NULL, 5, NULL, 1);
}

void setup_servo_hardware() {
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

    servos[i].current_degree = 90.0f;
    servos[i].start_degree = 90.0f;
    servos[i].target_degree = 90.0f;
    mcpwm_comparator_set_compare_value(servos[i].comparator,
                                       (uint32_t)deg_to_us(90.0f));
  }
  ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

void setup_comms() {
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
  xTaskCreate(rx_task, "uart_rx_task", 4096, NULL, 10, NULL);
}

// -----------------------------------------------------------------------------
// MAIN APPLICATION
// -----------------------------------------------------------------------------
void app_main(void) {
  setup_servo_hardware();
  setup_comms();
  setup_sensor_hardware();

  const esp_timer_create_args_t timer_args = {
      .callback = &motion_timer_callback, .name = "motion_loop"};
  esp_timer_handle_t motion_timer;
  ESP_ERROR_CHECK(esp_timer_create(&timer_args, &motion_timer));
  ESP_ERROR_CHECK(
      esp_timer_start_periodic(motion_timer, 1000000 / CONTROL_FREQ_HZ));

  ESP_LOGI(TAG, "System Running with 3 Independent Sensors.");

  sensor_packet_t reading;

  while (1) {
    if (xQueueReceive(sensor_queue, &reading, pdMS_TO_TICKS(5))) {
#if CALIBRATION_MODE
      printf("RAW: %d, %d, %d mV\n", reading.voltage_mv[0],
             reading.voltage_mv[1], reading.voltage_mv[2]);
#else
      // Print all 3 distances in one line for easy plotting/parsing
      printf("D0:%.1f D1:%.1f D2:%.1f\n", reading.distance_mm[0],
             reading.distance_mm[1], reading.distance_mm[2]);

      // --- EXAMPLE: REACT TO SENSORS ---
      // if (reading.distance_mm[0] < 150.0f && reading.distance_mm[1] < 150.0f)
      // {
      //     process_command("T1:180 D:500"); // Both close? Move Servo 1
      // }
#endif
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}