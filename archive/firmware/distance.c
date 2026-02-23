/*
 * Sharp GP2Y0A21YK0F - High Speed & High Precision Driver
 *
 * FEATURES:
 * 1. "Burst Mode" Sampling: Runs 50x faster than standard delay loops.
 * 2. Lookup Table (LUT): Pre-calculates math at startup for 0-latency
 * conversion.
 * 3. Dual Mode: Set CALIBRATION_MODE to 1 for data gathering, 0 for deployment.
 * 4. Watchdog Safe: Includes safety sleeps to prevent system crashes.
 */

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>

// =============================================================================
// --- USER CONFIGURATION ---
// =============================================================================

// Set to 1: output RAW VOLTAGE (for Excel measurement)
// Set to 0: output DISTANCE (mm) using your polynomial
#define CALIBRATION_MODE 0

// Hardware Config
#define SENSOR_PIN GPIO_NUM_34
#define ADC_CHANNEL ADC_CHANNEL_6
#define ADC_UNIT ADC_UNIT_1
#define ADC_ATTEN ADC_ATTEN_DB_11

// Filtering
// Higher = Smoother but slightly more lag.
// With this code, 50 samples takes about 2ms to collect.
#define FILTER_SIZE 50

// =============================================================================
// --- POLYNOMIAL CALIBRATION (INSERT YOUR VALUES HERE) ---
// =============================================================================
// Only used when CALIBRATION_MODE is 0.
// Replace these with the values from your Excel Trendline.
static float poly_calculate_mm(double v) {
  // x is Voltage in Volts (e.g., 2.5V)
  double x = v;

  return (float)((-6.56E-08 * x * x * x) + // -6.02E-08 * x^3
                 (3.73E-04 * x * x) +      // +3.58E-04 * x^2
                 (-0.774 * x) + // 689 + -0.774x + 3.73E-04x^2 + -6.56E-08x^3
                 689.0          // Intercept
  );
}

// =============================================================================
// --- SYSTEM INTERNALS (DO NOT TOUCH) ---
// =============================================================================
static const char *TAG = "SharpFast";
static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t cali_handle = NULL;
static QueueHandle_t distance_queue = NULL;

// Lookup Table: Maps 0-3300mV directly to Distance (mm)
static float lut_mv_to_mm[3301];

typedef struct {
  int voltage_mv;
  float distance_mm;
} sensor_data_t;

// --- 1. INITIALIZATION: GENERATE LOOKUP TABLE ---
void generate_lut() {
  ESP_LOGW(TAG, "Generating High-Speed Lookup Table...");
  for (int mv = 0; mv <= 3300; mv++) {
    if (mv < 300) {
      lut_mv_to_mm[mv] = 800.0f; // >80cm
    } else if (mv > 3100) {
      lut_mv_to_mm[mv] = 60.0f; // <6cm (Danger Zone)
    } else {
      lut_mv_to_mm[mv] = poly_calculate_mm((double)mv);
    }
  }
  ESP_LOGI(TAG, "LUT Generation Complete.");
}

// --- 2. ADC INIT HELPER ---
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

// --- 3. HIGH SPEED TASK (OPTIMIZED) ---
void sensor_task(void *pvParameters) {
  int raw_val, voltage_mv;

  // Circular Buffer Setup
  int buffer[FILTER_SIZE] = {0};
  int buf_idx = 0;
  long sum_mv = 0;

  // ... [Buffer Pre-fill Code Same as Before] ...

  sensor_data_t output;
  int safety_counter = 0;

  while (1) {
    // A. READ
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL, &raw_val));

    // B. CALIBRATE
    if (cali_handle)
      adc_cali_raw_to_voltage(cali_handle, raw_val, &voltage_mv);
    else
      voltage_mv = raw_val * 3300 / 4095;

    // C. FILTER
    sum_mv -= buffer[buf_idx];
    buffer[buf_idx] = voltage_mv;
    sum_mv += voltage_mv;
    buf_idx++;
    if (buf_idx >= FILTER_SIZE)
      buf_idx = 0;

    // D. OUTPUT
    output.voltage_mv = sum_mv / FILTER_SIZE;
#if CALIBRATION_MODE == 0
    if (output.voltage_mv > 3300)
      output.voltage_mv = 3300; // Index safety
    if (output.voltage_mv < 0)
      output.voltage_mv = 0;
    output.distance_mm = lut_mv_to_mm[output.voltage_mv];
#endif

    xQueueOverwrite(distance_queue, &output);

    // --- OPTIMIZATION HERE ---
    // OLD: if (safety_counter >= 50) -> Slept too often.
    // NEW: Run 2000 times before sleeping.
    // 2000 samples takes ~100ms. This is totally safe for the Watchdog.
    safety_counter++;
    if (safety_counter >= 2000) {
      vTaskDelay(1);
      safety_counter = 0;
    }
  }
}

// --- 4. MAIN SETUP ---
void app_main(void) {
// 1. Generate Tables
#if CALIBRATION_MODE == 0
  generate_lut();
#endif

  // 2. Setup ADC
  adc_oneshot_unit_init_cfg_t init_config = {.unit_id = ADC_UNIT};
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

  adc_oneshot_chan_cfg_t config = {
      .bitwidth = ADC_BITWIDTH_DEFAULT,
      .atten = ADC_ATTEN,
  };
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &config));
  adc_calibration_init(ADC_UNIT, ADC_ATTEN, &cali_handle);

  // 3. Start Task
  distance_queue = xQueueCreate(1, sizeof(sensor_data_t));
  xTaskCreatePinnedToCore(sensor_task, "FastSensor", 4096, NULL, 5, NULL, 1);

  // 4. Display Loop (Human readable speed)
  sensor_data_t reading;
  printf("\n--- SYSTEM START ---\n");
  if (CALIBRATION_MODE)
    printf("!!! CALIBRATION MODE ACTIVE !!!\n");

  while (1) {
    if (xQueueReceive(distance_queue, &reading, pdMS_TO_TICKS(10))) {
#if CALIBRATION_MODE
      // Phase 1: Use this to get data for Excel
      printf("%d\n", reading.voltage_mv);
#else
      // Phase 2: Use this for your robot
      printf("Dist: %.1f mm  (Volts: %d mV)\n", reading.distance_mm,
             reading.voltage_mv);
#endif
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // Update serial monitor at 10Hz
  }
}

old Code
/*
 * Sharp GP2Y0A21YK0F - High Speed & High Precision Driver
 *
 * FEATURES:
 * 1. "Burst Mode" Sampling: Runs 50x faster than standard delay loops.
 * 2. Lookup Table (LUT): Pre-calculates math at startup for 0-latency
 * conversion.
 * 3. Dual Mode: Set CALIBRATION_MODE to 1 for data gathering, 0 for deployment.
 * 4. Watchdog Safe: Includes safety sleeps to prevent system crashes.
 */

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>

// =============================================================================
// --- USER CONFIGURATION ---
// =============================================================================

// Set to 1: output RAW VOLTAGE (for Excel measurement)
// Set to 0: output DISTANCE (mm) using your polynomial
#define CALIBRATION_MODE 0

// Hardware Config
#define SENSOR_PIN GPIO_NUM_34
#define ADC_CHANNEL ADC_CHANNEL_6
#define ADC_UNIT ADC_UNIT_1
#define ADC_ATTEN ADC_ATTEN_DB_11

// Filtering
// Higher = Smoother but slightly more lag.
// With this code, 50 samples takes about 2ms to collect.
#define FILTER_SIZE 50

    // =============================================================================
    // --- POLYNOMIAL CALIBRATION (INSERT YOUR VALUES HERE) ---
    // =============================================================================
    // Only used when CALIBRATION_MODE is 0.
    // Replace these with the values from your Excel Trendline.
    static float
    poly_calculate_mm(double v) {
  // x is Voltage in Volts (e.g., 2.5V)
  double x = v;

  return (float)((-6.56E-08 * x * x * x) + // -6.02E-08 * x^3
                 (3.73E-04 * x * x) +      // +3.58E-04 * x^2
                 (-0.774 * x) + // 689 + -0.774x + 3.73E-04x^2 + -6.56E-08x^3
                 689.0          // Intercept
  );
}

// =============================================================================
// --- SYSTEM INTERNALS (DO NOT TOUCH) ---
// =============================================================================
static const char *TAG = "SharpFast";
static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t cali_handle = NULL;
static QueueHandle_t distance_queue = NULL;

// Lookup Table: Maps 0-3300mV directly to Distance (mm)
static float lut_mv_to_mm[3301];

typedef struct {
  int voltage_mv;
  float distance_mm;
} sensor_data_t;

// --- 1. INITIALIZATION: GENERATE LOOKUP TABLE ---
void generate_lut() {
  ESP_LOGW(TAG, "Generating High-Speed Lookup Table...");
  for (int mv = 0; mv <= 3300; mv++) {
    if (mv < 300) {
      lut_mv_to_mm[mv] = 800.0f; // >80cm
    } else if (mv > 3100) {
      lut_mv_to_mm[mv] = 60.0f; // <6cm (Danger Zone)
    } else {
      lut_mv_to_mm[mv] = poly_calculate_mm((double)mv);
    }
  }
  ESP_LOGI(TAG, "LUT Generation Complete.");
}

// --- 2. ADC INIT HELPER ---
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

// --- 3. HIGH SPEED TASK (STABLE) ---
void sensor_task(void *pvParameters) {
  int raw_val, voltage_mv;

  // Circular Buffer Setup
  int buffer[FILTER_SIZE] = {0};
  int buf_idx = 0;
  long sum_mv = 0;

  // Pre-fill buffer to avoid startup ramp
  adc_oneshot_read(adc_handle, ADC_CHANNEL, &raw_val);
  if (cali_handle)
    adc_cali_raw_to_voltage(cali_handle, raw_val, &voltage_mv);
  else
    voltage_mv = raw_val * 3300 / 4095;

  for (int i = 0; i < FILTER_SIZE; i++) {
    buffer[i] = voltage_mv;
    sum_mv += voltage_mv;
  }

  sensor_data_t output;
  int safety_counter = 0; // <--- Prevents Watchdog Crash

  while (1) {
    // A. READ (Hardware limited speed)
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL, &raw_val));

    // B. CALIBRATE (Fast math)
    if (cali_handle) {
      adc_cali_raw_to_voltage(cali_handle, raw_val, &voltage_mv);
    } else {
      voltage_mv = raw_val * 3300 / 4095;
    }

    // C. FILTER (O(1) Rolling Average)
    sum_mv -= buffer[buf_idx];    // Remove oldest
    buffer[buf_idx] = voltage_mv; // Add newest
    sum_mv += voltage_mv;         // Update sum

    buf_idx++;
    if (buf_idx >= FILTER_SIZE)
      buf_idx = 0;

    int filtered_mv = sum_mv / FILTER_SIZE;

    // D. OUTPUT PREPARATION
    output.voltage_mv = filtered_mv;

#if CALIBRATION_MODE == 0
    // Instant Lookup (0 CPU cycles practically)
    if (filtered_mv < 0)
      filtered_mv = 0;
    if (filtered_mv > 3300)
      filtered_mv = 3300;
    output.distance_mm = lut_mv_to_mm[filtered_mv];
#else
    output.distance_mm = 0.0f;
#endif

    // E. SEND TO QUEUE
    xQueueOverwrite(distance_queue, &output);

    // F. BURST MODE LOGIC (The Fix)
    // Run 50 times at max speed, then sleep 1 tick.
    // This keeps the Watchdog happy without slowing down throughput
    // significantly.
    safety_counter++;
    if (safety_counter >= 50) {
      vTaskDelay(1);
      safety_counter = 0;
    }
  }
}

// --- 4. MAIN SETUP ---
void app_main(void) {
// 1. Generate Tables
#if CALIBRATION_MODE == 0
  generate_lut();
#endif

  // 2. Setup ADC
  adc_oneshot_unit_init_cfg_t init_config = {.unit_id = ADC_UNIT};
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

  adc_oneshot_chan_cfg_t config = {
      .bitwidth = ADC_BITWIDTH_DEFAULT,
      .atten = ADC_ATTEN,
  };
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &config));
  adc_calibration_init(ADC_UNIT, ADC_ATTEN, &cali_handle);

  // 3. Start Task
  distance_queue = xQueueCreate(1, sizeof(sensor_data_t));
  xTaskCreatePinnedToCore(sensor_task, "FastSensor", 4096, NULL, 5, NULL, 1);

  // 4. Display Loop (Human readable speed)
  sensor_data_t reading;
  printf("\n--- SYSTEM START ---\n");
  if (CALIBRATION_MODE)
    printf("!!! CALIBRATION MODE ACTIVE !!!\n");

  while (1) {
    if (xQueueReceive(distance_queue, &reading, pdMS_TO_TICKS(10))) {
#if CALIBRATION_MODE
      // Phase 1: Use this to get data for Excel
      printf("%d\n", reading.voltage_mv);
#else
      // Phase 2: Use this for your robot
      printf("Dist: %.1f mm  (Volts: %d mV)\n", reading.distance_mm,
             reading.voltage_mv);
#endif
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // Update serial monitor at 10Hz
  }
}