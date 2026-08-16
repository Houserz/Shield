/**
 * @file main_dual_core.c
 * @brief Project SHIELD - Dual-Core Data Acquisition System Main Program
 *
 * Core 0 (PRO_CPU): Data acquisition tasks
 * Core 1 (APP_CPU): SD card storage task
 *
 * MCU: ESP32-S3 (Dual-Core Xtensa LX7 @ 240MHz)
 * Sensor List:
 *   - Adafruit BNO085 (9-DOF IMU)
 *   - SW-420 (Vibration Sensor)
 *   - ACS723 (Current Sensor)
 *   - MPL3115A2 (Barometer)
 *   - Adafruit MCP9808 (Temperature Sensor)
 *   - INMP441 (MEMS Microphone, I2S, high sample rate)
 *   - 751-1015-ND (Photodiode, analog/ADC, medium sample rate)
 *
 * B4 AUTONOMOUS BIAS-INJECTION SEQUENCER
 * --------------------------------------
 * This branch (feature/b4-bias-injection) turns the "one session per boot,
 * stopped by button or timeout" firmware into a fully autonomous sequencer
 * that runs all 81 B4 sessions (3 trials x 9 axes x 3 severity levels) back
 * to back in a single power-on, no host machine attached. Flash once, power
 * on, press GPIO46 once when ready, walk away (~6.75 h), come back to 81
 * populated RUN_XXX folders.
 *
 * GPIO46 is dual-purpose: before the first session of this boot starts, a
 * press is the manual "go" signal (lets the operator confirm the board is
 * positioned/stationary before it starts consuming the 5-minute budget).
 * Every session after that runs automatically with no further presses. Once
 * a session is actually running, a GPIO46 press instead means clean abort
 * (does NOT advance the checkpoint; the aborted step re-runs on the next
 * power cycle).
 *
 * Progress is checkpointed to NVS after each fully-completed step, so a power
 * loss resumes from the interrupted step — the post-resume boot still waits
 * for a start press before continuing, same as a fresh boot.
 */
#include "BNO08x.hpp"

#include <inttypes.h>

extern "C" {
#include "data_types.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "gaussian.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sd_storage.h"
#include "sensor_hal.h"

// BNO085 driver declarations
bool accel_init(SensorContext_t* ctx);
bool accel_read_sample(SensorContext_t* ctx, float* data_out);
bool gyro_init(SensorContext_t* ctx);
bool gyro_read_sample(SensorContext_t* ctx, float* data_out);
bool mag_init(SensorContext_t* ctx);
bool mag_read_sample(SensorContext_t* ctx, float* data_out);
}

#define STATUS_LED_PIN GPIO_NUM_4
#define BUTTON_PIN GPIO_NUM_46

#define MS_PER_HR 3600000UL
#define MS_PER_MIN 60000UL

// Each B4 session normally runs for a fixed 5 minutes. The sequencer task
// (running in app_main) owns session boundaries now -- not the old
// button-or-90-min logic.
//
// B4_SMOKE_TEST: set to 1 for a quick bench validation run before the real
// 6.75-hour sweep. It shrinks each session to B4_SMOKE_TEST_SESSION_MS so you
// can watch the full 81-step state machine cycle, exercise the GPIO46 abort
// path, and test power-cycle/NVS resume in minutes instead of hours. ALWAYS
// set this back to 0 before the real unattended run -- the boot-time log
// banner below makes it hard to miss if it's accidentally left on.
#define B4_SMOKE_TEST 0
#define B4_SMOKE_TEST_SESSION_MS (20UL * 1000UL)  // 20 s/session in smoke-test mode

#if B4_SMOKE_TEST
#define B4_SESSION_DURATION_MS B4_SMOKE_TEST_SESSION_MS
#else
#define B4_SESSION_DURATION_MS (5UL * MS_PER_MIN)  // real B4 protocol duration
#endif

// Time allowed for the SD writer to drain remaining queued samples and flush
// after a session flips to STOPPING, before we finalize + close the run.
#define B4_DRAIN_MS 1000UL

static const char* TAG = "SHIELD";

// ==================== Global Queue Handles ====================
static QueueHandle_t fast_queue = NULL;
static QueueHandle_t medium_queue = NULL;
static QueueHandle_t slow_queue = NULL;

// ==================== Global State ====================
// system_state gates the acquisition/writer tasks. The sequencer flips it to
// RUNNING to start a session and STOPPING to end one; the four worker tasks
// live for the whole boot and park (they are never deleted) between sessions.
static daq_state_t system_state = DAQ_STATE_IDLE;
static daq_statistics_t statistics = {0};

// Latched by the button-monitor task on a GPIO46 press. The sequencer polls
// this at the top of its per-step wait loop and performs a clean abort.
static volatile bool g_abort_requested = false;

// ==================== Hardware Configuration Instances ====================

// BNO085 - SPI Configuration
// CS: GPIO37, SCLK: GPIO38, MOSI: GPIO40, MISO: GPIO39, INT: GPIO5, RST: GPIO6
// NOTES:
//  - INT (GPIO5) must have a hardware pullup
//  - PS0 and PS1 pins on BNO085 must be tied to HIGH (3.3v) for SPI
static bno08x_config_t bno085_spi_cfg = []() {
  bno08x_config_t cfg;
  cfg.io_mosi = GPIO_NUM_40;  // DI on BNO085
  cfg.io_miso = GPIO_NUM_39;  // SDA on BNO085
  cfg.io_sclk = GPIO_NUM_38;  // SCL on BNO085
  cfg.io_cs = GPIO_NUM_37;
  cfg.io_int = GPIO_NUM_5;
  cfg.io_rst = GPIO_NUM_6;
  cfg.spi_peripheral = SPI3_HOST;
  return cfg;
}();

static BNO08x bno085_imu(bno085_spi_cfg);

// SW-420 - GPIO Configuration
// GPIO10: digital input, no conflict with ADC or other peripherals
static vibration_gpio_config_t vibration_gpio_cfg = {.gpio_pin = 16};

// MPL3115A2 - I2C Configuration
// I2C0 bus shared with MCP9808. Use 4.7kΩ external pull-ups on SDA/SCL.
static hal_i2c_config_t mpl3115_i2c_cfg = {
    .i2c_port = 0,       // I2C_NUM_0
    .sda_pin = 17,       // GPIO3
    .scl_pin = 18,       // GPIO4
    .device_addr = 0x60  // MPL3115A2 default I2C address
};

// MCP9808 - I2C Configuration
// Shares same I2C0 bus with MPL3115 (same SDA=3, SCL=4 pins)
static hal_i2c_config_t mcp9808_i2c_cfg = {
    .i2c_port = 0,       // I2C_NUM_0 (shared with MPL3115)
    .sda_pin = 17,       // GPIO3 (same as MPL3115)
    .scl_pin = 18,       // GPIO4 (same as MPL3115)
    .device_addr = 0x18  // MCP9808 default I2C address
};

// INMP441 - I2S Configuration (high sample rate)
// BCK/WS/SD use GPIO17/18/21 to avoid conflict with BNO085 INT=15 and RST=16
static inmp441_i2s_config_t inmp441_i2s_cfg = {
    .i2s_port = 0,           // I2S_NUM_0
    .bck_pin = 45,           // GPIO17 (bit clock)
    .ws_pin = 47,            // GPIO18 (word select / LRCLK)
    .data_in_pin = 48,       // GPIO21 (data input)
    .sample_rate_hz = 16000  // INMP441 typical; decimate to 1kHz logical rate
};

// ==================== B4 bias injection (mutable registers) ====================
// Was: nine `static const float B4_BIAS_*`, hand-edited + reflashed per session.
// Now: mutable, single-writer (the sequencer in app_main) / single-reader
// (vTaskFast). The injection block in vTaskFast is UNCHANGED — it still reads
// these names directly; only the qualifier changed from `const` to `volatile`.
// Units: m/s^2 for accelerometer, rad/s for gyroscope, uT for magnetometer.
static volatile float B4_BIAS_ACCEL_X = 0.0f;
static volatile float B4_BIAS_ACCEL_Y = 0.0f;
static volatile float B4_BIAS_ACCEL_Z = 0.0f;
static volatile float B4_BIAS_GYRO_X = 0.0f;
static volatile float B4_BIAS_GYRO_Y = 0.0f;
static volatile float B4_BIAS_GYRO_Z = 0.0f;
static volatile float B4_BIAS_MAG_X = 0.0f;
static volatile float B4_BIAS_MAG_Y = 0.0f;
static volatile float B4_BIAS_MAG_Z = 0.0f;

// Axis enum — fixed order matches the human-readable reference sheet
// tools/b4_session_plan.csv (accel_x..mag_z). Also indexes g_bias_ptr[].
enum B4Axis {
  B4_ACCEL_X = 0,
  B4_ACCEL_Y,
  B4_ACCEL_Z,
  B4_GYRO_X,
  B4_GYRO_Y,
  B4_GYRO_Z,
  B4_MAG_X,
  B4_MAG_Y,
  B4_MAG_Z,
  B4_AXIS_COUNT  // == 9
};

enum B4Level { B4_LOW = 0, B4_MEDIUM, B4_HIGH, B4_LEVEL_COUNT /* == 3 */ };

// Pointer table so the sequencer can write a bias by axis index while the
// injection code keeps reading the individual named registers unchanged.
static volatile float* const g_bias_ptr[B4_AXIS_COUNT] = {
    &B4_BIAS_ACCEL_X, &B4_BIAS_ACCEL_Y, &B4_BIAS_ACCEL_Z,
    &B4_BIAS_GYRO_X,  &B4_BIAS_GYRO_Y,  &B4_BIAS_GYRO_Z,
    &B4_BIAS_MAG_X,   &B4_BIAS_MAG_Y,   &B4_BIAS_MAG_Z};

// Severity by sensor family (axis / 3) and level. Matches the B4 protocol table:
//   Accelerometer  low 0.05  med 0.20  high 0.50   (m/s^2)
//   Gyroscope      low 0.005 med 0.020 high 0.050  (rad/s)
//   Magnetometer   low 1.0   med 4.0   high 10.0   (uT)
static const float B4_SEVERITY[3][B4_LEVEL_COUNT] = {
    {0.05f, 0.20f, 0.50f},    // accel family (axes 0..2)
    {0.005f, 0.020f, 0.050f}, // gyro family  (axes 3..5)
    {1.0f, 4.0f, 10.0f},      // mag family   (axes 6..8)
};

static const char* const B4_AXIS_NAME[B4_AXIS_COUNT] = {
    "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y",
    "gyro_z",  "mag_x",   "mag_y",   "mag_z"};
static const char* const B4_UNIT[3] = {"m/s^2", "rad/s", "uT"};  // by family
static const char* const B4_LEVEL_NAME[B4_LEVEL_COUNT] = {"low", "medium",
                                                          "high"};

// One entry per B4 session, generated (not hand-transcribed) at boot.
typedef struct {
  uint8_t trial;  // 1..3
  uint8_t axis;   // B4Axis
  uint8_t level;  // B4Level
  float value;    // resolved from B4_SEVERITY
} b4_step_t;

#define B4_TOTAL_STEPS (3 * B4_AXIS_COUNT * B4_LEVEL_COUNT)  // 81
static b4_step_t g_b4_plan[B4_TOTAL_STEPS];

// ==================== NVS checkpoint (resume-safe) ====================
#define B4_NVS_NAMESPACE "b4"
#define B4_NVS_KEY_STEP "step_idx"    // uint32: index of NEXT step to run (0..81)
#define B4_NVS_KEY_DONE "sweep_done"  // uint8: 1 once all 81 complete
#define B4_COMPLETE_FLAG_PATH (SD_MOUNT_POINT "/b4_complete.flag")

// ==================== B4 helper functions ====================

// Build the 81-step plan: outer trial (1..3), then axis (accel_x..mag_z),
// then level (low, medium, high). 81 = 3 x 9 x 3.
static void b4_generate_plan(void) {
  int idx = 0;
  for (int trial = 1; trial <= 3; trial++) {
    for (int axis = 0; axis < B4_AXIS_COUNT; axis++) {
      for (int level = 0; level < B4_LEVEL_COUNT; level++) {
        g_b4_plan[idx].trial = (uint8_t)trial;
        g_b4_plan[idx].axis = (uint8_t)axis;
        g_b4_plan[idx].level = (uint8_t)level;
        g_b4_plan[idx].value = B4_SEVERITY[axis / 3][level];
        idx++;
      }
    }
  }
}

// Zero every bias register (nominal). Used on abort and on sweep completion.
static void b4_zero_bias(void) {
  for (int a = 0; a < B4_AXIS_COUNT; a++) {
    *g_bias_ptr[a] = 0.0f;
  }
}

// Apply a step's bias: zero all axes, then set the one axis under test. This
// call IS the transition from the previous step directly into the next one —
// there is deliberately no dwell-at-nominal rest period between sessions.
static void b4_set_bias_step(const b4_step_t* s) {
  b4_zero_bias();
  *g_bias_ptr[s->axis] = s->value;
}

// Sanity-print the generated plan so axis/level order and values are auditable
// against the protocol table in the serial log.
static void b4_log_plan(void) {
  ESP_LOGI(TAG, "B4 plan: %d steps (trial x axis x level)", B4_TOTAL_STEPS);
  for (int i = 0; i < B4_TOTAL_STEPS; i++) {
    const b4_step_t* s = &g_b4_plan[i];
    ESP_LOGI(TAG, "  step %2d: trial=%u axis=%-8s level=%-6s value=%.3f %s", i,
             (unsigned)s->trial, B4_AXIS_NAME[s->axis], B4_LEVEL_NAME[s->level],
             s->value, B4_UNIT[s->axis / 3]);
  }
}

// Write the auto-logged B4 metadata sidecar (RUN_XXX/b4_meta.json). Called once
// with status "running" at session start and again with the terminal status
// ("completed" / "aborted") at session close. firmware_commit is embedded at
// build time via IDF's app descriptor (defaults to `git describe`).
static void b4_write_meta(const char* run_path, const b4_step_t* s,
                          int step_index, const char* status) {
  char path[MAX_FILE_PATH_LEN];
  snprintf(path, sizeof(path), "%s/b4_meta.json", run_path);

  FILE* f = fopen(path, "w");
  if (!f) {
    ESP_LOGE(TAG, "b4_meta open failed: path=%s", path);
    return;
  }

  const esp_app_desc_t* app = esp_app_get_description();
  const int family = s->axis / 3;

  fprintf(f, "{\n");
  fprintf(f, "  \"fault_type\": \"bias_injection\",\n");
  fprintf(f, "  \"fault_severity\": \"%s\",\n", B4_LEVEL_NAME[s->level]);
  fprintf(f, "  \"axis\": \"%s\",\n", B4_AXIS_NAME[s->axis]);
  fprintf(f, "  \"value\": %.6f,\n", s->value);
  fprintf(f, "  \"unit\": \"%s\",\n", B4_UNIT[family]);
  fprintf(f, "  \"trial\": %u,\n", (unsigned)s->trial);
  fprintf(f, "  \"step_index\": %d,\n", step_index);
  fprintf(f, "  \"status\": \"%s\",\n", status);
  fprintf(f, "  \"firmware_commit\": \"%s\",\n", app ? app->version : "unknown");
  fprintf(f, "  \"idf_version\": \"%s\"\n", app ? app->idf_ver : "unknown");
  fprintf(f, "}\n");

  fclose(f);
}

// Read the checkpoint. Missing keys default to (step_idx=0, sweep_done=0).
static void b4_nvs_read_checkpoint(uint32_t* step_idx, uint8_t* sweep_done) {
  *step_idx = 0;
  *sweep_done = 0;

  nvs_handle_t h;
  esp_err_t e = nvs_open(B4_NVS_NAMESPACE, NVS_READWRITE, &h);
  if (e != ESP_OK) {
    ESP_LOGW(TAG, "nvs_open('%s') failed: %s — starting from step 0",
             B4_NVS_NAMESPACE, esp_err_to_name(e));
    return;
  }

  uint32_t si = 0;
  uint8_t sd = 0;
  if (nvs_get_u32(h, B4_NVS_KEY_STEP, &si) == ESP_OK) {
    *step_idx = si;
  }
  if (nvs_get_u8(h, B4_NVS_KEY_DONE, &sd) == ESP_OK) {
    *sweep_done = sd;
  }
  nvs_close(h);
}

// Persist the index of the NEXT step to run. Called ONLY after a step fully
// completes (session closed + metadata finalized).
static bool b4_nvs_write_step(uint32_t next_step_idx) {
  nvs_handle_t h;
  esp_err_t e = nvs_open(B4_NVS_NAMESPACE, NVS_READWRITE, &h);
  if (e != ESP_OK) {
    ESP_LOGE(TAG, "nvs_open for step write failed: %s", esp_err_to_name(e));
    return false;
  }
  e = nvs_set_u32(h, B4_NVS_KEY_STEP, next_step_idx);
  if (e == ESP_OK) {
    e = nvs_commit(h);
  }
  nvs_close(h);
  if (e != ESP_OK) {
    ESP_LOGE(TAG, "nvs write step_idx=%u failed: %s", (unsigned)next_step_idx,
             esp_err_to_name(e));
    return false;
  }
  return true;
}

// Mark the whole sweep complete.
static bool b4_nvs_set_done(void) {
  nvs_handle_t h;
  esp_err_t e = nvs_open(B4_NVS_NAMESPACE, NVS_READWRITE, &h);
  if (e != ESP_OK) {
    ESP_LOGE(TAG, "nvs_open for done write failed: %s", esp_err_to_name(e));
    return false;
  }
  e = nvs_set_u8(h, B4_NVS_KEY_DONE, 1);
  if (e == ESP_OK) {
    e = nvs_commit(h);
  }
  nvs_close(h);
  return e == ESP_OK;
}

// Idle forever with a status-LED pattern that is visually distinguishable at a
// glance: slow blink (0.5 s) = sweep DONE, fast blink (0.1 s) = ABORTED. Both
// differ from the solid-ON "session running" state. This never returns.
static void b4_led_idle_forever(bool aborted) {
  const TickType_t period = aborted ? pdMS_TO_TICKS(100) : pdMS_TO_TICKS(500);
  for (;;) {
    gpio_set_level(STATUS_LED_PIN, 1);
    vTaskDelay(period);
    gpio_set_level(STATUS_LED_PIN, 0);
    vTaskDelay(period);
  }
}

// ==================== Sensor Array Definition ====================
// Set .enabled = false to disable a sensor (skip init and acquisition)

#define NUM_SENSORS 9

static SensorContext_t my_sensors[NUM_SENSORS] = {
    {.id = 1,
     .type = SENSOR_TYPE_VIBRATION,
     .sampling_rate_hz = 1000,
     .enabled = true,  // Disabled for testing
     .hw_config = &vibration_gpio_cfg,
     .init = vibration_init,
     .read_sample = vibration_read_sample},
    // [2] ACS723 Current Sensor - Medium Tier
    {.id = 2,
     .type = SENSOR_TYPE_CURRENT,
     .sampling_rate_hz = 200,
     .enabled = true,
     .hw_config = NULL,
     .init = current_init,
     .read_sample = current_read_sample},
    // [3] MPL3115A2 Pressure Sensor - Slow Tier
    {.id = 3,
     .type = SENSOR_TYPE_PRESSURE,
     .sampling_rate_hz = 50,
     .enabled = true,
     .hw_config = &mpl3115_i2c_cfg,
     .init = mpl3115_init,
     .read_sample = mpl3115_read_sample},
    // [4] MCP9808 Temperature Sensor - Slow Tier
    {.id = 4,
     .type = SENSOR_TYPE_TEMP,
     .sampling_rate_hz = 50,
     .enabled = true,
     .hw_config = &mcp9808_i2c_cfg,
     .init = mcp9808_init,
     .read_sample = mcp9808_read_sample},
    // [5] INMP441 Microphone - Fast Tier (high sample rate)
    {.id = 5,
     .type = SENSOR_TYPE_MICROPHONE,
     .sampling_rate_hz = 1000,
     .enabled = true,  // Disabled for testing
     .hw_config = &inmp441_i2s_cfg,
     .init = inmp441_init,
     .read_sample = inmp441_read_sample},
    // [6] 751-1015-ND Photodiode - Medium Tier (medium sample rate)
    {.id = 6,
     .type = SENSOR_TYPE_PHOTODIODE,
     .sampling_rate_hz = 200,
     .enabled = true,
     .hw_config = NULL,
     .init = photodiode_init,
     .read_sample = photodiode_read_sample},
    // [7] BNO085 Magnetometer - Fast Tier
    {.id = 7,
     .type = SENSOR_TYPE_MAGNETOMETER,
     .sampling_rate_hz = 1000,
     .enabled = true,
     .hw_config = &bno085_imu,
     .init = mag_init,
     .read_sample = mag_read_sample},
    // [8] BNO085 Gyroscope - Fast Tier
    {.id = 8,
     .type = SENSOR_TYPE_GYROSCOPE,
     .sampling_rate_hz = 1000,
     .enabled = true,
     .hw_config = &bno085_imu,
     .init = gyro_init,
     .read_sample = gyro_read_sample},
    // [9] BNO085 Accelerometer - Fast Tier
    {.id = 9,
     .type = SENSOR_TYPE_ACCELEROMETER,
     .sampling_rate_hz = 1000,
     .enabled = true,
     .hw_config = &bno085_imu,
     .init = accel_init,
     .read_sample = accel_read_sample}};

// ==================== Core 0 Acquisition Tasks ====================
//
// All three acquisition tasks and the SD writer are created ONCE at boot and
// live for the entire power-on. Between sessions they PARK in the outer loop
// (waiting for system_state == RUNNING) rather than self-deleting, so the same
// create->acquire->close cycle can repeat 81 times without task churn.

/**
 * @brief Fast task (1kHz, Core 0)
 * For high-speed sensors (BNO085 raw sensors, SW-420 Vibration, INMP441
 * Microphone)
 */
void vTaskFast(void* pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(1) > 0 ? pdMS_TO_TICKS(1) : 1;

  for (;;) {
    // Park until the sequencer starts a session.
    while (system_state != DAQ_STATE_RUNNING) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (system_state == DAQ_STATE_RUNNING) {
      for (int i = 0; i < NUM_SENSORS; i++) {
        if (!my_sensors[i].enabled || my_sensors[i].sampling_rate_hz != 1000)
          continue;

        float data[3] = {0};
        if (my_sensors[i].read_sample(&my_sensors[i], data)) {
          // B4 bias injection (reads the mutable B4_BIAS_* registers)
          if (my_sensors[i].type == SENSOR_TYPE_ACCELEROMETER) {
            data[0] += B4_BIAS_ACCEL_X;
            data[1] += B4_BIAS_ACCEL_Y;
            data[2] += B4_BIAS_ACCEL_Z;
          } else if (my_sensors[i].type == SENSOR_TYPE_GYROSCOPE) {
            data[0] += B4_BIAS_GYRO_X;
            data[1] += B4_BIAS_GYRO_Y;
            data[2] += B4_BIAS_GYRO_Z;
          } else if (my_sensors[i].type == SENSOR_TYPE_MAGNETOMETER) {
            data[0] += B4_BIAS_MAG_X;
            data[1] += B4_BIAS_MAG_Y;
            data[2] += B4_BIAS_MAG_Z;
          }

          fast_queue_msg_t msg = {
              .type = QUEUE_MSG_DATA,
              .data = {.timestamp_ms = get_timestamp_ms(),
                       .sensor_id = (uint8_t)my_sensors[i].id,
                       .reserved = {0},
                       .data = {data[0], data[1], data[2]}}};

          if (xQueueSend(fast_queue, &msg, 0) != pdTRUE) {
            statistics.queue_overruns++;
          } else {
            statistics.fast_samples++;
          }
        }
      }

      vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
  }
}

/**
 * @brief Medium task (200Hz, Core 0)
 * For medium-speed sensors (ACS723 Current, 751-1015-ND Photodiode)
 */
void vTaskMedium(void* pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(5) > 0 ? pdMS_TO_TICKS(5) : 1;

  for (;;) {
    while (system_state != DAQ_STATE_RUNNING) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (system_state == DAQ_STATE_RUNNING) {
      for (int i = 0; i < NUM_SENSORS; i++) {
        if (my_sensors[i].enabled && my_sensors[i].sampling_rate_hz == 200) {
          float data = 0.0f;
          if (my_sensors[i].read_sample(&my_sensors[i], &data)) {
            medium_queue_msg_t msg = {
                .type = QUEUE_MSG_DATA,
                .data = {.timestamp_ms = get_timestamp_ms(),
                         .sensor_id = (uint8_t)my_sensors[i].id,
                         .reserved = {0},
                         .data = data}};

            if (xQueueSend(medium_queue, &msg, 0) != pdTRUE) {
              statistics.queue_overruns++;
            } else {
              statistics.medium_samples++;
            }
          }
        }
      }

      vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
  }
}

/**
 * @brief Slow task (50Hz, Core 0)
 * For low-speed sensors (MPL3115A2 Barometer, MCP9808 Temperature)
 */
void vTaskSlow(void* pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(20);

  for (;;) {
    while (system_state != DAQ_STATE_RUNNING) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (system_state == DAQ_STATE_RUNNING) {
      for (int i = 0; i < NUM_SENSORS; i++) {
        if (my_sensors[i].enabled && my_sensors[i].sampling_rate_hz == 50) {
          float data = 0.0f;
          if (my_sensors[i].read_sample(&my_sensors[i], &data)) {
            slow_queue_msg_t msg = {
                .type = QUEUE_MSG_DATA,
                .data = {.timestamp_ms = get_timestamp_ms(),
                         .sensor_id = (uint8_t)my_sensors[i].id,
                         .reserved = {0},
                         .data = data}};

            if (xQueueSend(slow_queue, &msg, 0) != pdTRUE) {
              statistics.queue_overruns++;
            } else {
              statistics.slow_samples++;
            }
          }
        }
      }

      vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
  }
}

// ==================== Core 1 Storage Task ====================

/**
 * @brief SD card write task (Core 1)
 * Reads data from three queues and writes to SD card. Lives for the whole boot;
 * on each session end (RUNNING -> STOPPING) it drains any remaining queued
 * samples and performs a final flush, then parks for the next session.
 */
void vTaskSDWriter(void* pvParameters) {
  fast_queue_msg_t fast_msg;
  medium_queue_msg_t medium_msg;
  slow_queue_msg_t slow_msg;

  for (;;) {
    // Park until the sequencer starts a session.
    while (system_state != DAQ_STATE_RUNNING) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    uint32_t last_stats_update = 0;

    while (system_state == DAQ_STATE_RUNNING) {
      bool has_data = false;

      // Process Fast queue
      while (xQueueReceive(fast_queue, &fast_msg, 0) == pdTRUE) {
        if (fast_msg.type == QUEUE_MSG_DATA) {
          if (!sd_write_fast_data(&fast_msg.data)) {
            statistics.sd_errors++;
          }
          has_data = true;
        }
      }

      // Process Medium queue
      while (xQueueReceive(medium_queue, &medium_msg, 0) == pdTRUE) {
        if (medium_msg.type == QUEUE_MSG_DATA) {
          if (!sd_write_medium_data(&medium_msg.data)) {
            statistics.sd_errors++;
          }
          has_data = true;
        }
      }

      // Process Slow queue
      while (xQueueReceive(slow_queue, &slow_msg, 0) == pdTRUE) {
        if (slow_msg.type == QUEUE_MSG_DATA) {
          if (!sd_write_slow_data(&slow_msg.data)) {
            statistics.sd_errors++;
          }
          has_data = true;
        }
      }

      // Periodically flush buffers and update statistics (every second)
      uint32_t current_time = get_timestamp_ms();
      if (current_time - last_stats_update > 1000) {
        sd_flush_all_buffers();
        statistics.duration_ms = current_time;

        // Update metadata
        const run_session_t* session = sd_get_current_session();
        if (session->is_active) {
          metadata_update_statistics(session->meta_file, &statistics);
        }

        last_stats_update = current_time;
      }

      // If no data, rest a bit
      if (!has_data) {
        vTaskDelay(pdMS_TO_TICKS(10));
      }
    }

    // Session ended: drain whatever is still queued, then final flush, so we
    // do not lose up to a full queue depth of samples at the boundary.
    while (xQueueReceive(fast_queue, &fast_msg, 0) == pdTRUE) {
      if (fast_msg.type == QUEUE_MSG_DATA && !sd_write_fast_data(&fast_msg.data))
        statistics.sd_errors++;
    }
    while (xQueueReceive(medium_queue, &medium_msg, 0) == pdTRUE) {
      if (medium_msg.type == QUEUE_MSG_DATA &&
          !sd_write_medium_data(&medium_msg.data))
        statistics.sd_errors++;
    }
    while (xQueueReceive(slow_queue, &slow_msg, 0) == pdTRUE) {
      if (slow_msg.type == QUEUE_MSG_DATA && !sd_write_slow_data(&slow_msg.data))
        statistics.sd_errors++;
    }
    sd_flush_all_buffers();
  }
}

// ==================== Button Monitor Task ====================

/**
 * @brief GPIO46 abort monitor (low priority)
 * Latches g_abort_requested on a press so the sequencer can perform a clean
 * abort even if the press lands outside its wait loop. GPIO46 is active-low
 * (idle high via external pull-up), matching the original firmware.
 */
void vTaskButtonMonitor(void* pvParameters) {
  for (;;) {
    if (gpio_get_level(BUTTON_PIN) == 0) {
      if (!g_abort_requested) {
        ESP_LOGW(TAG, "GPIO46 pressed — abort requested");
      }
      g_abort_requested = true;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ==================== Sequencer helpers ====================

// Cleanly stop the currently-running session: flip to STOPPING, let the SD
// writer drain + flush, record final stats, finalize meta.json, write the B4
// sidecar with the given terminal status, and close the run.
static void b4_finish_session(const run_session_t* session, const b4_step_t* s,
                              int step_index, uint32_t session_start_ms,
                              const char* status) {
  system_state = DAQ_STATE_STOPPING;

  // Give the acquisition tasks time to stop producing and the SD writer time
  // to drain the queues and flush.
  vTaskDelay(pdMS_TO_TICKS(B4_DRAIN_MS));

  sd_flush_all_buffers();
  statistics.duration_ms = get_timestamp_ms() - session_start_ms;

  metadata_update_statistics(session->meta_file, &statistics);
  metadata_finalize(session->meta_file);
  b4_write_meta(session->run_path, s, step_index, status);

  sd_close_run_session();
}

// Block until GPIO46 is pressed and released, blinking the status LED to show
// the board is armed and waiting. Called ONCE per boot, before the first
// session of the sequencer loop — every session after that runs unattended.
//
// vTaskButtonMonitor is already running by the time this is called and will
// latch g_abort_requested on the very press we're waiting for; clear it
// afterward so the first session doesn't see a stale abort and immediately
// terminate itself.
static void b4_wait_for_start_button(void) {
  ESP_LOGI(TAG, "Armed — press GPIO46 to start the first session...");

  bool led_on = false;
  TickType_t last_toggle = xTaskGetTickCount();
  const TickType_t blink_period = pdMS_TO_TICKS(250);

  while (gpio_get_level(BUTTON_PIN) != 0) {
    if (xTaskGetTickCount() - last_toggle >= blink_period) {
      led_on = !led_on;
      gpio_set_level(STATUS_LED_PIN, led_on ? 1 : 0);
      last_toggle = xTaskGetTickCount();
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  gpio_set_level(STATUS_LED_PIN, 0);

  // Debounce, then wait for release so we don't fall straight through on a
  // press that's still held down.
  vTaskDelay(pdMS_TO_TICKS(50));
  while (gpio_get_level(BUTTON_PIN) == 0) {
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  vTaskDelay(pdMS_TO_TICKS(50));

  g_abort_requested = false;
  ESP_LOGI(TAG, "Start confirmed — sequencer beginning, no further presses "
                "needed until the sweep ends.");
}

// ==================== Main Program ====================

/**
 * @brief Main program entry point
 *
 * Runs the autonomous B4 sequencer: 81 back-to-back 5-minute bias-injection
 * sessions in one power-on, resume-safe via NVS. GPIO46 starts the first
 * session of the boot; a press during any running session aborts the sweep.
 */
extern "C" void app_main(void) {
  ESP_LOGI(TAG, "========== Project SHIELD (B4 sequencer) starting ==========");

  // Initialize data types module
  data_types_init();
  ESP_LOGI(TAG, "Data types module initialized");

  // Initialize NVS (standard ESP-IDF boilerplate; used for the B4 checkpoint)
  esp_err_t nvs_err = nvs_flash_init();
  if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES ||
      nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGW(TAG, "NVS needs erase (%s) — erasing", esp_err_to_name(nvs_err));
    ESP_ERROR_CHECK(nvs_flash_erase());
    nvs_err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(nvs_err);
  ESP_LOGI(TAG, "NVS initialized OK");

  // SD card initialization
  ESP_LOGI(TAG, "Initializing SD card...");
  if (!sd_storage_init()) {
    ESP_LOGE(TAG, "SD card initialization FAILED - aborting");
    return;
  }
  ESP_LOGI(TAG, "SD card initialized OK");

  i2c_config_t i2c_cfg = {};
  i2c_cfg.mode = I2C_MODE_MASTER;
  i2c_cfg.sda_io_num = 17;
  i2c_cfg.scl_io_num = 18;
  i2c_cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
  i2c_cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
  i2c_cfg.master.clk_speed = 400000;
  i2c_cfg.clk_flags = 0;
  ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_cfg));
  ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
  i2c_reset_tx_fifo(I2C_NUM_0);
  i2c_reset_rx_fifo(I2C_NUM_0);
  ESP_LOGI(TAG, "I2C master initialized on port %d", I2C_NUM_0);
  vTaskDelay(pdMS_TO_TICKS(100));  // give devices time to settle

  if (!bno085_imu.initialize()) {
    ESP_LOGE(TAG, "BNO085 initialize() FAILED - aborting");
    return;
  }
  ESP_LOGI(TAG, "BNO085 initialized OK");

  // Initialize all sensors
  const char* sensor_names[] = {
      "SW-420 Vibration",    "ACS723 Current",     "MPL3115 Pressure",
      "MCP9808 Temp",        "INMP441 Microphone", "751-1015-ND Photodiode",
      "BNO085 Magnetometer", "BNO085 Gyroscope",   "BNO085 Accelerometer"};
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!my_sensors[i].enabled) {
      ESP_LOGI(TAG, "Sensor [%d] %s disabled - skipping", i, sensor_names[i]);
      continue;
    }
    ESP_LOGI(TAG, "Initializing sensor [%d] %s...", i, sensor_names[i]);
    bool ok = my_sensors[i].init(&my_sensors[i]);
    if (ok) {
      ESP_LOGI(TAG, "  Sensor [%d] %s initialized OK", i, sensor_names[i]);
    } else {
      ESP_LOGE(TAG, "  Sensor [%d] %s initialization FAILED", i,
               sensor_names[i]);
    }
  }

  // Create FreeRTOS queues (once — reused across all 81 sessions)
  fast_queue = xQueueCreate(FAST_QUEUE_SIZE, sizeof(fast_queue_msg_t));
  medium_queue = xQueueCreate(MEDIUM_QUEUE_SIZE, sizeof(medium_queue_msg_t));
  slow_queue = xQueueCreate(SLOW_QUEUE_SIZE, sizeof(slow_queue_msg_t));

  if (!fast_queue || !medium_queue || !slow_queue) {
    ESP_LOGE(TAG,
             "Queue creation FAILED (fast=%p medium=%p slow=%p) - aborting",
             fast_queue, medium_queue, slow_queue);
    return;
  }
  ESP_LOGI(TAG, "FreeRTOS queues created OK");

  // Configure status LED (GPIO 4) - solid ON while a session runs
  gpio_config_t led_cfg = {
      .pin_bit_mask = (1ULL << STATUS_LED_PIN),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&led_cfg);
  gpio_set_level(STATUS_LED_PIN, 0);

  // Configure button input (GPIO 46) - dual purpose: one press arms the
  // first session of this boot (see b4_wait_for_start_button), a press once
  // a session is running instead means ABORT the sweep
  gpio_config_t btn_conf = {
      .pin_bit_mask = (1ULL << BUTTON_PIN),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&btn_conf);

  // Build and log the 81-step plan
  b4_generate_plan();
  b4_log_plan();

#if B4_SMOKE_TEST
  ESP_LOGW(TAG,
           "******************************************************");
  ESP_LOGW(TAG,
           "*** B4_SMOKE_TEST=1 -- sessions shortened to %lu ms  ***",
           (unsigned long)B4_SMOKE_TEST_SESSION_MS);
  ESP_LOGW(TAG,
           "*** This is NOT the real B4 protocol. Set back to 0. ***");
  ESP_LOGW(TAG,
           "******************************************************");
#endif

  // Launch the worker tasks ONCE. system_state is still IDLE, so they park
  // immediately and wait for the sequencer to flip it to RUNNING per session.
  // Core 0 (PRO_CPU): Data acquisition tasks
  xTaskCreatePinnedToCore(vTaskFast, "FastTask", 4096, NULL, 10, NULL, 0);
  xTaskCreatePinnedToCore(vTaskMedium, "MediumTask", 4096, NULL, 8, NULL, 0);
  xTaskCreatePinnedToCore(vTaskSlow, "SlowTask", 4096, NULL, 6, NULL, 0);
  // Core 1 (APP_CPU): SD card write task
  xTaskCreatePinnedToCore(vTaskSDWriter, "SDWriter", 8192, NULL, 5, NULL, 1);
  // Low-priority abort monitor
  xTaskCreatePinnedToCore(vTaskButtonMonitor, "BtnMon", 2048, NULL, 2, NULL, 1);

  // ---- Resume from the NVS checkpoint ----
  uint32_t step_idx = 0;
  uint8_t sweep_done = 0;
  b4_nvs_read_checkpoint(&step_idx, &sweep_done);
  ESP_LOGI(TAG, "B4 checkpoint: step_idx=%u sweep_done=%u",
           (unsigned)step_idx, (unsigned)sweep_done);

  if (sweep_done) {
    ESP_LOGI(TAG, "B4 sweep already complete — nothing to do. Idling.");
    b4_zero_bias();
    b4_led_idle_forever(false);  // slow-blink "done" (never returns)
  }

  if (step_idx >= B4_TOTAL_STEPS) {
    // Defensive: checkpoint past the end but done-flag not set. Treat as done.
    ESP_LOGW(TAG, "step_idx %u >= %d but sweep_done unset — marking done",
             (unsigned)step_idx, B4_TOTAL_STEPS);
    b4_zero_bias();
    b4_nvs_set_done();
    b4_led_idle_forever(false);
  }

  // ---- Wait for manual start (gates only the first session this boot) ----
  b4_wait_for_start_button();

  // ---- Sequencer: run steps step_idx .. 80 ----
  for (uint32_t i = step_idx; i < B4_TOTAL_STEPS; i++) {
    const b4_step_t* step = &g_b4_plan[i];

    // Direct transition into this step's bias (no dwell-at-nominal rest).
    b4_set_bias_step(step);

    ESP_LOGI(TAG,
             "==== B4 step %u/%d: trial=%u axis=%s level=%s value=%.4f %s ====",
             (unsigned)i, B4_TOTAL_STEPS - 1, (unsigned)step->trial,
             B4_AXIS_NAME[step->axis], B4_LEVEL_NAME[step->level], step->value,
             B4_UNIT[step->axis / 3]);

    // Create the run session (retry once on failure).
    if (!sd_create_run_session()) {
      ESP_LOGE(TAG, "sd_create_run_session failed — retrying once");
      vTaskDelay(pdMS_TO_TICKS(500));
      if (!sd_create_run_session()) {
        ESP_LOGE(TAG, "sd_create_run_session failed again — SAFE HALT");
        b4_zero_bias();
        b4_led_idle_forever(true);  // fast-blink error (never returns)
      }
    }

    const run_session_t* session = sd_get_current_session();
    ESP_LOGI(TAG, "Run session: %s at %s", session->run_id, session->run_path);

    // Fresh statistics for this session's meta.json.
    statistics = daq_statistics_t{};

    // meta.json (existing schema) + b4_meta.json sidecar (B4 fields).
    if (!metadata_create(session->meta_file, session->run_id)) {
      ESP_LOGE(TAG, "metadata_create FAILED");
    }
    b4_write_meta(session->run_path, step, (int)i, "running");

    // Start acquisition and time the fixed-length session.
    system_state = DAQ_STATE_RUNNING;
    gpio_set_level(STATUS_LED_PIN, 1);
    uint32_t session_start_ms = get_timestamp_ms();

    bool aborted = false;
    while ((get_timestamp_ms() - session_start_ms) < B4_SESSION_DURATION_MS) {
      if (g_abort_requested) {
        aborted = true;
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(50));
    }

    if (aborted) {
      ESP_LOGW(TAG, "ABORT during step %u — closing run as aborted", (unsigned)i);
      b4_finish_session(session, step, (int)i, session_start_ms, "aborted");
      b4_zero_bias();
      gpio_set_level(STATUS_LED_PIN, 0);
      // Deliberately do NOT advance the NVS checkpoint: this step re-runs on
      // the next power cycle. Park; require a fresh power cycle to resume.
      ESP_LOGW(TAG, "Halted after abort. Power-cycle to resume from step %u.",
               (unsigned)i);
      b4_led_idle_forever(true);  // fast-blink "aborted" (never returns)
    }

    // Normal completion of this step.
    b4_finish_session(session, step, (int)i, session_start_ms, "completed");
    gpio_set_level(STATUS_LED_PIN, 0);

    ESP_LOGI(TAG,
             "Step %u done: fast=%" PRIu32 " medium=%" PRIu32 " slow=%" PRIu32
             " overruns=%" PRIu32 " sd_errors=%" PRIu32,
             (unsigned)i, statistics.fast_samples, statistics.medium_samples,
             statistics.slow_samples, statistics.queue_overruns,
             statistics.sd_errors);

    // Checkpoint AFTER this step fully completes (session closed + finalized).
    if (!b4_nvs_write_step(i + 1)) {
      ESP_LOGE(TAG, "checkpoint write failed after step %u", (unsigned)i);
    }
  }

  // ---- All 81 steps complete ----
  b4_zero_bias();
  b4_nvs_set_done();

  FILE* flag = fopen(B4_COMPLETE_FLAG_PATH, "w");
  if (flag) {
    const esp_app_desc_t* app = esp_app_get_description();
    fprintf(flag, "B4 bias-injection sweep complete: %d/%d steps.\n",
            B4_TOTAL_STEPS, B4_TOTAL_STEPS);
    fprintf(flag, "firmware_commit=%s\n", app ? app->version : "unknown");
    fclose(flag);
    ESP_LOGI(TAG, "Wrote completion marker: %s", B4_COMPLETE_FLAG_PATH);
  } else {
    ESP_LOGE(TAG, "Failed to write completion marker: %s",
             B4_COMPLETE_FLAG_PATH);
  }

  ESP_LOGI(TAG, "========== B4 sweep COMPLETE (%d sessions) ==========",
           B4_TOTAL_STEPS);
  b4_led_idle_forever(false);  // slow-blink "done" (never returns)
}
