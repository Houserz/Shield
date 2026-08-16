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
 * A1 SHAKER-TABLE SEQUENCER
 * --------------------------
 * This branch (feature/a1-shaker-sequencer) automates the remaining A1
 * vibration-stress sessions for this batch, run in order: low, medium,
 * medium, high. Each session is the existing 70-minute (5 min settle /
 * 60 min stress / 5 min recovery) recording, unchanged.
 *
 * The shaker itself is driven by a1_batch_runner.py on a laptop over serial,
 * which chains all 4 sessions back-to-back with ZERO gap between them (the
 * instant one session's recovery ends, the next one's settle begins). For
 * this firmware to stay time-aligned with that, GPIO46 gates only the FIRST
 * session of the boot — matching the B4 sweep's pattern — then the remaining
 * steps auto-continue unattended. Press GPIO46 at (as close as possible to)
 * the moment a1_batch_runner.py is launched; from then on both sides run the
 * same 70-minute cadence with no further handoff required. A press once a
 * session is running is a clean abort instead (does NOT advance the
 * checkpoint; that step re-runs on the next power cycle, gated by another
 * button press).
 *
 * Progress is checkpointed to NVS after each fully-completed step. The
 * checkpoint is tagged with a fingerprint of the run order
 * (low-medium-medium-high); if a stale checkpoint from a different plan is
 * ever found in NVS, it's discarded and the sequence restarts at step 0
 * rather than silently resuming at the wrong step — see the B4 sweep's
 * step_idx bug this avoids.
 */
#include "BNO08x.hpp"

#include <inttypes.h>
#include <string.h>

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

#define RUN_DURATION 70UL  // Shaker-table experiment: run for 1 hr 10 min (70 minutes)
#define MS_PER_MIN 60000UL
// Noise injection stays disabled for the shaker-table experiment — not wired
// into this sequencer at all (unlike the old single-run firmware's #define
// gate), since A1 is a clean-signal capture.

#define A1_SESSION_DURATION_MS (RUN_DURATION * MS_PER_MIN)

// Time allowed for the SD writer to drain remaining queued samples and flush
// after a session flips to STOPPING, before we finalize + close the run.
#define A1_DRAIN_MS 1000UL

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

// Latched by the button-monitor task on a GPIO46 press. Consumed either as
// the manual start signal (before a session) or as an abort request (during
// a running session) depending on where the sequencer currently is.
static volatile bool g_abort_requested = false;

// ==================== A1 shaker sequence (fixed order) ====================
enum A1Severity { A1_LOW = 0, A1_MEDIUM, A1_HIGH, A1_SEVERITY_COUNT };

static const char* const A1_SEVERITY_NAME[A1_SEVERITY_COUNT] = {"low", "medium",
                                                                 "high"};
static const int A1_RPM[A1_SEVERITY_COUNT] = {80, 140, 200};

// This batch: low, medium, medium, high — in that exact order.
static const uint8_t A1_PLAN[] = {A1_LOW, A1_MEDIUM, A1_MEDIUM, A1_HIGH};
#define A1_TOTAL_STEPS \
  ((uint32_t)(sizeof(A1_PLAN) / sizeof(A1_PLAN[0])))  // 4

// ==================== NVS checkpoint (resume-safe) ====================
#define A1_NVS_NAMESPACE "a1"
#define A1_NVS_KEY_STEP "step_idx"    // uint32: index of NEXT step to run (0..4)
#define A1_NVS_KEY_DONE "sweep_done"  // uint8: 1 once all steps complete
#define A1_NVS_KEY_PLAN "plan_tag"    // string: fingerprint of A1_PLAN
#define A1_COMPLETE_FLAG_PATH (SD_MOUNT_POINT "/a1_complete.flag")

// ==================== A1 helper functions ====================

// Build a fingerprint of the current A1_PLAN, e.g. "low-medium-medium-high".
// Stored alongside the checkpoint so a stale step_idx left over from a
// differently-ordered run can never be silently reused.
static void a1_build_plan_tag(char* out, size_t out_len) {
  size_t pos = 0;
  for (size_t i = 0; i < A1_TOTAL_STEPS && pos < out_len; i++) {
    int n = snprintf(out + pos, out_len - pos, "%s%s",
                      A1_SEVERITY_NAME[A1_PLAN[i]],
                      (i + 1 < A1_TOTAL_STEPS) ? "-" : "");
    if (n < 0) break;
    pos += (size_t)n;
  }
}

// Write the auto-logged A1 metadata sidecar (RUN_XXX/a1_meta.json). Called
// once with status "running" at session start and again with the terminal
// status ("completed" / "aborted") at session close. Replaces the old
// paper-log cross-referencing — the severity/RPM this RUN_XXX belongs to is
// now recorded by the firmware itself, not inferred after the fact.
static void a1_write_meta(const char* run_path, uint8_t severity,
                          int step_index, const char* status) {
  char path[MAX_FILE_PATH_LEN];
  snprintf(path, sizeof(path), "%s/a1_meta.json", run_path);

  FILE* f = fopen(path, "w");
  if (!f) {
    ESP_LOGE(TAG, "a1_meta open failed: path=%s", path);
    return;
  }

  const esp_app_desc_t* app = esp_app_get_description();

  fprintf(f, "{\n");
  fprintf(f, "  \"fault_type\": \"shaker_vibration\",\n");
  fprintf(f, "  \"severity\": \"%s\",\n", A1_SEVERITY_NAME[severity]);
  fprintf(f, "  \"rpm\": %d,\n", A1_RPM[severity]);
  fprintf(f, "  \"step_index\": %d,\n", step_index);
  fprintf(f, "  \"status\": \"%s\",\n", status);
  fprintf(f, "  \"firmware_commit\": \"%s\",\n", app ? app->version : "unknown");
  fprintf(f, "  \"idf_version\": \"%s\"\n", app ? app->idf_ver : "unknown");
  fprintf(f, "}\n");

  fclose(f);
}

// Read the checkpoint. If the stored plan tag doesn't match the current
// A1_PLAN (or no checkpoint exists yet), the checkpoint is reset to step 0 —
// this is the guard against the exact stale-NVS-checkpoint failure mode found
// on the B4 sweep, where a leftover step_idx from an earlier/different run
// silently skipped the first several steps.
static void a1_nvs_read_checkpoint(uint32_t* step_idx, uint8_t* sweep_done) {
  *step_idx = 0;
  *sweep_done = 0;

  nvs_handle_t h;
  esp_err_t e = nvs_open(A1_NVS_NAMESPACE, NVS_READWRITE, &h);
  if (e != ESP_OK) {
    ESP_LOGW(TAG, "nvs_open('%s') failed: %s — starting from step 0",
             A1_NVS_NAMESPACE, esp_err_to_name(e));
    return;
  }

  char expected_tag[64];
  a1_build_plan_tag(expected_tag, sizeof(expected_tag));

  char stored_tag[64] = {0};
  size_t tag_len = sizeof(stored_tag);
  esp_err_t tag_err = nvs_get_str(h, A1_NVS_KEY_PLAN, stored_tag, &tag_len);

  if (tag_err != ESP_OK || strcmp(stored_tag, expected_tag) != 0) {
    ESP_LOGW(TAG,
             "A1 plan tag mismatch (stored='%s' expected='%s', err=%s) — "
             "resetting checkpoint to step 0",
             tag_err == ESP_OK ? stored_tag : "<none>", expected_tag,
             esp_err_to_name(tag_err));
    nvs_set_str(h, A1_NVS_KEY_PLAN, expected_tag);
    nvs_set_u32(h, A1_NVS_KEY_STEP, 0);
    nvs_set_u8(h, A1_NVS_KEY_DONE, 0);
    nvs_commit(h);
    nvs_close(h);
    return;  // *step_idx / *sweep_done already 0
  }

  uint32_t si = 0;
  uint8_t sd = 0;
  if (nvs_get_u32(h, A1_NVS_KEY_STEP, &si) == ESP_OK) {
    *step_idx = si;
  }
  if (nvs_get_u8(h, A1_NVS_KEY_DONE, &sd) == ESP_OK) {
    *sweep_done = sd;
  }
  nvs_close(h);
}

// Persist the index of the NEXT step to run. Called ONLY after a step fully
// completes (session closed + metadata finalized).
static bool a1_nvs_write_step(uint32_t next_step_idx) {
  nvs_handle_t h;
  esp_err_t e = nvs_open(A1_NVS_NAMESPACE, NVS_READWRITE, &h);
  if (e != ESP_OK) {
    ESP_LOGE(TAG, "nvs_open for step write failed: %s", esp_err_to_name(e));
    return false;
  }
  e = nvs_set_u32(h, A1_NVS_KEY_STEP, next_step_idx);
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

// Mark the whole sequence complete.
static bool a1_nvs_set_done(void) {
  nvs_handle_t h;
  esp_err_t e = nvs_open(A1_NVS_NAMESPACE, NVS_READWRITE, &h);
  if (e != ESP_OK) {
    ESP_LOGE(TAG, "nvs_open for done write failed: %s", esp_err_to_name(e));
    return false;
  }
  e = nvs_set_u8(h, A1_NVS_KEY_DONE, 1);
  if (e == ESP_OK) {
    e = nvs_commit(h);
  }
  nvs_close(h);
  return e == ESP_OK;
}

// Idle forever with a status-LED pattern that is visually distinguishable at
// a glance: slow blink (0.5 s) = sequence DONE, fast blink (0.1 s) = ABORTED.
// Both differ from the solid-ON "session running" state. Never returns.
static void a1_led_idle_forever(bool aborted) {
  const TickType_t period = aborted ? pdMS_TO_TICKS(100) : pdMS_TO_TICKS(500);
  for (;;) {
    gpio_set_level(STATUS_LED_PIN, 1);
    vTaskDelay(period);
    gpio_set_level(STATUS_LED_PIN, 0);
    vTaskDelay(period);
  }
}

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
 * Reads data from three queues and writes to SD card
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
 * @brief GPIO46 monitor (low priority)
 * Latches g_abort_requested on a press so the sequencer can consume it either
 * as the manual start signal (before a session) or as a clean-abort request
 * (during a running session), whichever applies at the time. GPIO46 is
 * active-low (idle high via external pull-up), matching the original
 * firmware.
 */
void vTaskButtonMonitor(void* pvParameters) {
  for (;;) {
    if (gpio_get_level(BUTTON_PIN) == 0) {
      if (!g_abort_requested) {
        ESP_LOGW(TAG, "GPIO46 pressed");
      }
      g_abort_requested = true;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ==================== Sequencer helpers ====================

// Cleanly stop the currently-running session: flip to STOPPING, let the SD
// writer drain + flush, record final stats, finalize meta.json, write the A1
// sidecar with the given terminal status, and close the run.
static void a1_finish_session(const run_session_t* session, uint8_t severity,
                              int step_index, uint32_t session_start_ms,
                              const char* status) {
  system_state = DAQ_STATE_STOPPING;

  // Give the acquisition tasks time to stop producing and the SD writer time
  // to drain the queues and flush.
  vTaskDelay(pdMS_TO_TICKS(A1_DRAIN_MS));

  sd_flush_all_buffers();
  statistics.duration_ms = get_timestamp_ms() - session_start_ms;

  metadata_update_statistics(session->meta_file, &statistics);
  metadata_finalize(session->meta_file);
  a1_write_meta(session->run_path, severity, step_index, status);

  sd_close_run_session();
}

// Block until GPIO46 is pressed and released, blinking the status LED to show
// the board is armed and waiting. Called ONCE per boot, before the first
// session of the sequencer loop — every session after that runs unattended,
// in lockstep with a1_batch_runner.py's zero-gap chaining on the shaker side.
//
// vTaskButtonMonitor is already running by the time this is called and will
// latch g_abort_requested on the very press we're waiting for; clear it
// afterward so the first session doesn't see a stale abort and immediately
// terminate itself.
static void a1_wait_for_start_button(uint8_t severity) {
  ESP_LOGI(TAG,
           "Armed — start a1_batch_runner.py now (first level: %s / %d RPM), "
           "then press GPIO46 to start recording...",
           A1_SEVERITY_NAME[severity], A1_RPM[severity]);

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
                "needed until all 4 sessions finish.");
}

// ==================== Main Program ====================

/**
 * @brief Main program entry point
 *
 * Runs the A1 shaker sequence: 4 back-to-back 70-minute recordings, order
 * low/medium/medium/high, resume-safe via NVS. GPIO46 starts the first
 * session of the boot; the rest auto-continue unattended, in sync with
 * a1_batch_runner.py's zero-gap chaining on the shaker side. A press during
 * any running session aborts that session.
 */
extern "C" void app_main(void) {
  ESP_LOGI(TAG, "========== Project SHIELD (A1 shaker sequencer) starting ==========");

  // Initialize data types module
  data_types_init();
  ESP_LOGI(TAG, "Data types module initialized");

  // Initialize NVS (standard ESP-IDF boilerplate; used for the A1 checkpoint)
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

  // Create FreeRTOS queues (once — reused across all 4 sessions)
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
  // first session of this boot (see a1_wait_for_start_button), a press once
  // a session is running instead aborts that session
  gpio_config_t btn_conf = {
      .pin_bit_mask = (1ULL << BUTTON_PIN),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&btn_conf);

  // Launch the worker tasks ONCE. system_state is still IDLE, so they park
  // immediately and wait for the sequencer to flip it to RUNNING per session.
  // Core 0 (PRO_CPU): Data acquisition tasks
  xTaskCreatePinnedToCore(vTaskFast, "FastTask", 4096, NULL, 10, NULL, 0);
  xTaskCreatePinnedToCore(vTaskMedium, "MediumTask", 4096, NULL, 8, NULL, 0);
  xTaskCreatePinnedToCore(vTaskSlow, "SlowTask", 4096, NULL, 6, NULL, 0);
  // Core 1 (APP_CPU): SD card write task
  xTaskCreatePinnedToCore(vTaskSDWriter, "SDWriter", 8192, NULL, 5, NULL, 1);
  // Low-priority button monitor
  xTaskCreatePinnedToCore(vTaskButtonMonitor, "BtnMon", 2048, NULL, 2, NULL, 1);

  // ---- Resume from the NVS checkpoint ----
  uint32_t step_idx = 0;
  uint8_t sweep_done = 0;
  a1_nvs_read_checkpoint(&step_idx, &sweep_done);
  ESP_LOGI(TAG, "A1 checkpoint: step_idx=%u sweep_done=%u",
           (unsigned)step_idx, (unsigned)sweep_done);

  if (sweep_done) {
    ESP_LOGI(TAG, "A1 sequence already complete — nothing to do. Idling.");
    a1_led_idle_forever(false);  // slow-blink "done" (never returns)
  }

  if (step_idx >= A1_TOTAL_STEPS) {
    // Defensive: checkpoint past the end but done-flag not set. Treat as done.
    ESP_LOGW(TAG, "step_idx %u >= %u but sweep_done unset — marking done",
             (unsigned)step_idx, (unsigned)A1_TOTAL_STEPS);
    a1_nvs_set_done();
    a1_led_idle_forever(false);
  }

  // ---- Wait for manual start (gates only the first session this boot) ----
  a1_wait_for_start_button(A1_PLAN[step_idx]);

  // ---- Sequencer: run steps step_idx .. A1_TOTAL_STEPS-1 ----
  uint32_t steps_completed_this_boot = 0;
  for (uint32_t i = step_idx; i < A1_TOTAL_STEPS; i++) {
    uint8_t severity = A1_PLAN[i];

    ESP_LOGI(TAG, "==== A1 step %u/%u: severity=%s rpm=%d ====", (unsigned)i,
             (unsigned)(A1_TOTAL_STEPS - 1), A1_SEVERITY_NAME[severity],
             A1_RPM[severity]);

    // Create the run session (retry once on failure).
    if (!sd_create_run_session()) {
      ESP_LOGE(TAG, "sd_create_run_session failed — retrying once");
      vTaskDelay(pdMS_TO_TICKS(500));
      if (!sd_create_run_session()) {
        ESP_LOGE(TAG, "sd_create_run_session failed again — SAFE HALT");
        a1_led_idle_forever(true);  // fast-blink error (never returns)
      }
    }

    const run_session_t* session = sd_get_current_session();
    ESP_LOGI(TAG, "Run session: %s at %s", session->run_id, session->run_path);

    // Fresh statistics for this session's meta.json.
    statistics = daq_statistics_t{};

    // meta.json (existing schema) + a1_meta.json sidecar (severity/RPM).
    if (!metadata_create(session->meta_file, session->run_id)) {
      ESP_LOGE(TAG, "metadata_create FAILED");
    }
    a1_write_meta(session->run_path, severity, (int)i, "running");

    // Start acquisition and time the fixed 70-minute session.
    system_state = DAQ_STATE_RUNNING;
    gpio_set_level(STATUS_LED_PIN, 1);
    uint32_t session_start_ms = get_timestamp_ms();

    bool aborted = false;
    while ((get_timestamp_ms() - session_start_ms) < A1_SESSION_DURATION_MS) {
      if (g_abort_requested) {
        aborted = true;
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(50));
    }

    if (aborted) {
      ESP_LOGW(TAG, "ABORT during step %u — closing run as aborted", (unsigned)i);
      a1_finish_session(session, severity, (int)i, session_start_ms, "aborted");
      gpio_set_level(STATUS_LED_PIN, 0);
      // Deliberately do NOT advance the NVS checkpoint: this step re-runs on
      // the next power cycle, gated by another button press.
      ESP_LOGW(TAG, "Halted after abort. Power-cycle to retry step %u.",
               (unsigned)i);
      a1_led_idle_forever(true);  // fast-blink "aborted" (never returns)
    }

    // Normal completion of this step.
    a1_finish_session(session, severity, (int)i, session_start_ms, "completed");
    gpio_set_level(STATUS_LED_PIN, 0);
    steps_completed_this_boot++;

    ESP_LOGI(TAG,
             "Step %u done: fast=%" PRIu32 " medium=%" PRIu32 " slow=%" PRIu32
             " overruns=%" PRIu32 " sd_errors=%" PRIu32,
             (unsigned)i, statistics.fast_samples, statistics.medium_samples,
             statistics.slow_samples, statistics.queue_overruns,
             statistics.sd_errors);

    // Checkpoint AFTER this step fully completes (session closed + finalized).
    if (!a1_nvs_write_step(i + 1)) {
      ESP_LOGE(TAG, "checkpoint write failed after step %u", (unsigned)i);
    }
  }

  // ---- All steps complete ----
  a1_nvs_set_done();

  FILE* flag = fopen(A1_COMPLETE_FLAG_PATH, "w");
  if (flag) {
    const esp_app_desc_t* app = esp_app_get_description();
    // A1_TOTAL_STEPS/A1_TOTAL_STEPS is trustworthy here (unlike the B4 sweep's
    // flag) because a1_nvs_read_checkpoint() resets a stale/mismatched
    // checkpoint to 0 before the loop above ever runs — this code path is
    // only reached once every step_index has genuinely been through
    // a1_finish_session(..., "completed"), whether in this boot or an
    // earlier resumed one.
    fprintf(flag, "A1 shaker sequence complete: %u/%u steps.\n",
            (unsigned)A1_TOTAL_STEPS, (unsigned)A1_TOTAL_STEPS);
    fprintf(flag, "steps_completed_this_boot=%u (resumed from step_idx=%u)\n",
            (unsigned)steps_completed_this_boot, (unsigned)step_idx);
    fprintf(flag, "plan=");
    for (uint32_t i = 0; i < A1_TOTAL_STEPS; i++) {
      fprintf(flag, "%s%s", A1_SEVERITY_NAME[A1_PLAN[i]],
              (i + 1 < A1_TOTAL_STEPS) ? "," : "\n");
    }
    fprintf(flag, "firmware_commit=%s\n", app ? app->version : "unknown");
    fclose(flag);
    ESP_LOGI(TAG, "Wrote completion marker: %s", A1_COMPLETE_FLAG_PATH);
  } else {
    ESP_LOGE(TAG, "Failed to write completion marker: %s",
             A1_COMPLETE_FLAG_PATH);
  }

  ESP_LOGI(TAG, "========== A1 sequence COMPLETE (%u sessions) ==========",
           (unsigned)A1_TOTAL_STEPS);
  a1_led_idle_forever(false);  // slow-blink "done" (never returns)
}
