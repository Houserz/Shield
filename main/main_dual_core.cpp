/**
 * @file main_dual_core.cpp
 * @brief Project SHIELD - Dual-Core Data Acquisition System Main Program
 *        Branch feature/review1-validation (review-1 platform fixes, C-11)
 *
 * Core 0 (PRO_CPU): Data acquisition tasks
 * Core 1 (APP_CPU): SD card storage task
 *
 * Based on the generic one-shot recorder of branch shaker-table-tests
 * (ba0ab6c, with the green/red status LEDs). Acquisition cadence, record
 * layouts, sensor set and the tick-based timestamp are UNCHANGED so that the
 * new recordings are comparable with the released Tier 2 corpus.
 *
 * What is new (numbering follows the C-11 spec):
 *   1. Storage: periodic fsync, checked close, boot self-test (sd_storage.c).
 *   2. ADS1115: conversion-ready polling instead of a zero-tick wait
 *      (driver_ads1115.c). The photodiode is converted again.
 *   3. Gate 0 G0.11 stays in the host-side converter (no firmware part).
 *   4. Board identity: factory MAC written into meta.json device_info.
 *   5. RTC: serial command "SETTIME <unix-seconds>" sets the clock so
 *      meta.json start_time is a real date, not 1970-01-01.
 *   6. Operator markers: serial "MARK <label>", or a SHORT press (<1 s) of
 *      the GPIO46 button, writes a MARK line to events.log with the record
 *      clock. A LONG press (>= 3 s) stops the session. (The released
 *      firmware stopped on any press; see BUTTON_SHORT_PRESS_IS_MARKER.)
 *   7. Provenance: firmware git hash, tick rate, BNO085 report IDs and
 *      intervals, and the hub's dynamic-calibration config read back through
 *      sh2_getCalConfig() when the SH2 header is reachable.
 *   8. Close verification: after the session closes, the tier-file sizes are
 *      read back from the card and written to meta.json and events.log; a
 *      zero-length tier file latches the red LED so the operator sees the
 *      loss at the bench.
 */
#include "BNO08x.hpp"

#if __has_include("sh2.h")
extern "C" {
#include "sh2.h"
}
#define HAVE_SH2_HEADER 1
#else
#define HAVE_SH2_HEADER 0
#endif

#include <inttypes.h>
#include <stdio.h>

extern "C" {
#include "data_types.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "gaussian.h"
#include "sd_storage.h"
#include "sensor_hal.h"
#include "sdkconfig.h"
#include <sys/time.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>

// BNO085 driver declarations
bool accel_init(SensorContext_t* ctx);
bool accel_read_sample(SensorContext_t* ctx, float* data_out);
bool gyro_init(SensorContext_t* ctx);
bool gyro_read_sample(SensorContext_t* ctx, float* data_out);
bool mag_init(SensorContext_t* ctx);
bool mag_read_sample(SensorContext_t* ctx, float* data_out);
void ads1115_get_stats(uint32_t* conv_timeouts, uint32_t* i2c_errors);
}

// On-board status LEDs (per PCB netlist: GPIO4=Green LED1, GPIO7=Blue LED2, GPIO8=Red LED3)
#define GREEN_ACTIVITY_LED_PIN GPIO_NUM_4  // Flashes while the SD writer task is reading/writing
#define RED_ERROR_LED_PIN GPIO_NUM_8       // Latches on if any SD write fails or a tier file is empty at close
#define BUTTON_PIN GPIO_NUM_46

#define RUN_DURATION 120UL   // safety cap in minutes; the operator stops earlier with a long press
#define NOISE_INJECTION_DELAY 30UL
#define MS_PER_HR 3600000UL
#define MS_PER_MIN 60000UL
#define NOISE_INJECTION false  // never on for the review-1 validation and campaign builds

// Button semantics (item 6). Set to false to restore "any press stops".
#define BUTTON_SHORT_PRESS_IS_MARKER true
#define BUTTON_LONG_PRESS_MS 3000
#define BUTTON_SHORT_PRESS_MAX_MS 1000

// BNO085 report configuration as requested by the drivers (item 7). Keep in
// step with driver_accelerometer.cpp / driver_gyroscope.cpp / driver_magnetometer.cpp.
#define BNO085_ACCEL_INTERVAL_US 1000UL
#define BNO085_GYRO_INTERVAL_US  1000UL
#define BNO085_MAG_INTERVAL_US   1000UL

// Metadata statistics rewrite period (was 1 s; each rewrite is a FAT remove+rename)
#define META_UPDATE_PERIOD_MS 10000UL

static const char* TAG = "SHIELD";

// ==================== Global Queue Handles ====================
static QueueHandle_t fast_queue = NULL;
static QueueHandle_t medium_queue = NULL;
static QueueHandle_t slow_queue = NULL;

// ==================== Global State ====================
static daq_state_t system_state = DAQ_STATE_IDLE;
static daq_statistics_t statistics = {};
static volatile bool g_stop_requested = false;
static volatile uint32_t g_marker_count = 0;

// ==================== Hardware Configuration Instances ====================

// BNO085 - SPI Configuration
// CS: GPIO37, SCLK: GPIO38, MOSI: GPIO40, MISO: GPIO39, INT: GPIO5, RST: GPIO6
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
static vibration_gpio_config_t vibration_gpio_cfg = {.gpio_pin = 16};

// MPL3115A2 - I2C Configuration (I2C0 shared with MCP9808)
static hal_i2c_config_t mpl3115_i2c_cfg = {
    .i2c_port = 0, .sda_pin = 17, .scl_pin = 18, .device_addr = 0x60};

// MCP9808 - I2C Configuration
static hal_i2c_config_t mcp9808_i2c_cfg = {
    .i2c_port = 0, .sda_pin = 17, .scl_pin = 18, .device_addr = 0x18};

// INMP441 - I2S Configuration
static inmp441_i2s_config_t inmp441_i2s_cfg = {
    .i2s_port = 0, .bck_pin = 45, .ws_pin = 47, .data_in_pin = 48, .sample_rate_hz = 16000};

// ==================== Sensor Array Definition (UNCHANGED) ====================

#define NUM_SENSORS 9

static SensorContext_t my_sensors[NUM_SENSORS] = {
    {.id = 1, .type = SENSOR_TYPE_VIBRATION,     .sampling_rate_hz = 1000, .enabled = true, .hw_config = &vibration_gpio_cfg, .init = vibration_init,  .read_sample = vibration_read_sample},
    {.id = 2, .type = SENSOR_TYPE_CURRENT,       .sampling_rate_hz = 200,  .enabled = true, .hw_config = NULL,                .init = current_init,    .read_sample = current_read_sample},
    {.id = 3, .type = SENSOR_TYPE_PRESSURE,      .sampling_rate_hz = 50,   .enabled = true, .hw_config = &mpl3115_i2c_cfg,    .init = mpl3115_init,    .read_sample = mpl3115_read_sample},
    {.id = 4, .type = SENSOR_TYPE_TEMP,          .sampling_rate_hz = 50,   .enabled = true, .hw_config = &mcp9808_i2c_cfg,    .init = mcp9808_init,    .read_sample = mcp9808_read_sample},
    {.id = 5, .type = SENSOR_TYPE_MICROPHONE,    .sampling_rate_hz = 1000, .enabled = true, .hw_config = &inmp441_i2s_cfg,    .init = inmp441_init,    .read_sample = inmp441_read_sample},
    {.id = 6, .type = SENSOR_TYPE_PHOTODIODE,    .sampling_rate_hz = 200,  .enabled = true, .hw_config = NULL,                .init = photodiode_init, .read_sample = photodiode_read_sample},
    {.id = 7, .type = SENSOR_TYPE_MAGNETOMETER,  .sampling_rate_hz = 1000, .enabled = true, .hw_config = &bno085_imu,         .init = mag_init,        .read_sample = mag_read_sample},
    {.id = 8, .type = SENSOR_TYPE_GYROSCOPE,     .sampling_rate_hz = 1000, .enabled = true, .hw_config = &bno085_imu,         .init = gyro_init,       .read_sample = gyro_read_sample},
    {.id = 9, .type = SENSOR_TYPE_ACCELEROMETER, .sampling_rate_hz = 1000, .enabled = true, .hw_config = &bno085_imu,         .init = accel_init,      .read_sample = accel_read_sample}};

// ==================== Core 0 Acquisition Tasks (UNCHANGED cadence) ====================

void vTaskFast(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1) > 0 ? pdMS_TO_TICKS(1) : 1;

  while (system_state == DAQ_STATE_RUNNING) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (!my_sensors[i].enabled || my_sensors[i].sampling_rate_hz != 1000) continue;

      float data[3] = {0};
      if (my_sensors[i].read_sample(&my_sensors[i], data)) {
        fast_queue_msg_t msg = {.type = QUEUE_MSG_DATA,
                                .data = {.timestamp_ms = get_timestamp_ms(),
                                         .sensor_id = (uint8_t)my_sensors[i].id,
                                         .reserved = {0},
                                         .data = {data[0], data[1], data[2]}}};
        if (xQueueSend(fast_queue, &msg, 0) != pdTRUE) statistics.queue_overruns++;
        else statistics.fast_samples++;
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  vTaskDelete(NULL);
}

void vTaskMedium(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(5) > 0 ? pdMS_TO_TICKS(5) : 1;

  while (system_state == DAQ_STATE_RUNNING) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (my_sensors[i].enabled && my_sensors[i].sampling_rate_hz == 200) {
        float data = 0.0f;
        if (my_sensors[i].read_sample(&my_sensors[i], &data)) {
          medium_queue_msg_t msg = {.type = QUEUE_MSG_DATA,
                                    .data = {.timestamp_ms = get_timestamp_ms(),
                                             .sensor_id = (uint8_t)my_sensors[i].id,
                                             .reserved = {0},
                                             .data = data}};
          if (xQueueSend(medium_queue, &msg, 0) != pdTRUE) statistics.queue_overruns++;
          else statistics.medium_samples++;
        }
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  vTaskDelete(NULL);
}

void vTaskSlow(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20);

  while (system_state == DAQ_STATE_RUNNING) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (my_sensors[i].enabled && my_sensors[i].sampling_rate_hz == 50) {
        float data = 0.0f;
        if (my_sensors[i].read_sample(&my_sensors[i], &data)) {
          slow_queue_msg_t msg = {.type = QUEUE_MSG_DATA,
                                  .data = {.timestamp_ms = get_timestamp_ms(),
                                           .sensor_id = (uint8_t)my_sensors[i].id,
                                           .reserved = {0},
                                           .data = data}};
          if (xQueueSend(slow_queue, &msg, 0) != pdTRUE) statistics.queue_overruns++;
          else statistics.slow_samples++;
        }
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  vTaskDelete(NULL);
}

// ==================== Core 1 Storage Task ====================

void vTaskSDWriter(void* pvParameters) {
  fast_queue_msg_t fast_msg;
  medium_queue_msg_t medium_msg;
  slow_queue_msg_t slow_msg;

  uint32_t last_flush = 0;
  uint32_t last_meta_update = 0;

  while (system_state == DAQ_STATE_RUNNING) {
    bool has_data = false;

    while (xQueueReceive(fast_queue, &fast_msg, 0) == pdTRUE) {
      if (fast_msg.type == QUEUE_MSG_DATA) {
        if (!sd_write_fast_data(&fast_msg.data)) { statistics.sd_errors++; gpio_set_level(RED_ERROR_LED_PIN, 1); }
        has_data = true;
      }
    }
    while (xQueueReceive(medium_queue, &medium_msg, 0) == pdTRUE) {
      if (medium_msg.type == QUEUE_MSG_DATA) {
        if (!sd_write_medium_data(&medium_msg.data)) { statistics.sd_errors++; gpio_set_level(RED_ERROR_LED_PIN, 1); }
        has_data = true;
      }
    }
    while (xQueueReceive(slow_queue, &slow_msg, 0) == pdTRUE) {
      if (slow_msg.type == QUEUE_MSG_DATA) {
        if (!sd_write_slow_data(&slow_msg.data)) { statistics.sd_errors++; gpio_set_level(RED_ERROR_LED_PIN, 1); }
        has_data = true;
      }
    }

    gpio_set_level(GREEN_ACTIVITY_LED_PIN, has_data ? 1 : 0);

    uint32_t now = get_timestamp_ms();

    // Flush (libc -> VFS) every second; fsync (VFS -> card) is done inside
    // flush_buffer every SD_SYNC_INTERVAL_MS.
    if (now - last_flush > 1000) {
      gpio_set_level(GREEN_ACTIVITY_LED_PIN, 1);
      if (!sd_flush_all_buffers()) { statistics.sd_errors++; gpio_set_level(RED_ERROR_LED_PIN, 1); }
      statistics.duration_ms = now;
      last_flush = now;
      gpio_set_level(GREEN_ACTIVITY_LED_PIN, has_data ? 1 : 0);
    }

    // Statistics into meta.json every META_UPDATE_PERIOD_MS (each rewrite is a FAT remove+rename)
    if (now - last_meta_update > META_UPDATE_PERIOD_MS) {
      const run_session_t* session = sd_get_current_session();
      if (session->is_active) metadata_update_statistics(session->meta_file, &statistics);
      last_meta_update = now;
    }

    if (!has_data) vTaskDelay(pdMS_TO_TICKS(10));
  }

  gpio_set_level(GREEN_ACTIVITY_LED_PIN, 1);
  sd_flush_all_buffers();
  sd_sync_all();
  gpio_set_level(GREEN_ACTIVITY_LED_PIN, 0);
  vTaskDelete(NULL);
}

// ==================== Serial console: SETTIME / MARK / STOP / STATUS (items 5, 6) ====================

static void handle_console_line(char* line) {
  // trim trailing whitespace
  size_t n = strlen(line);
  while (n > 0 && (line[n - 1] == ' ' || line[n - 1] == '\t')) line[--n] = '\0';

  if (strncmp(line, "SETTIME ", 8) == 0) {
    long long epoch = atoll(line + 8);
    if (epoch > 1600000000LL) {
      struct timeval tv = {.tv_sec = (time_t)epoch, .tv_usec = 0};
      settimeofday(&tv, NULL);
      char msg[96];
      snprintf(msg, sizeof(msg), "SETTIME epoch=%lld t_ms=%" PRIu32, epoch, get_timestamp_ms());
      sd_write_log("INFO", msg);
      ESP_LOGI(TAG, "%s", msg);
    } else {
      ESP_LOGW(TAG, "SETTIME: bad epoch '%s'", line + 8);
    }
  } else if (strncmp(line, "MARK", 4) == 0) {
    const char* label = (line[4] == ' ') ? line + 5 : "";
    sd_write_marker(label);
    g_marker_count = g_marker_count + 1;
    ESP_LOGI(TAG, "MARK #%" PRIu32 " '%s' at t_ms=%" PRIu32, g_marker_count, label, get_timestamp_ms());
  } else if (strcmp(line, "STOP") == 0) {
    ESP_LOGI(TAG, "STOP requested from console");
    sd_write_log("INFO", "STOP requested from console");
    g_stop_requested = true;
  } else if (strcmp(line, "STATUS") == 0) {
    uint32_t to = 0, ie = 0;
    ads1115_get_stats(&to, &ie);
    ESP_LOGI(TAG, "t_ms=%" PRIu32 " fast=%" PRIu32 " medium=%" PRIu32 " slow=%" PRIu32
             " overruns=%" PRIu32 " sd_errors=%" PRIu32 " ads_timeouts=%" PRIu32
             " ads_i2c_err=%" PRIu32 " markers=%" PRIu32,
             get_timestamp_ms(), statistics.fast_samples, statistics.medium_samples,
             statistics.slow_samples, statistics.queue_overruns, statistics.sd_errors,
             to, ie, g_marker_count);
  } else {
    ESP_LOGW(TAG, "console: unknown command '%s' (SETTIME <epoch> | MARK <label> | STOP | STATUS)", line);
  }
}

// UART0 line reader. If the devkit routes the console over USB-Serial-JTAG
// instead of UART0, switch this to the usb_serial_jtag driver (see the C-11
// note); the button marker works regardless.
void vTaskConsole(void* pvParameters) {
  // Review-1: the boards are attached through the S3's native USB (USB-Serial-JTAG), which is only a
  // secondary console in this sdkconfig, so UART0 never sees what the laptop types. Read both:
  // UART0 (a USB-UART adapter on GPIO43/44) and the USB-Serial-JTAG port (the COM port the laptop sees).
  bool have_uart = false, have_usj = false;
  uart_config_t cfg = {};
  cfg.baud_rate = 115200;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.source_clk = UART_SCLK_DEFAULT;
  if (uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0) == ESP_OK) {
    uart_param_config(UART_NUM_0, &cfg);
    have_uart = true;
  }
  usb_serial_jtag_driver_config_t ucfg = {};
  ucfg.tx_buffer_size = 256;
  ucfg.rx_buffer_size = 1024;
  if (usb_serial_jtag_driver_install(&ucfg) == ESP_OK) {
    have_usj = true;
  }
  ESP_LOGI(TAG, "console input: UART0 %s, USB-Serial-JTAG %s",
           have_uart ? "on" : "off", have_usj ? "on" : "off");
  if (!have_uart && !have_usj) {
    ESP_LOGW(TAG, "console: no input driver; serial commands disabled (button still works)");
    vTaskDelete(NULL);
    return;
  }

  static char line[96];
  int n = 0;
  for (;;) {
    uint8_t c = 0;
    int r = 0;
    if (have_uart) {
      r = uart_read_bytes(UART_NUM_0, &c, 1, have_usj ? 0 : pdMS_TO_TICKS(100));
    }
    if (r != 1 && have_usj) {
      r = usb_serial_jtag_read_bytes(&c, 1, pdMS_TO_TICKS(have_uart ? 20 : 100));
    }
    if (r == 1) {
      if (c == '\n' || c == '\r') {
        line[n] = '\0';
        if (n > 0) handle_console_line(line);
        n = 0;
      } else if (n < (int)sizeof(line) - 1) {
        line[n++] = (char)c;
      }
    }
    if (system_state == DAQ_STATE_STOPPING) break;
  }
  vTaskDelete(NULL);
}

// ==================== Button: short press = MARK, long press = STOP (item 6) ====================

// Returns true when a stop is requested. Called every 20 ms from the main loop.
static bool poll_button(void) {
  static uint32_t pressed_since_ms = 0;
  static bool was_pressed = false;
  static bool long_fired = false;

  bool pressed = (gpio_get_level(BUTTON_PIN) == 0);
  uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

  if (pressed && !was_pressed) {
    pressed_since_ms = now;
    long_fired = false;
  }
  if (pressed && !long_fired && (now - pressed_since_ms) >= BUTTON_LONG_PRESS_MS) {
    long_fired = true;
    ESP_LOGI(TAG, "Button long press -> STOP");
    sd_write_log("INFO", "STOP requested by button (long press)");
    was_pressed = pressed;
    return true;
  }
  if (!pressed && was_pressed && !long_fired) {
    uint32_t held = now - pressed_since_ms;
    if (!BUTTON_SHORT_PRESS_IS_MARKER) {
      ESP_LOGI(TAG, "Button press -> STOP");
      was_pressed = pressed;
      return true;
    }
    if (held <= BUTTON_SHORT_PRESS_MAX_MS) {
      sd_write_marker("button");
      g_marker_count = g_marker_count + 1;
      ESP_LOGI(TAG, "Button short press -> MARK #%" PRIu32 " at t_ms=%" PRIu32, g_marker_count, get_timestamp_ms());
      // acknowledge: two quick green blinks
      for (int i = 0; i < 2; i++) { gpio_set_level(GREEN_ACTIVITY_LED_PIN, 0); vTaskDelay(pdMS_TO_TICKS(60)); gpio_set_level(GREEN_ACTIVITY_LED_PIN, 1); vTaskDelay(pdMS_TO_TICKS(60)); }
    } else {
      ESP_LOGW(TAG, "Button held %" PRIu32 " ms: ignored (short <%d ms = MARK, long >=%d ms = STOP)", held, BUTTON_SHORT_PRESS_MAX_MS, BUTTON_LONG_PRESS_MS);
    }
  }
  was_pressed = pressed;
  return false;
}

// ==================== Main Program ====================

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "========== Project SHIELD starting (review-1 validation build %s) ==========", FW_GIT_HASH);

  data_types_init();

  // Provenance block for meta.json (items 4, 7)
  meta_provenance_t prov = {};
  metadata_read_board_mac(prov.board_mac, sizeof(prov.board_mac));
  strncpy(prov.firmware_git, FW_GIT_HASH, sizeof(prov.firmware_git) - 1);
  prov.tick_hz = CONFIG_FREERTOS_HZ;
  prov.accel_interval_us = BNO085_ACCEL_INTERVAL_US;
  prov.gyro_interval_us = BNO085_GYRO_INTERVAL_US;
  prov.mag_interval_us = BNO085_MAG_INTERVAL_US;
  prov.accel_report = "SH2_ACCELEROMETER (0x01, calibrated, gravity included)";
  prov.gyro_report = "SH2_GYROSCOPE_UNCALIBRATED (0x07)";
  prov.mag_report = "SH2_MAGNETIC_FIELD_UNCALIBRATED (0x0F)";
  prov.bno085_cal_config = -1;
  prov.rtc_set = false;
  ESP_LOGI(TAG, "Board MAC %s, tick %d Hz", prov.board_mac, CONFIG_FREERTOS_HZ);

  // SD card initialization (with self-test, item 1)
  if (!sd_storage_init()) {
    ESP_LOGE(TAG, "SD card initialization FAILED - aborting");
    gpio_config_t err_cfg = {.pin_bit_mask = (1ULL << RED_ERROR_LED_PIN), .mode = GPIO_MODE_OUTPUT,
                             .pull_up_en = GPIO_PULLUP_DISABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE,
                             .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&err_cfg);
    gpio_set_level(RED_ERROR_LED_PIN, 1);
    return;
  }
  prov.sd_selftest_attempts = sd_get_selftest_attempts();

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
  vTaskDelay(pdMS_TO_TICKS(100));

  if (!bno085_imu.initialize()) {
    ESP_LOGE(TAG, "BNO085 initialize() FAILED - aborting");
    return;
  }
  ESP_LOGI(TAG, "BNO085 initialized OK");

#if HAVE_SH2_HEADER
  {
    // Read back the hub's dynamic-calibration configuration (item 7). This
    // firmware never changes it; the value is the hub's power-on default.
    uint8_t cal = 0;
    if (sh2_getCalConfig(&cal) == SH2_OK) {
      prov.bno085_cal_config = (int)cal;
      ESP_LOGI(TAG, "BNO085 dynamic-calibration config = 0x%02x (1=accel 2=gyro 4=mag 8=planar)", cal);
    } else {
      ESP_LOGW(TAG, "BNO085 sh2_getCalConfig failed; meta.json will say 'not read back'");
    }
  }
#else
  ESP_LOGW(TAG, "sh2.h not reachable from main; dynamic-calibration config not read back (add the SH2 dir to esp32_BNO08x INCLUDE_DIRS)");
#endif

  // Initialize all sensors
  const char* sensor_names[] = {"SW-420 Vibration", "ACS723 Current", "MPL3115 Pressure",
                                "MCP9808 Temp", "INMP441 Microphone", "751-1015-ND Photodiode",
                                "BNO085 Magnetometer", "BNO085 Gyroscope", "BNO085 Accelerometer"};
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!my_sensors[i].enabled) { ESP_LOGI(TAG, "Sensor [%d] %s disabled - skipping", i, sensor_names[i]); continue; }
    bool ok = my_sensors[i].init(&my_sensors[i]);
    ESP_LOGI(TAG, "  Sensor [%d] %s %s", i, sensor_names[i], ok ? "initialized OK" : "initialization FAILED");
  }

  // LEDs and button
  gpio_config_t led_cfg = {.pin_bit_mask = (1ULL << GREEN_ACTIVITY_LED_PIN) | (1ULL << RED_ERROR_LED_PIN),
                           .mode = GPIO_MODE_OUTPUT, .pull_up_en = GPIO_PULLUP_DISABLE,
                           .pull_down_en = GPIO_PULLDOWN_DISABLE, .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&led_cfg);
  gpio_set_level(GREEN_ACTIVITY_LED_PIN, 0);
  gpio_set_level(RED_ERROR_LED_PIN, 0);

  gpio_config_t btn_conf = {.pin_bit_mask = (1ULL << BUTTON_PIN), .mode = GPIO_MODE_INPUT,
                            .pull_up_en = GPIO_PULLUP_DISABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE,
                            .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&btn_conf);

  // Console task (item 5/6): SETTIME before the session if the laptop is attached.
  xTaskCreatePinnedToCore(vTaskConsole, "Console", 4096, NULL, 3, NULL, 1);

  // Give the operator 5 s to send SETTIME before the session opens (optional).
  ESP_LOGI(TAG, "Waiting 5 s for optional 'SETTIME <epoch>' on the console...");
  vTaskDelay(pdMS_TO_TICKS(5000));
  prov.rtc_set = (time(NULL) > 1600000000);
  metadata_set_provenance(&prov);

  // Create run session
  if (!sd_create_run_session()) {
    ESP_LOGE(TAG, "Run session creation FAILED - aborting");
    gpio_set_level(RED_ERROR_LED_PIN, 1);
    return;
  }
  const run_session_t* session = sd_get_current_session();
  ESP_LOGI(TAG, "Run session created: %s at %s", session->run_id, session->run_path);

  if (!metadata_create(session->meta_file, session->run_id)) ESP_LOGE(TAG, "Metadata file creation FAILED");

  fast_queue = xQueueCreate(FAST_QUEUE_SIZE, sizeof(fast_queue_msg_t));
  medium_queue = xQueueCreate(MEDIUM_QUEUE_SIZE, sizeof(medium_queue_msg_t));
  slow_queue = xQueueCreate(SLOW_QUEUE_SIZE, sizeof(slow_queue_msg_t));
  if (!fast_queue || !medium_queue || !slow_queue) { ESP_LOGE(TAG, "Queue creation FAILED - aborting"); return; }

  system_state = DAQ_STATE_RUNNING;
  ESP_LOGI(TAG, "System state -> RUNNING, launching tasks...");

  xTaskCreatePinnedToCore(vTaskFast, "FastTask", 4096, NULL, 10, NULL, 0);
  xTaskCreatePinnedToCore(vTaskMedium, "MediumTask", 4096, NULL, 8, NULL, 0);
  xTaskCreatePinnedToCore(vTaskSlow, "SlowTask", 4096, NULL, 6, NULL, 0);
  xTaskCreatePinnedToCore(vTaskSDWriter, "SDWriter", 8192, NULL, 5, NULL, 1);

  uint32_t acq_start_ms = get_timestamp_ms();
  sd_write_log("INFO", "Acquisition START");
  ESP_LOGI(TAG, "Acquisition START at %" PRIu32 " ms since boot", acq_start_ms);

  // Run until: long button press, console STOP, or the safety cap.
  while (system_state == DAQ_STATE_RUNNING && !g_stop_requested &&
         (get_timestamp_ms() - acq_start_ms < RUN_DURATION * MS_PER_MIN)) {
    if (poll_button()) break;
    if (NOISE_INJECTION && !noise_injection_is_enabled() &&
        get_timestamp_ms() - acq_start_ms >= NOISE_INJECTION_DELAY * MS_PER_MIN) {
      noise_injection_set_enabled(true);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }

  uint32_t acq_end_ms = get_timestamp_ms();
  statistics.duration_ms = acq_end_ms - acq_start_ms;
  system_state = DAQ_STATE_STOPPING;
  ESP_LOGI(TAG, "Acquisition STOP at %" PRIu32 " ms (ran %" PRIu32 " ms = %.2f h)", acq_end_ms,
           statistics.duration_ms, statistics.duration_ms / 3600000.0f);

  // Let the tasks exit and the writer do its final flush + sync
  vTaskDelay(pdMS_TO_TICKS(1500));
  sd_flush_all_buffers();
  metadata_update_statistics(session->meta_file, &statistics);

  // Close and verify (item 8)
  bool close_ok = sd_close_run_session();
  sd_close_report_t rep = {};
  sd_get_last_close(&rep);
  metadata_finalize(session->meta_file, rep.fast_bytes, rep.medium_bytes, rep.slow_bytes,
                    close_ok, g_marker_count);

  uint32_t ads_to = 0, ads_ie = 0;
  ads1115_get_stats(&ads_to, &ads_ie);
  ESP_LOGI(TAG, "CLOSE %s: fast=%" PRIu32 " B medium=%" PRIu32 " B slow=%" PRIu32
           " B | fast=%" PRIu32 " medium=%" PRIu32 " slow=%" PRIu32 " samples | overruns=%" PRIu32
           " sd_errors=%" PRIu32 " ads_timeouts=%" PRIu32 " ads_i2c_err=%" PRIu32 " markers=%" PRIu32,
           close_ok ? "OK" : "FAIL", rep.fast_bytes, rep.medium_bytes, rep.slow_bytes,
           statistics.fast_samples, statistics.medium_samples, statistics.slow_samples,
           statistics.queue_overruns, statistics.sd_errors, ads_to, ads_ie, g_marker_count);

  vQueueDelete(fast_queue);
  vQueueDelete(medium_queue);
  vQueueDelete(slow_queue);
  sd_storage_deinit();

  gpio_set_level(GREEN_ACTIVITY_LED_PIN, 0);
  gpio_set_level(RED_ERROR_LED_PIN, close_ok ? 0 : 1);   // red stays on: the operator must not trust this session

  ESP_LOGI(TAG, "========== Project SHIELD finished (%s) ==========", close_ok ? "session verified" : "SESSION NOT VERIFIED - red LED");
}
