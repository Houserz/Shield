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
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "sensor_hal.h"
#include "data_types.h"
#include "sd_storage.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#define STATUS_LED_PIN GPIO_NUM_4

static const char *TAG = "SHIELD";

// ==================== Global Queue Handles ====================
static QueueHandle_t fast_queue = NULL;
static QueueHandle_t medium_queue = NULL;
static QueueHandle_t slow_queue = NULL;

// ==================== Global State ====================
static daq_state_t system_state = DAQ_STATE_IDLE;
static daq_statistics_t statistics = {0};

// ==================== Hardware Configuration Instances ====================
// TODO: Modify the following pin configurations based on actual hardware connections

// BNO085 - SPI Configuration (per Adafruit BNO085 datasheet)
// SPI requires: CS, SCLK, MOSI, MISO, INT (data ready), RST (reset)
// IMPORTANT: PS0 and PS1 must be BOTH HIGH (3.3V) for SPI mode. If both are GND, chip stays in I2C mode!
static spi_config_t bno085_spi_cfg = {
    .spi_host = 2,      // SPI2 or SPI3
    .cs_pin = 7,       // TODO: CS pin
    .sclk_pin = 6,     // TODO: SCLK (SCK)
    .mosi_pin = 8,     // TODO: MOSI (DI on BNO085)
    .miso_pin = 9,     // TODO: MISO (SDA on BNO085)
    .int_pin = 15,      // TODO: INT pin (data ready, active low) - use -1 if not connected
    .rst_pin = 16       // TODO: RST pin (reset, active low) - use -1 if not connected
};

// SW-420 - GPIO Configuration
// GPIO10: digital input, no conflict with ADC or other peripherals
static vibration_gpio_config_t vibration_gpio_cfg = {
    .gpio_pin = 16
};

// ACS723 - ADC Configuration
// ADC1_CH0 = GPIO1. ACS723 output (5V supply) needs a voltage divider (e.g. 2:3)
// to bring max output below 3.3V before connecting to this pin.
static adc_config_t current_adc_cfg = {
    .adc_channel = 0,   // ADC1_CH0
    .gpio_pin = 1       // GPIO1
};

// MPL3115A2 - I2C Configuration
// I2C0 bus shared with MCP9808. Use 4.7kΩ external pull-ups on SDA/SCL.
static hal_i2c_config_t mpl3115_i2c_cfg = {
    .i2c_port = 0,      // I2C_NUM_0
    .sda_pin = 17,       // GPIO3
    .scl_pin = 18,       // GPIO4
    .device_addr = 0x60 // MPL3115A2 default I2C address
};

// MCP9808 - I2C Configuration
// Shares same I2C0 bus with MPL3115 (same SDA=3, SCL=4 pins)
static hal_i2c_config_t mcp9808_i2c_cfg = {
    .i2c_port = 0,      // I2C_NUM_0 (shared with MPL3115)
    .sda_pin = 17,       // GPIO3 (same as MPL3115)
    .scl_pin = 18,       // GPIO4 (same as MPL3115)
    .device_addr = 0x18 // MCP9808 default I2C address
};

// INMP441 - I2S Configuration (high sample rate)
// BCK/WS/SD use GPIO17/18/21 to avoid conflict with BNO085 INT=15 and RST=16
static inmp441_i2s_config_t inmp441_i2s_cfg = {
    .i2s_port = 0,         // I2S_NUM_0
    .bck_pin = 45,          // GPIO17 (bit clock)
    .ws_pin = 47,           // GPIO18 (word select / LRCLK)
    .data_in_pin = 48,      // GPIO21 (data input)
    .sample_rate_hz = 16000 // INMP441 typical; decimate to 1kHz logical rate
};

// 751-1015-ND Photodiode - ADC Configuration (medium sample rate)
// ADC1_CH1 = GPIO2. Connect photodiode TIA/load-resistor output to this pin.
static adc_config_t photodiode_adc_cfg = {
    .adc_channel = 1,   // ADC1_CH1
    .gpio_pin = 2       // GPIO2
};

// ==================== Sensor Array Definition ====================
// Set .enabled = false to disable a sensor (skip init and acquisition)

#define NUM_SENSORS 7

static SensorContext_t my_sensors[NUM_SENSORS] = {
    // [0] BNO085 IMU - Fast Tier
    {
        .id = 0,
        .type = SENSOR_TYPE_IMU,
        .sampling_rate_hz = 1000,
        .enabled = false,
        .hw_config = &bno085_spi_cfg,
        .init = bno085_init,
        .read_sample = bno085_read_sample
    },
    // [1] SW-420 Vibration Sensor - Fast Tier
    {
        .id = 1,
        .type = SENSOR_TYPE_VIBRATION,
        .sampling_rate_hz = 1000,
        .enabled = true,
        .hw_config = &vibration_gpio_cfg,
        .init = vibration_init,
        .read_sample = vibration_read_sample
    },
    // [2] ACS723 Current Sensor - Medium Tier
    {
        .id = 2,
        .type = SENSOR_TYPE_CURRENT,
        .sampling_rate_hz = 200,
        .enabled = false,
        .hw_config = &current_adc_cfg,
        .init = current_init,
        .read_sample = current_read_sample
    },
    // [3] MPL3115A2 Pressure Sensor - Slow Tier
    {
        .id = 3,
        .type = SENSOR_TYPE_PRESSURE,
        .sampling_rate_hz = 50,
        .enabled = true,
        .hw_config = &mpl3115_i2c_cfg,
        .init = mpl3115_init,
        .read_sample = mpl3115_read_sample
    },
    // [4] MCP9808 Temperature Sensor - Slow Tier
    {
        .id = 4,
        .type = SENSOR_TYPE_TEMP,
        .sampling_rate_hz = 50,
        .enabled = true,
        .hw_config = &mcp9808_i2c_cfg,
        .init = mcp9808_init,
        .read_sample = mcp9808_read_sample
    },
    // [5] INMP441 Microphone - Fast Tier (high sample rate)
    {
        .id = 5,
        .type = SENSOR_TYPE_MICROPHONE,
        .sampling_rate_hz = 1000,
        .enabled = true,
        .hw_config = &inmp441_i2s_cfg,
        .init = inmp441_init,
        .read_sample = inmp441_read_sample
    },
    // [6] 751-1015-ND Photodiode - Medium Tier (medium sample rate)
    {
        .id = 6,
        .type = SENSOR_TYPE_PHOTODIODE,
        .sampling_rate_hz = 200,
        .enabled = true,
        .hw_config = &photodiode_adc_cfg,
        .init = photodiode_init,
        .read_sample = photodiode_read_sample
    }
};

// ==================== Core 0 Acquisition Tasks ====================

/**
 * @brief Fast task (1kHz, Core 0)
 * For high-speed sensors (BNO085 IMU, SW-420 Vibration, INMP441 Microphone)
 */
void vTaskFast(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1) > 0 ? pdMS_TO_TICKS(1) : 1;  // Min 1 tick
    
    while (system_state == DAQ_STATE_RUNNING) {
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (my_sensors[i].enabled && my_sensors[i].sampling_rate_hz == 1000) {
                float data = 0.0f;
                if (my_sensors[i].read_sample(&my_sensors[i], &data)) {
                    fast_queue_msg_t msg = {
                        .type = QUEUE_MSG_DATA,
                        .data = {
                            .timestamp_ms = get_timestamp_ms(),
                            .sensor_id = my_sensors[i].id,
                            .data = data
                        }
                    };
                    
                    if (xQueueSend(fast_queue, &msg, 0) != pdTRUE) {
                        statistics.queue_overruns++;
                    } else {
                        statistics.fast_samples++;
                    }
                }
            }
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    
    vTaskDelete(NULL);
}

/**
 * @brief Medium task (200Hz, Core 0)
 * For medium-speed sensors (ACS723 Current, 751-1015-ND Photodiode)
 */
void vTaskMedium(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5) > 0 ? pdMS_TO_TICKS(5) : 1;  // Min 1 tick
    
    while (system_state == DAQ_STATE_RUNNING) {
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (my_sensors[i].enabled && my_sensors[i].sampling_rate_hz == 200) {
                float data = 0.0f;
                if (my_sensors[i].read_sample(&my_sensors[i], &data)) {
                    medium_queue_msg_t msg = {
                        .type = QUEUE_MSG_DATA,
                        .data = {
                            .timestamp_ms = get_timestamp_ms(),
                            .sensor_id = (uint8_t)my_sensors[i].id,
                            .reserved = {0},
                            .data = data
                        }
                    };
                    
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
    
    vTaskDelete(NULL);
}

/**
 * @brief Slow task (50Hz, Core 0)
 * For low-speed sensors (MPL3115A2 Barometer, MCP9808 Temperature)
 */
void vTaskSlow(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20);  // 20ms = 50Hz

    while (system_state == DAQ_STATE_RUNNING) {
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (my_sensors[i].enabled && my_sensors[i].sampling_rate_hz == 50) {
                float data = 0.0f;
                if (my_sensors[i].read_sample(&my_sensors[i], &data)) {
                    slow_queue_msg_t msg = {
                        .type = QUEUE_MSG_DATA,
                        .data = {
                            .timestamp_ms = get_timestamp_ms(),
                            .sensor_id = my_sensors[i].id,
                            .data = data
                        }
                    };
                    
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
    
    vTaskDelete(NULL);
}

// ==================== Core 1 Storage Task ====================

/**
 * @brief SD card write task (Core 1)
 * Reads data from three queues and writes to SD card
 */
void vTaskSDWriter(void *pvParameters) {
    fast_queue_msg_t fast_msg;
    medium_queue_msg_t medium_msg;
    slow_queue_msg_t slow_msg;
    
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
            const run_session_t *session = sd_get_current_session();
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
    
    // Final flush
    sd_flush_all_buffers();
    
    vTaskDelete(NULL);
}

void i2c_scan(i2c_port_t port) {
    ESP_LOGI("i2c_scan", "Scanning I2C port %d...", port);
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        if (err == ESP_OK) {
            ESP_LOGI("i2c_scan", "  Found device at 0x%02X", addr);
        }
    }
    ESP_LOGI("i2c_scan", "Scan complete");
}

// ==================== Main Program ====================

/**
 * @brief Main program entry point
 * TODO: Modify acquisition duration, trigger method, etc. based on actual requirements
 */
void app_main(void) {
    ESP_LOGI(TAG, "========== Project SHIELD starting ==========");

    // Initialize data types module
    data_types_init();
    ESP_LOGI(TAG, "Data types module initialized");

    // SD card initialization
    ESP_LOGI(TAG, "Initializing SD card...");
    if (!sd_storage_init()) {
        ESP_LOGE(TAG, "SD card initialization FAILED - aborting");
        return;
    }
    ESP_LOGI(TAG, "SD card initialized OK");

    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 17,
        .scl_io_num = 18,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
        .clk_flags = 0,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_cfg));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
    i2c_reset_tx_fifo(I2C_NUM_0);
    i2c_reset_rx_fifo(I2C_NUM_0);
    vTaskDelay(pdMS_TO_TICKS(100));  // give devices time to settle
    i2c_scan(I2C_NUM_0);

    // Initialize all sensors
    const char *sensor_names[] = {"BNO085 IMU", "SW-420 Vibration", "ACS723 Current", "MPL3115 Pressure", "MCP9808 Temp",
                                  "INMP441 Microphone", "751-1015-ND Photodiode"};
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
            ESP_LOGE(TAG, "  Sensor [%d] %s initialization FAILED", i, sensor_names[i]);
        }
    }

    // Create run session
    ESP_LOGI(TAG, "Creating run session...");
    if (!sd_create_run_session()) {
        ESP_LOGE(TAG, "Run session creation FAILED - aborting");
        return;
    }

    const run_session_t *session = sd_get_current_session();
    ESP_LOGI(TAG, "Run session created: %s at %s", session->run_id, session->run_path);

    // Create metadata file
    if (metadata_create(session->meta_file, session->run_id)) {
        ESP_LOGI(TAG, "Metadata file created OK");
    } else {
        ESP_LOGE(TAG, "Metadata file creation FAILED");
    }

    // Create FreeRTOS queues
    fast_queue = xQueueCreate(FAST_QUEUE_SIZE, sizeof(fast_queue_msg_t));
    medium_queue = xQueueCreate(MEDIUM_QUEUE_SIZE, sizeof(medium_queue_msg_t));
    slow_queue = xQueueCreate(SLOW_QUEUE_SIZE, sizeof(slow_queue_msg_t));

    if (!fast_queue || !medium_queue || !slow_queue) {
        ESP_LOGE(TAG, "Queue creation FAILED (fast=%p medium=%p slow=%p) - aborting",
                 fast_queue, medium_queue, slow_queue);
        return;
    }
    ESP_LOGI(TAG, "FreeRTOS queues created OK");

    // Configure status LED (GPIO 4) - on while acquiring, off when done
    gpio_config_t led_cfg = {
        .pin_bit_mask = (1ULL << STATUS_LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&led_cfg);

    // Set to running state
    system_state = DAQ_STATE_RUNNING;
    gpio_set_level(STATUS_LED_PIN, 1);
    ESP_LOGI(TAG, "System state -> RUNNING, launching tasks...");

    // Core 0 (PRO_CPU): Data acquisition tasks
    xTaskCreatePinnedToCore(vTaskFast, "FastTask", 4096, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(vTaskMedium, "MediumTask", 4096, NULL, 8, NULL, 0);
    xTaskCreatePinnedToCore(vTaskSlow, "SlowTask", 4096, NULL, 6, NULL, 0);

    // Core 1 (APP_CPU): SD card write task
    xTaskCreatePinnedToCore(vTaskSDWriter, "SDWriter", 8192, NULL, 5, NULL, 1);

    uint32_t acq_start_ms = get_timestamp_ms();
    ESP_LOGI(TAG, "Acquisition START at %"PRIu32" ms since boot", acq_start_ms);
    
    // Run for 15 hours. Split into 1-hour chunks to avoid pdMS_TO_TICKS() overflow
    for (int hour = 1; hour <= 15 && system_state == DAQ_STATE_RUNNING; hour++) {
        vTaskDelay(pdMS_TO_TICKS(3600 * 1000));
        ESP_LOGI(TAG, "Hour %d/15 completed (%"PRIu32" ms elapsed)",
                 hour, get_timestamp_ms() - acq_start_ms);
    }

    uint32_t acq_end_ms = get_timestamp_ms();
    uint32_t acq_duration_ms = acq_end_ms - acq_start_ms;
    statistics.duration_ms = acq_duration_ms;

    // Stop acquisition
    system_state = DAQ_STATE_STOPPING;
    ESP_LOGI(TAG, "Acquisition STOP at %"PRIu32" ms since boot (ran %"PRIu32" ms = %.2f hours)",
             acq_end_ms, acq_duration_ms, acq_duration_ms / 3600000.0f);

    // Wait for tasks to end
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Finalize metadata
    metadata_finalize(session->meta_file);
    ESP_LOGI(TAG, "Metadata finalized");

    // Close session
    sd_close_run_session();
    ESP_LOGI(TAG, "Session closed");

    // Clean up resources
    vQueueDelete(fast_queue);
    vQueueDelete(medium_queue);
    vQueueDelete(slow_queue);
    sd_storage_deinit();
    gpio_set_level(STATUS_LED_PIN, 0);

    ESP_LOGI(TAG, "========== Project SHIELD finished ==========");
    ESP_LOGI(TAG, "Stats: fast=%"PRIu32" medium=%"PRIu32" slow=%"PRIu32" overruns=%"PRIu32" sd_errors=%"PRIu32,
             statistics.fast_samples, statistics.medium_samples, statistics.slow_samples,
             statistics.queue_overruns, statistics.sd_errors);
}

