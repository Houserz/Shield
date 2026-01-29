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
 */

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "sensor_hal.h"
#include "data_types.h"
#include "sd_storage.h"

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

// BNO085 - SPI Configuration
// TODO: Confirm ESP32-S3 SPI pin assignment
static spi_config_t bno085_spi_cfg = {
    .spi_host = 2,      // TODO: Choose SPI2 or SPI3
    .cs_pin = 0,        // TODO: Set actual CS pin
    .sclk_pin = 0,      // TODO: Set actual SCLK pin
    .mosi_pin = 0,      // TODO: Set actual MOSI pin
    .miso_pin = 0       // TODO: Set actual MISO pin
};

// SW-420 - GPIO Configuration
// TODO: Confirm ESP32-S3 GPIO pin
static gpio_config_t vibration_gpio_cfg = {
    .gpio_pin = 0       // TODO: Set actual GPIO pin (any GPIO supporting input)
};

// ACS723 - ADC Configuration
// TODO: Confirm ESP32-S3 ADC pin (ADC1: GPIO1-10, ADC2 not recommended when using WiFi)
static adc_config_t current_adc_cfg = {
    .adc_channel = 0,   // TODO: Set actual ADC channel
    .gpio_pin = 0       // TODO: Set corresponding GPIO pin (e.g., GPIO1 corresponds to ADC1_CH0)
};

// MPL3115A2 - I2C Configuration
// TODO: Confirm ESP32-S3 I2C pins (any GPIO will work)
static i2c_config_t mpl3115_i2c_cfg = {
    .i2c_port = 0,      // I2C_NUM_0
    .sda_pin = 0,       // TODO: Set actual SDA pin
    .scl_pin = 0,       // TODO: Set actual SCL pin
    .device_addr = 0x60 // MPL3115A2 default I2C address
};

// MCP9808 - I2C Configuration
// TODO: Shares same I2C bus with MPL3115 (same SDA and SCL pins)
static i2c_config_t mcp9808_i2c_cfg = {
    .i2c_port = 0,      // I2C_NUM_0 (shared with MPL3115)
    .sda_pin = 0,       // TODO: Same as MPL3115
    .scl_pin = 0,       // TODO: Same as MPL3115
    .device_addr = 0x18 // MCP9808 default I2C address
};

// ==================== Sensor Array Definition ====================

#define NUM_SENSORS 5

static SensorContext_t my_sensors[NUM_SENSORS] = {
    // [0] BNO085 IMU - Fast Tier
    {
        .id = 0,
        .type = SENSOR_TYPE_IMU,
        .sampling_rate_hz = 1000,
        .hw_config = &bno085_spi_cfg,
        .init = bno085_init,
        .read_sample = bno085_read_sample
    },
    // [1] SW-420 Vibration Sensor - Fast Tier
    {
        .id = 1,
        .type = SENSOR_TYPE_VIBRATION,
        .sampling_rate_hz = 1000,
        .hw_config = &vibration_gpio_cfg,
        .init = vibration_init,
        .read_sample = vibration_read_sample
    },
    // [2] ACS723 Current Sensor - Medium Tier
    {
        .id = 2,
        .type = SENSOR_TYPE_CURRENT,
        .sampling_rate_hz = 200,
        .hw_config = &current_adc_cfg,
        .init = current_init,
        .read_sample = current_read_sample
    },
    // [3] MPL3115A2 Pressure Sensor - Slow Tier
    {
        .id = 3,
        .type = SENSOR_TYPE_PRESSURE,
        .sampling_rate_hz = 50,
        .hw_config = &mpl3115_i2c_cfg,
        .init = mpl3115_init,
        .read_sample = mpl3115_read_sample
    },
    // [4] MCP9808 Temperature Sensor - Slow Tier
    {
        .id = 4,
        .type = SENSOR_TYPE_TEMP,
        .sampling_rate_hz = 50,
        .hw_config = &mcp9808_i2c_cfg,
        .init = mcp9808_init,
        .read_sample = mcp9808_read_sample
    }
};

// ==================== Core 0 Acquisition Tasks ====================

/**
 * @brief Fast task (1kHz, Core 0)
 * For high-speed sensors (BNO085 IMU, SW-420 Vibration)
 */
void vTaskFast(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1);  // 1ms = 1kHz
    
    while (system_state == DAQ_STATE_RUNNING) {
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (my_sensors[i].sampling_rate_hz == 1000) {
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
 * For medium-speed sensors (ACS723 Current)
 */
void vTaskMedium(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5);  // 5ms = 200Hz
    
    while (system_state == DAQ_STATE_RUNNING) {
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (my_sensors[i].sampling_rate_hz == 200) {
                float data = 0.0f;
                if (my_sensors[i].read_sample(&my_sensors[i], &data)) {
                    medium_queue_msg_t msg = {
                        .type = QUEUE_MSG_DATA,
                        .data = {
                            .timestamp_ms = get_timestamp_ms(),
                            .current = data
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
            if (my_sensors[i].sampling_rate_hz == 50) {
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

    // Initialize all sensors
    const char *sensor_names[] = {"BNO085 IMU", "SW-420 Vibration", "ACS723 Current", "MPL3115 Pressure", "MCP9808 Temp"};
    for (int i = 0; i < NUM_SENSORS; i++) {
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

    // Set to running state
    system_state = DAQ_STATE_RUNNING;
    ESP_LOGI(TAG, "System state -> RUNNING, launching tasks...");

    // Core 0 (PRO_CPU): Data acquisition tasks
    xTaskCreatePinnedToCore(vTaskFast, "FastTask", 4096, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(vTaskMedium, "MediumTask", 4096, NULL, 8, NULL, 0);
    xTaskCreatePinnedToCore(vTaskSlow, "SlowTask", 4096, NULL, 6, NULL, 0);

    // Core 1 (APP_CPU): SD card write task
    xTaskCreatePinnedToCore(vTaskSDWriter, "SDWriter", 8192, NULL, 5, NULL, 1);
    ESP_LOGI(TAG, "All tasks launched. Acquiring for 30 seconds...");

    // Automatically stops after 30 seconds (for testing)
    vTaskDelay(pdMS_TO_TICKS(30000));

    // Stop acquisition
    system_state = DAQ_STATE_STOPPING;
    ESP_LOGI(TAG, "System state -> STOPPING");

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

    ESP_LOGI(TAG, "========== Project SHIELD finished ==========");
    ESP_LOGI(TAG, "Stats: fast=%"PRIu32" medium=%"PRIu32" slow=%"PRIu32" overruns=%"PRIu32" sd_errors=%"PRIu32,
             statistics.fast_samples, statistics.medium_samples, statistics.slow_samples,
             statistics.queue_overruns, statistics.sd_errors);
}

