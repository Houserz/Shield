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
#include "BNO08x.hpp"

extern "C" {
    #include "sensor_hal.h"
    #include "data_types.h"
    #include "sd_storage.h"
    #include "streamer.h"
    #include "driver/i2c.h"
    #include "driver/gpio.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "freertos/queue.h"
    #include "esp_log.h"

    // BNO085 driver declarations
    bool accel_init(SensorContext_t *ctx);
    bool accel_read_sample(SensorContext_t *ctx, float *data_out);
    bool gyro_init(SensorContext_t *ctx);
    bool gyro_read_sample(SensorContext_t *ctx, float *data_out);
    bool mag_init(SensorContext_t *ctx);
    bool mag_read_sample(SensorContext_t *ctx, float *data_out);
}

#define STATUS_LED_PIN GPIO_NUM_7
#define BUTTON_PIN GPIO_NUM_46

static const bool TESTING_SHORT_DURATION = true; // true = 120 s test run; false = 15-hour run

static const char *TAG = "SHIELD";

// ==================== Global Queue Handles ====================
static QueueHandle_t fast_queue = NULL;
static QueueHandle_t medium_queue = NULL;
static QueueHandle_t slow_queue = NULL;

// ==================== Global State ====================
static daq_state_t system_state = DAQ_STATE_IDLE;
static daq_statistics_t statistics = {0};

// ==================== Hardware Configuration Instances ====================

// BNO085 - SPI Configuration
// CS: GPIO37, SCLK: GPIO38, MOSI: GPIO40, MISO: GPIO39, INT: GPIO5, RST: GPIO6
// NOTES: 
//  - INT (GPIO5) must have a hardware pullup
//  - PS0 and PS1 pins on BNO085 must be tied to HIGH (3.3v) for SPI
static bno08x_config_t bno085_spi_cfg = []() {
    bno08x_config_t cfg;
    cfg.io_mosi = GPIO_NUM_40; // DI on BNO085
    cfg.io_miso = GPIO_NUM_39; // SDA on BNO085
    cfg.io_sclk = GPIO_NUM_38; // SCL on BNO085
    cfg.io_cs   = GPIO_NUM_37;
    cfg.io_int  = GPIO_NUM_5;
    cfg.io_rst  = GPIO_NUM_6;
    cfg.spi_peripheral = SPI3_HOST;
    return cfg;
}();

static BNO08x bno085_imu(bno085_spi_cfg);

// SW-420 - GPIO Configuration
// GPIO16: digital input, no conflict with ADC or other peripherals
static vibration_gpio_config_t vibration_gpio_cfg = {
    .gpio_pin = 16
};

// MPL3115A2 - I2C Configuration
// I2C0 bus shared with MCP9808. Use 4.7kΩ external pull-ups on SDA/SCL.
static hal_i2c_config_t mpl3115_i2c_cfg = {
    .i2c_port = 0,      // I2C_NUM_0
    .sda_pin = 17,       // GPIO17
    .scl_pin = 18,       // GPIO18
    .device_addr = 0x60 // MPL3115A2 default I2C address
};

// MCP9808 - I2C Configuration
// Shares same I2C0 bus with MPL3115 (same SDA=17, SCL=18 pins)
static hal_i2c_config_t mcp9808_i2c_cfg = {
    .i2c_port = 0,      // I2C_NUM_0 (shared with MPL3115)
    .sda_pin = 17,       // GPIO17 (same as MPL3115)
    .scl_pin = 18,       // GPIO18 (same as MPL3115)
    .device_addr = 0x18 // MCP9808 default I2C address
};

// INMP441 - I2S Configuration (high sample rate)
// BCK/WS/SD use GPIO45/47/48 to avoid conflict with other peripherals
static inmp441_i2s_config_t inmp441_i2s_cfg = {
    .i2s_port = 0,         // I2S_NUM_0
    .bck_pin = 45,          // GPIO45 (bit clock)
    .ws_pin = 47,           // GPIO47 (word select / LRCLK)
    .data_in_pin = 48,      // GPIO48 (data input)
    .sample_rate_hz = 16000 // INMP441 typical; decimate to 1kHz logical rate
};

// ==================== Sensor Array Definition ====================
// Set .enabled = false to disable a sensor (skip init and acquisition)

#define NUM_SENSORS 9

static SensorContext_t my_sensors[NUM_SENSORS] = {
    {
        .id = 1,
        .type = SENSOR_TYPE_VIBRATION,
        .sampling_rate_hz = 1000,
        .enabled = true,  // Disabled for testing
        .hw_config = &vibration_gpio_cfg,
        .init = vibration_init,
        .read_sample = vibration_read_sample,
        .process_sample = NULL
    },
    // [2] ACS723 Current Sensor - Medium Tier
    {
        .id = 2,
        .type = SENSOR_TYPE_CURRENT,
        .sampling_rate_hz = 200,
        .enabled = true,
        .hw_config = NULL,
        .init = current_init,
        .read_sample = current_read_sample,
        .process_sample = NULL
    },
    // [3] MPL3115A2 Pressure Sensor - Slow Tier
    {
        .id = 3,
        .type = SENSOR_TYPE_PRESSURE,
        .sampling_rate_hz = 50,
        .enabled = true,
        .hw_config = &mpl3115_i2c_cfg,
        .init = mpl3115_init,
        .read_sample = mpl3115_read_sample,
        .process_sample = NULL
    },
    // [4] MCP9808 Temperature Sensor - Slow Tier
    {
        .id = 4,
        .type = SENSOR_TYPE_TEMP,
        .sampling_rate_hz = 50,
        .enabled = true,
        .hw_config = &mcp9808_i2c_cfg,
        .init = mcp9808_init,
        .read_sample = mcp9808_read_sample,
        .process_sample = NULL
    },
    // [5] INMP441 Microphone - Fast Tier (high sample rate)
    {
        .id = 5,
        .type = SENSOR_TYPE_MICROPHONE,
        .sampling_rate_hz = 1000,
        .enabled = true,  // Disabled for testing
        .hw_config = &inmp441_i2s_cfg,
        .init = inmp441_init,
        .read_sample = inmp441_read_sample,
        .process_sample = NULL
    },
    // [6] 751-1015-ND Photodiode - Medium Tier (medium sample rate)
    {
        .id = 6,
        .type = SENSOR_TYPE_PHOTODIODE,
        .sampling_rate_hz = 200,
        .enabled = true,
        .hw_config = NULL,
        .init = photodiode_init,
        .read_sample = photodiode_read_sample,
        .process_sample = NULL
    },
    // [7] BNO085 Magnetometer - Fast Tier
    {
        .id = 7,
        .type = SENSOR_TYPE_MAGNETOMETER,
        .sampling_rate_hz = 1000,
        .enabled = true,
        .hw_config = &bno085_imu,
        .init = mag_init,
        .read_sample = mag_read_sample,
        .process_sample = mag_process_sample
    },
    // [8] BNO085 Gyroscope - Fast Tier
    {
        .id = 8,
        .type = SENSOR_TYPE_GYROSCOPE,
        .sampling_rate_hz = 1000,
        .enabled = true,
        .hw_config = &bno085_imu,
        .init = gyro_init,
        .read_sample = gyro_read_sample,
        .process_sample = gyro_process_sample
    },
    // [9] BNO085 Accelerometer - Fast Tier
    {
        .id = 9,
        .type = SENSOR_TYPE_ACCELEROMETER,
        .sampling_rate_hz = 1000,
        .enabled = true,
        .hw_config = &bno085_imu,
        .init = accel_init,
        .read_sample = accel_read_sample,
        .process_sample = accel_process_sample
    }
};

static uint8_t sensor_axis_count(const SensorContext_t *sensor) {
    if (!sensor) return 1;
    switch (sensor->type) {
        case SENSOR_TYPE_ACCELEROMETER:
        case SENSOR_TYPE_GYROSCOPE:
        case SENSOR_TYPE_MAGNETOMETER:
            return 3;
        default:
            return 1;
    }
}

static void copy_sensor_data(float *dst, const float *src, uint8_t axis_count) {
    for (uint8_t i = 0; i < 3; i++) {
        dst[i] = (i < axis_count) ? src[i] : 0.0f;
    }
}

static bool process_sensor_sample(SensorContext_t *sensor,
                                  const float *raw,
                                  float *processed,
                                  uint8_t *flags_out) {
    uint8_t axis_count = sensor_axis_count(sensor);

    if (sensor && sensor->process_sample &&
        sensor->process_sample(sensor, raw, processed, flags_out)) {
        return true;
    }

    copy_sensor_data(processed, raw, axis_count);
    if (flags_out) {
        *flags_out = DATA_FLAG_PROCESSED_SAME_AS_RAW;
    }
    return true;
}

static sensor_data_record_v2_t make_sensor_record(uint32_t timestamp_ms,
                                                  const SensorContext_t *sensor,
                                                  data_kind_t kind,
                                                  uint8_t flags,
                                                  const float *data) {
    sensor_data_record_v2_t rec = {
        .timestamp_ms = timestamp_ms,
        .sensor_id = (uint8_t)sensor->id,
        .kind = (uint8_t)kind,
        .axis_count = sensor_axis_count(sensor),
        .flags = flags,
        .data = {0.0f, 0.0f, 0.0f}
    };
    copy_sensor_data(rec.data, data, rec.axis_count);
    return rec;
}

static void note_record_stored(const sensor_data_record_v2_t *rec, int tier_hz) {
    if (!rec) return;

    if (tier_hz == 1000) {
        statistics.fast_records++;
    } else if (tier_hz == 200) {
        statistics.medium_records++;
    } else if (tier_hz == 50) {
        statistics.slow_records++;
    }

    if (rec->kind == DATA_KIND_RAW) {
        statistics.raw_records++;
    } else if (rec->kind == DATA_KIND_PROCESSED) {
        statistics.processed_records++;
    }

    if (rec->sensor_id <= MAX_SENSOR_ID) {
        statistics.sensor_records[rec->sensor_id]++;
    }
}

static void note_logical_sample(const SensorContext_t *sensor) {
    if (!sensor) return;

    if (sensor->sampling_rate_hz == 1000) {
        statistics.fast_samples++;
    } else if (sensor->sampling_rate_hz == 200) {
        statistics.medium_samples++;
    } else if (sensor->sampling_rate_hz == 50) {
        statistics.slow_samples++;
    }

    if ((uint8_t)sensor->id <= MAX_SENSOR_ID) {
        statistics.sensor_samples[(uint8_t)sensor->id]++;
    }
}

static bool enqueue_fast_record(const sensor_data_record_v2_t *rec) {
    streamer_publish_record(rec);
    fast_queue_msg_t msg = {
        .type = QUEUE_MSG_DATA,
        .data = *rec
    };

    if (xQueueSend(fast_queue, &msg, 0) != pdTRUE) {
        statistics.queue_overruns++;
        return false;
    }

    note_record_stored(rec, 1000);
    return true;
}

static bool enqueue_medium_record(const sensor_data_record_v2_t *rec) {
    streamer_publish_record(rec);
    medium_queue_msg_t msg = {
        .type = QUEUE_MSG_DATA,
        .data = *rec
    };

    if (xQueueSend(medium_queue, &msg, 0) != pdTRUE) {
        statistics.queue_overruns++;
        return false;
    }

    note_record_stored(rec, 200);
    return true;
}

static bool enqueue_slow_record(const sensor_data_record_v2_t *rec) {
    streamer_publish_record(rec);
    slow_queue_msg_t msg = {
        .type = QUEUE_MSG_DATA,
        .data = *rec
    };

    if (xQueueSend(slow_queue, &msg, 0) != pdTRUE) {
        statistics.queue_overruns++;
        return false;
    }

    note_record_stored(rec, 50);
    return true;
}

// ==================== Core 0 Acquisition Tasks ====================

/**
 * @brief Fast task (1kHz, Core 0)
 * For high-speed sensors (BNO085 raw sensors, SW-420 Vibration, INMP441 Microphone)
 */
void vTaskFast(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1) > 0 ? pdMS_TO_TICKS(1) : 1;

    while (system_state == DAQ_STATE_RUNNING) {
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (!my_sensors[i].enabled || my_sensors[i].sampling_rate_hz != 1000) continue;

            float raw[3] = {0.0f, 0.0f, 0.0f};
            if (my_sensors[i].read_sample(&my_sensors[i], raw)) {
                float processed[3] = {0.0f, 0.0f, 0.0f};
                uint8_t processed_flags = DATA_FLAG_NONE;
                uint32_t timestamp_ms = get_timestamp_ms();
                process_sensor_sample(&my_sensors[i], raw, processed, &processed_flags);

                sensor_data_record_v2_t raw_rec =
                    make_sensor_record(timestamp_ms, &my_sensors[i], DATA_KIND_RAW, DATA_FLAG_NONE, raw);
                sensor_data_record_v2_t processed_rec =
                    make_sensor_record(timestamp_ms, &my_sensors[i], DATA_KIND_PROCESSED, processed_flags, processed);

                enqueue_fast_record(&raw_rec);
                enqueue_fast_record(&processed_rec);
                note_logical_sample(&my_sensors[i]);
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
    const TickType_t xFrequency = pdMS_TO_TICKS(5) > 0 ? pdMS_TO_TICKS(5) : 1;

    while (system_state == DAQ_STATE_RUNNING) {
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (my_sensors[i].enabled && my_sensors[i].sampling_rate_hz == 200) {
                float raw[3] = {0.0f, 0.0f, 0.0f};
                if (my_sensors[i].read_sample(&my_sensors[i], raw)) {
                    float processed[3] = {0.0f, 0.0f, 0.0f};
                    uint8_t processed_flags = DATA_FLAG_NONE;
                    uint32_t timestamp_ms = get_timestamp_ms();
                    process_sensor_sample(&my_sensors[i], raw, processed, &processed_flags);

                    sensor_data_record_v2_t raw_rec =
                        make_sensor_record(timestamp_ms, &my_sensors[i], DATA_KIND_RAW, DATA_FLAG_NONE, raw);
                    sensor_data_record_v2_t processed_rec =
                        make_sensor_record(timestamp_ms, &my_sensors[i], DATA_KIND_PROCESSED, processed_flags, processed);

                    enqueue_medium_record(&raw_rec);
                    enqueue_medium_record(&processed_rec);
                    note_logical_sample(&my_sensors[i]);
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
    const TickType_t xFrequency = pdMS_TO_TICKS(20);

    while (system_state == DAQ_STATE_RUNNING) {
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (my_sensors[i].enabled && my_sensors[i].sampling_rate_hz == 50) {
                float raw[3] = {0.0f, 0.0f, 0.0f};
                if (my_sensors[i].read_sample(&my_sensors[i], raw)) {
                    float processed[3] = {0.0f, 0.0f, 0.0f};
                    uint8_t processed_flags = DATA_FLAG_NONE;
                    uint32_t timestamp_ms = get_timestamp_ms();
                    process_sensor_sample(&my_sensors[i], raw, processed, &processed_flags);

                    sensor_data_record_v2_t raw_rec =
                        make_sensor_record(timestamp_ms, &my_sensors[i], DATA_KIND_RAW, DATA_FLAG_NONE, raw);
                    sensor_data_record_v2_t processed_rec =
                        make_sensor_record(timestamp_ms, &my_sensors[i], DATA_KIND_PROCESSED, processed_flags, processed);

                    enqueue_slow_record(&raw_rec);
                    enqueue_slow_record(&processed_rec);
                    note_logical_sample(&my_sensors[i]);
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
    
    uint32_t last_flush_time = 0;
    uint32_t last_metadata_update = 0;
    uint32_t empty_after_stop_since = 0;
    
    while (true) {
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
        
        // Periodically flush buffers; metadata is much less frequent to reduce FAT churn.
        uint32_t current_time = get_timestamp_ms();
        if (current_time - last_flush_time > 1000) {
            sd_flush_all_buffers();
            if (system_state == DAQ_STATE_RUNNING) {
                statistics.duration_ms = current_time;
            }
            last_flush_time = current_time;
        }

        if (current_time - last_metadata_update > 60000) {
            const run_session_t *session = sd_get_current_session();
            if (session->is_active) {
                metadata_update_statistics(session->meta_file, &statistics);
            }

            last_metadata_update = current_time;
        }

        bool queues_empty =
            uxQueueMessagesWaiting(fast_queue) == 0 &&
            uxQueueMessagesWaiting(medium_queue) == 0 &&
            uxQueueMessagesWaiting(slow_queue) == 0;
        if (system_state != DAQ_STATE_RUNNING && queues_empty) {
            if (empty_after_stop_since == 0) {
                empty_after_stop_since = current_time;
            } else if (current_time - empty_after_stop_since > 200) {
                break;
            }
        } else {
            empty_after_stop_since = 0;
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
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "========== Project SHIELD starting ==========");

    // Initialize data types module
    data_types_init();
    ESP_LOGI(TAG, "Data types module initialized");

    // Real-time streamer (USB primary + Wi-Fi SoftAP secondary)
    streamer_init();

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
    const char *sensor_names[] = {"SW-420 Vibration", "ACS723 Current", "MPL3115 Pressure", "MCP9808 Temp",
                                  "INMP441 Microphone", "751-1015-ND Photodiode", "BNO085 Magnetometer", "BNO085 Gyroscope", "BNO085 Accelerometer"};
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

    // Configure status LED (GPIO 7) - on while acquiring, off when done
    gpio_config_t led_cfg = {
        .pin_bit_mask = (1ULL << STATUS_LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&led_cfg);

    // Configure button input (GPIO 46) - press to stop data acquisition
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn_conf);

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
    
    // Continue to run data acquisition until button press or 15 hours have elapsed
    // while (gpio_get_level(BUTTON_PIN) && system_state == DAQ_STATE_RUNNING &&
    //        (get_timestamp_ms() - acq_start_ms < 15UL * 3600000UL)) {
    //   vTaskDelay(pdMS_TO_TICKS(100));
    // }
    //Run for 15 hours. Split into 1-hour chunks to avoid pdMS_TO_TICKS() overflow
    if (!TESTING_SHORT_DURATION) {
        for (int hour = 1; hour <= 15 && system_state == DAQ_STATE_RUNNING; hour++) {
            vTaskDelay(pdMS_TO_TICKS(3600 * 1000));
            ESP_LOGI(TAG, "Hour %d/15 completed (%"PRIu32" ms elapsed)",
                    hour, get_timestamp_ms() - acq_start_ms);
        }
    } else {
        // Testing: run for 120 seconds
        vTaskDelay(pdMS_TO_TICKS(120 * 1000));
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
    metadata_update_statistics(session->meta_file, &statistics);
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
    ESP_LOGI(TAG, "Records: fast=%"PRIu32" medium=%"PRIu32" slow=%"PRIu32" raw=%"PRIu32" processed=%"PRIu32,
             statistics.fast_records, statistics.medium_records, statistics.slow_records,
             statistics.raw_records, statistics.processed_records);
    ESP_LOGI(TAG, "Stream: usb_sent=%"PRIu32" wifi_sent=%"PRIu32" drops=%"PRIu32,
             streamer_get_sent_usb(), streamer_get_sent_wifi(), streamer_get_drops());
}
