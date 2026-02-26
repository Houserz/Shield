/**
 * @file driver_ads1115.c
 * @brief Shared ADS1115 16-bit I2C ADC driver
 *
 * Supports multiple sensors on different channels (A0-A3).
 * Uses single-shot mode per read.
 *
 * Wiring:
 *   ADDR -> GND  => I2C address 0x48
 *   A0   -> Photodiode output
 *   A1   -> ACS723 Current sensor output
 */

 #include "sensor_hal.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ads1115";

/**
 * @brief Trigger single-shot conversion and read result
 */
bool ads1115_read_voltage(const ads1115_config_t *cfg, uint16_t channel, float *voltage_out) {
    if (cfg == NULL || voltage_out == NULL) return false;

    // Build config register: OS | channel | PGA | MODE_SINGLE | data_rate | comparator off
    uint16_t config = ADS1115_OS_SINGLE
                    | channel
                    | cfg->pga
                    | ADS1115_MODE_SINGLE
                    | cfg->data_rate
                    | 0x0003;  // disable comparator

    uint8_t config_bytes[3] = {
        ADS1115_CONFIG_REG,
        (uint8_t)(config >> 8),
        (uint8_t)(config & 0xFF)
    };

    // Write config to trigger conversion
    esp_err_t err = i2c_master_write_to_device(
        cfg->i2c_port, cfg->i2c_addr,
        config_bytes, sizeof(config_bytes),
        pdMS_TO_TICKS(100)
    );
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Config write failed: %s", esp_err_to_name(err));
        return false;
    }

    // Wait for conversion (depends on data rate; 128SPS = ~8ms, add margin)
    vTaskDelay(pdMS_TO_TICKS(10));

    // Point to conversion register
    uint8_t reg = ADS1115_CONV_REG;
    err = i2c_master_write_to_device(
        cfg->i2c_port, cfg->i2c_addr,
        &reg, 1,
        pdMS_TO_TICKS(100)
    );
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Register select failed: %s", esp_err_to_name(err));
        return false;
    }

    // Read 2 bytes
    uint8_t data[2] = {0};
    err = i2c_master_read_from_device(
        cfg->i2c_port, cfg->i2c_addr,
        data, 2,
        pdMS_TO_TICKS(100)
    );
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Read failed: %s", esp_err_to_name(err));
        return false;
    }

    int16_t raw = (int16_t)((data[0] << 8) | data[1]);

    // Convert raw to voltage based on PGA setting
    float fsr;
    switch (cfg->pga) {
        case ADS1115_PGA_6144MV: fsr = 6.144f; break;
        case ADS1115_PGA_4096MV: fsr = 4.096f; break;
        case ADS1115_PGA_2048MV: fsr = 2.048f; break;
        case ADS1115_PGA_1024MV: fsr = 1.024f; break;
        case ADS1115_PGA_512MV:  fsr = 0.512f; break;
        case ADS1115_PGA_256MV:  fsr = 0.256f; break;
        default:                 fsr = 2.048f; break;
    }

    *voltage_out = ((float)raw / 32768.0f) * fsr;
    return true;
}