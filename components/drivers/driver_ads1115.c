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
 *
 * REVIEW-1 CHANGE (C-11 item 2, manuscript §3.3.1 lines 333-346, §3.7 lines
 * 451-455): the original driver waited vTaskDelay(pdMS_TO_TICKS(2)) for the
 * conversion. On the recording build (CONFIG_FREERTOS_HZ=100) that is zero
 * ticks, so the driver read the conversion register immediately and returned
 * the PREVIOUS conversion. Because the current (A1) and photodiode (A0) reads
 * alternate on one converter, every photodiode sample repeated the current
 * sample and the photodiode was never measured in Tier 2 or Tier 1 session
 * 002. This version polls the OS bit of the config register (bit 15: 1 =
 * device idle, conversion complete) with a bounded microsecond timeout, so
 * the result is independent of the tick rate.
 */

#include "sensor_hal.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ads1115";

/* Conversion at 860 SPS takes ~1.16 ms; allow 5 ms before declaring a fault. */
#define ADS1115_CONV_TIMEOUT_US   5000
#define ADS1115_POLL_INTERVAL_US  100

/* Statistics visible to the application (read with ads1115_get_stats). */
static uint32_t s_conv_timeouts = 0;
static uint32_t s_i2c_errors = 0;

void ads1115_get_stats(uint32_t *conv_timeouts, uint32_t *i2c_errors) {
    if (conv_timeouts) *conv_timeouts = s_conv_timeouts;
    if (i2c_errors) *i2c_errors = s_i2c_errors;
}

/**
 * @brief Trigger single-shot conversion, wait for completion, read result
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
        s_i2c_errors++;
        ESP_LOGE(TAG, "Config write failed: %s", esp_err_to_name(err));
        return false;
    }

    // Wait for conversion: poll the OS bit (bit 15 of the config register).
    // 0 = conversion in progress, 1 = idle (conversion complete).
    const int64_t t0 = esp_timer_get_time();
    for (;;) {
        uint8_t creg = ADS1115_CONFIG_REG;
        uint8_t cb[2] = {0, 0};
        err = i2c_master_write_read_device(
            cfg->i2c_port, cfg->i2c_addr,
            &creg, 1, cb, sizeof(cb),
            pdMS_TO_TICKS(100)
        );
        if (err != ESP_OK) {
            s_i2c_errors++;
            ESP_LOGE(TAG, "Config read-back failed: %s", esp_err_to_name(err));
            return false;
        }
        if (cb[0] & 0x80) {
            break;  // OS bit set: conversion complete
        }
        if (esp_timer_get_time() - t0 > ADS1115_CONV_TIMEOUT_US) {
            s_conv_timeouts++;
            ESP_LOGW(TAG, "Conversion timeout on channel 0x%04X", channel);
            return false;  // do not return a stale value
        }
        esp_rom_delay_us(ADS1115_POLL_INTERVAL_US);
    }

    // Point to conversion register
    uint8_t reg = ADS1115_CONV_REG;
    err = i2c_master_write_to_device(
        cfg->i2c_port, cfg->i2c_addr,
        &reg, 1,
        pdMS_TO_TICKS(100)
    );
    if (err != ESP_OK) {
        s_i2c_errors++;
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
        s_i2c_errors++;
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
