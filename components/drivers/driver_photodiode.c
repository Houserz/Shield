/**
 * @file driver_photodiode.c
 * @brief BPW34 Photodiode Driver via ADS1115 I2C ADC (A0 channel, 200 Hz)
 *
 * Reads photodiode voltage from ADS1115 channel A0 over I2C.
 * Output: voltage in V. Irradiance approx: Ee [mW/cm²] ≈ V / 0.35 at 950 nm.
 */

#include "sensor_hal.h"
#include "esp_log.h"

static const char *TAG = "bpw34";

static ads1115_config_t s_ads1115_cfg = {
    .i2c_port  = I2C_NUM_0,
    .i2c_addr  = ADS1115_I2C_ADDR,
    .pga       = ADS1115_PGA_4096MV,   // ±4.096V range, good for 0-3.3V signal
    .data_rate = ADS1115_DR_250SPS,
};

bool photodiode_init(SensorContext_t *ctx) {
    // ADS1115 needs no explicit init beyond I2C bus being up
    // I2C is initialized in app_main before sensors are inited
    ESP_LOGI(TAG, "BPW34 photodiode init OK (ADS1115 A0, I2C 0x%02X)", ADS1115_I2C_ADDR);
    return true;
}

bool photodiode_read_sample(SensorContext_t *ctx, float *data_out) {
    if (data_out == NULL) return false;
    return ads1115_read_voltage(&s_ads1115_cfg, ADS1115_CH0, data_out);
}