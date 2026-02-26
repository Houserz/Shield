/**
 * @file driver_current.c
 * @brief ACS723 Current Sensor Driver via ADS1115 I2C ADC (A1 channel, 200 Hz)
 *
 * Datasheet: Allegro ACS723-DS, Rev. 2
 * Hardware: ACS723 VIOUT -> ADS1115 A1, VCC = 5V, GND, BW_SEL -> GND (80kHz)
 *
 * Variant defaults: ACS723LLCTR-10AB (±10A, 200 mV/A, bidirectional)
 * To change variant, update SENS_MV_PER_A and VCC_V below:
 *   5AB:  400 mV/A, ±5A,  VIOUT(Q) = VCC * 0.5
 *   10AB: 200 mV/A, ±10A, VIOUT(Q) = VCC * 0.5
 *   20AB: 100 mV/A, ±20A, VIOUT(Q) = VCC * 0.5
 *   40AB:  50 mV/A, ±40A, VIOUT(Q) = VCC * 0.5
 *
 * Conversion formula (bidirectional):
 *   VIOUT(Q) = VCC * 0.5 = 2.5V at 0A
 *   IP (A) = (VIOUT - VIOUT(Q)) / Sens
 */

#include "sensor_hal.h"
#include "esp_log.h"

static const char *TAG = "acs723";

#define ACS723_VCC_V          5.0f    // VCC supply voltage (must be 5V)
#define ACS723_SENS_MV_PER_A  200.0f  // Sensitivity: change if using different variant
#define ACS723_VIOUT_Q        (ACS723_VCC_V * 0.5f)  // Zero-current output voltage (2.5V)

static ads1115_config_t s_ads1115_cfg = {
    .i2c_port  = I2C_NUM_0,
    .i2c_addr  = ADS1115_I2C_ADDR,
    .pga       = ADS1115_PGA_4096MV,  // ±4.096V, covers 0~5V VIOUT with headroom
    .data_rate = ADS1115_DR_250SPS,
};

bool current_init(SensorContext_t *ctx) {
    ESP_LOGI(TAG, "ACS723 init OK (ADS1115 A1, I2C 0x%02X, sens=%.0f mV/A)",
             ADS1115_I2C_ADDR, ACS723_SENS_MV_PER_A);
    return true;
}

bool current_read_sample(SensorContext_t *ctx, float *data_out) {
    if (data_out == NULL) return false;

    float voltage = 0.0f;
    if (!ads1115_read_voltage(&s_ads1115_cfg, ADS1115_CH1, &voltage)) {
        return false;
    }

    // Convert voltage to current (A)
    // IP = (VIOUT - VIOUT(Q)) / Sens
    *data_out = (voltage - ACS723_VIOUT_Q) / (ACS723_SENS_MV_PER_A / 1000.0f);
    return true;
}