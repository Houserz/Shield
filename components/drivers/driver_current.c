/**
 * @file driver_current.c
 * @brief ACS723 Current Sensor Driver (ADC, 200Hz)
 *
 * Supported chip: ACS723 (±5A, 400mV/A, bidirectional)
 *  - Current range: ±5A
 *  - Sensitivity: 400mV/A
 *  - Supply voltage: 4.5V~5.5V
 *  - Output at zero current: VIOUT(Q) = VCC×0.5; after voltage division, calculate as 1.5V
 *  - Formula: I = (VIOUT - VIOUT(Q)) / 0.4
 *
 * If VCC=5V and a 2:3 divider is used for ADC input (divider ratio 0.6), then VIOUT(Q)=1.5V.
 * For different hardware, change VOUT_QUIESCENT or SENSITIVITY_MV_PER_A as needed.
 *
 * Uses ESP-IDF esp_adc oneshot driver (non-deprecated API).
 */

#include "sensor_hal.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include <stdlib.h>

static const char *TAG = "acs723";

// ACS723 ±5A model fixed parameters
#define SENSITIVITY_MV_PER_A    400.0f   // 400mV/A
#define VOUT_QUIESCENT          1.5f     // ADC voltage at zero current (V), 5V×0.5×0.6

static adc_oneshot_unit_handle_t s_adc1_handle = NULL;

bool current_init(SensorContext_t *ctx) {
    if (ctx == NULL || ctx->hw_config == NULL) {
        ESP_LOGE(TAG, "Invalid context or hw_config");
        return false;
    }

    adc_config_t *adc_cfg = (adc_config_t *)ctx->hw_config;
    if (adc_cfg->adc_channel < 0 || adc_cfg->adc_channel > 7) {
        ESP_LOGE(TAG, "Invalid ADC1 channel: %d (0-7)", adc_cfg->adc_channel);
        return false;
    }

    adc_channel_t chan = (adc_channel_t)adc_cfg->adc_channel;

    if (s_adc1_handle == NULL) {
        adc_oneshot_unit_init_cfg_t init_cfg = {
            .unit_id = ADC_UNIT_1,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        if (adc_oneshot_new_unit(&init_cfg, &s_adc1_handle) != ESP_OK) {
            ESP_LOGE(TAG, "adc_oneshot_new_unit failed");
            return false;
        }
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12,
    };
    if (adc_oneshot_config_channel(s_adc1_handle, chan, &chan_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_config_channel failed");
        return false;
    }

    ESP_LOGI(TAG, "ACS723 init OK: ch=%d, GPIO=%d, 400mV/A ±5A",
             adc_cfg->adc_channel, adc_cfg->gpio_pin);
    return true;
}

bool current_read_sample(SensorContext_t *ctx, float *data_out) {
    if (ctx == NULL || ctx->hw_config == NULL || data_out == NULL) {
        return false;
    }

    adc_config_t *adc_cfg = (adc_config_t *)ctx->hw_config;
    adc_channel_t chan = (adc_channel_t)adc_cfg->adc_channel;
    int adc_raw = 0;

    if (adc_oneshot_read(s_adc1_handle, chan, &adc_raw) != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_read failed");
        return false;
    }

    // 12-bit raw -> voltage (0~3.3V typical with ADC_ATTEN_DB_12)
    float voltage = (float)adc_raw * 3.3f / 4095.0f;

    // I = (VIOUT - VIOUT(Q)) / Sensitivity
    *data_out = (voltage - VOUT_QUIESCENT) / (SENSITIVITY_MV_PER_A / 1000.0f);
    return true;
}
