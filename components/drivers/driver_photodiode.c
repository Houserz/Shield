/**
 * @file driver_photodiode.c
 * @brief BPW34 Photodiode Driver (Vishay Silicon PIN, ADC interface, 200 Hz)
 *
 * Datasheet: Vishay BPW34/BPW34S Document Number 81521, Rev. 2.1, 23-Aug-11.
 * Hardware: ADC analog input (photodiode in photovoltaic or transimpedance circuit).
 *
 * BPW34 Basic Characteristics (T_amb = 25 °C, from datasheet):
 *   - Open circuit voltage Vo: 350 mV typ @ Ee = 1 mW/cm², λ = 950 nm
 *   - Short circuit current Ik: 47 μA typ @ Ee = 1 mW/cm², λ = 950 nm
 *   - Reverse light current Ira: 40 ~ 50 μA @ Ee = 1 mW/cm², λ = 950 nm, VR = 5 V
 *   - Spectral range λ0.1: 430 to 1100 nm; peak sensitivity λp = 900 nm
 *   - Temperature coefficient of Vo: -2.6 mV/K; of Ik: 0.1 %/K
 *
 * Output: data_out is ADC voltage (V). Irradiance approx: Ee [mW/cm²] ≈ V / 0.35 at 950 nm.
 * Uses ESP-IDF esp_adc oneshot driver (same API style as driver_current.c).
 */

#include "sensor_hal.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include <stdlib.h>

static const char *TAG = "bpw34";

/** BPW34 typical open-circuit voltage at 1 mW/cm², 950 nm (datasheet). Used for irradiance scale. */
#define BPW34_VO_AT_1MW_PER_CM2_MV  350.0f

static adc_oneshot_unit_handle_t s_adc1_handle = NULL;

/**
 * @brief Initialize BPW34 photodiode (ADC channel)
 *
 * Configures ADC1 channel and attenuation per ctx->hw_config (adc_config_t).
 * 12-bit resolution, 0~3.3 V range (ADC_ATTEN_DB_12).
 * If ADC1 unit is already created by another driver (e.g. current), only channel is configured.
 *
 * @param ctx Sensor context (ctx->hw_config = adc_config_t*)
 * @return true = init OK, false = invalid config or ADC error
 */
bool photodiode_init(SensorContext_t *ctx) {
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
        esp_err_t err = adc_oneshot_new_unit(&init_cfg, &s_adc1_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "adc_oneshot_new_unit failed: %s", esp_err_to_name(err));
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

    ESP_LOGI(TAG, "BPW34 init OK: ch=%d, GPIO=%d (Vo typ 350 mV @ 1 mW/cm², 950 nm)",
             adc_cfg->adc_channel, adc_cfg->gpio_pin);
    return true;
}

/**
 * @brief Read photodiode voltage from ADC
 *
 * Reads raw ADC, converts to voltage (0~3.3 V). data_out is in volts.
 * For BPW34: irradiance Ee [mW/cm²] ≈ voltage / 0.35 at λ ≈ 950 nm (typical).
 *
 * @param ctx Sensor context
 * @param data_out Output voltage in V (or use with BPW34_VO_AT_1MW_PER_CM2_MV for Ee)
 * @return true = read OK, false = read failed
 */
bool photodiode_read_sample(SensorContext_t *ctx, float *data_out) {
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

    /* 12-bit raw -> voltage (0~3.3 V with ADC_ATTEN_DB_12) */
    float voltage = (float)adc_raw * 3.3f / 4095.0f;
    *data_out = voltage;
    return true;
}
