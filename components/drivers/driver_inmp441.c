/**
 * @file driver_inmp441.c
 * @brief INMP441 MEMS Microphone Driver (I2S interface)
 *
 * Implements INMP441 per InvenSense datasheet DS-INMP441-00 Rev 1.1.
 * - I2S Philips format, 24-bit two's complement, MSB-first
 * - 64 SCK cycles per WS stereo frame (fSCK = 64 × fWS)
 * - SCK: 0.5–3.2 MHz, WS: 7.8–50 kHz
 * - Startup: 2^18 SCK cycles (~85 ms @ 3.072 MHz) after power-up
 */

#include "sensor_hal.h"
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>

static const char *TAG = "inmp441";

// INMP441 datasheet: 24-bit data, 64 SCK per WS frame, Philips I2S format
// Read 4ms per call (~64 samples) for more stable RMS; stereo to support L/R=GND or L/R=VDD
#define INMP441_SAMPLES_PER_READ 64
#define INMP441_READ_BLOCK       (INMP441_SAMPLES_PER_READ * 2)  // Stereo: L,R,L,R...

// 24-bit two's complement: full scale = 2^23 - 1 ≈ 8388607
#define INMP441_FULL_SCALE       (8388607.0f)

typedef struct {
    i2s_chan_handle_t rx_handle;
    inmp441_i2s_config_t *cfg;
    bool initialized;
    float last_rms;  // Last valid RMS for fallback
} inmp441_state_t;

static inmp441_state_t s_inmp441 = {0};

// Convert 24-bit two's complement to float [-1, 1]
// ESP32 I2S 24-bit: sample is in HIGH 24 bits of 32-bit word (LS byte padding)
static inline float inmp441_sample_to_float(uint32_t raw) {
    uint32_t s24 = (raw >> 8) & 0x00FFFFFF;
    int32_t s32;
    if (s24 & 0x00800000) {
        s32 = (int32_t)(s24 | 0xFF000000);
    } else {
        s32 = (int32_t)s24;
    }
    return (float)s32 / INMP441_FULL_SCALE;
}

bool inmp441_init(SensorContext_t *ctx) {
    if (ctx == NULL || ctx->hw_config == NULL) return false;

    inmp441_i2s_config_t *cfg = (inmp441_i2s_config_t *)ctx->hw_config;

    if (cfg->bck_pin < 0 || cfg->ws_pin < 0 || cfg->data_in_pin < 0) {
        ESP_LOGE(TAG, "INMP441: Pin config incomplete (bck=%d ws=%d data_in=%d)",
                 cfg->bck_pin, cfg->ws_pin, cfg->data_in_pin);
        return false;
    }

    int sample_rate = cfg->sample_rate_hz;
    if (sample_rate < 7800 || sample_rate > 50000) {
        ESP_LOGW(TAG, "INMP441: sample_rate %d out of spec (7.8–50 kHz), using 16000", sample_rate);
        sample_rate = 16000;
    }

    // Channel config
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG((i2s_port_t)cfg->i2s_port, I2S_ROLE_MASTER);
    chan_cfg.dma_frame_num = 128;
    chan_cfg.auto_clear = true;

    esp_err_t ret = i2s_new_channel(&chan_cfg, NULL, &s_inmp441.rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "INMP441: i2s_new_channel failed %s", esp_err_to_name(ret));
        return false;
    }

    // STD config: Philips I2S, 24-bit, stereo (read both L/R - breakout L/R pin varies)
    // Datasheet: "64 SCK cycles per WS stereo frame", "MSB delayed by one SCK" → Philips
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_24BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = cfg->bck_pin,
            .ws = cfg->ws_pin,
            .dout = I2S_GPIO_UNUSED,
            .din = cfg->data_in_pin,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    // For 24-bit, MCLK multiple should be divisible by 3 (per ESP-IDF doc)
    std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_384;

    ret = i2s_channel_init_std_mode(s_inmp441.rx_handle, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "INMP441: i2s_channel_init_std_mode failed %s", esp_err_to_name(ret));
        i2s_del_channel(s_inmp441.rx_handle);
        s_inmp441.rx_handle = NULL;
        return false;
    }

    ret = i2s_channel_enable(s_inmp441.rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "INMP441: i2s_channel_enable failed %s", esp_err_to_name(ret));
        i2s_channel_disable(s_inmp441.rx_handle);
        i2s_del_channel(s_inmp441.rx_handle);
        s_inmp441.rx_handle = NULL;
        return false;
    }

    // Datasheet: "Zero output for the first 2^18 SCK cycles (85 ms @ 3.072 MHz) following power-up"
    vTaskDelay(pdMS_TO_TICKS(90));

    s_inmp441.cfg = cfg;
    s_inmp441.initialized = true;
    s_inmp441.last_rms = 0.0f;

    ESP_LOGI(TAG, "INMP441: Init OK (sample_rate=%d Hz)", sample_rate);
    return true;
}

bool inmp441_read_sample(SensorContext_t *ctx, float *data_out) {
    if (ctx == NULL || data_out == NULL || !s_inmp441.initialized) return false;

    // Read one block (~1 ms @ 16 kHz): INMP441_READ_BLOCK samples (stereo frame, we use left)
    // 24-bit in 32-bit slot: 4 bytes per sample
    uint32_t buf[INMP441_READ_BLOCK];
    size_t bytes_read = 0;

    esp_err_t ret = i2s_channel_read(s_inmp441.rx_handle, buf, sizeof(buf), &bytes_read, pdMS_TO_TICKS(100));
    if (ret != ESP_OK || bytes_read == 0) {
        *data_out = s_inmp441.last_rms;
        return true;  // Non-fatal: return last valid
    }

    // Stereo: buf = [L0,R0,L1,R1,...]; use max(L_rms, R_rms) - INMP441 L/R pin varies by breakout
    size_t num_pairs = bytes_read / (2 * sizeof(uint32_t));
    if (num_pairs == 0) {
        *data_out = s_inmp441.last_rms;
        return true;
    }

    float sum_sq_left = 0.0f, sum_sq_right = 0.0f;
    for (size_t i = 0; i < num_pairs; i++) {
        float sl = inmp441_sample_to_float(buf[i * 2]);
        float sr = inmp441_sample_to_float(buf[i * 2 + 1]);
        sum_sq_left += sl * sl;
        sum_sq_right += sr * sr;
    }
    float rms_left = sqrtf(sum_sq_left / (float)num_pairs);
    float rms_right = sqrtf(sum_sq_right / (float)num_pairs);
    float rms = (rms_left > rms_right) ? rms_left : rms_right;
    s_inmp441.last_rms = rms;
    *data_out = rms;
    return true;
}
