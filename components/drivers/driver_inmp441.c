/**
 * @file driver_inmp441.c
 * @brief INMP441 MEMS Microphone Driver (I2S interface)
 *
 * Implements INMP441 per InvenSense datasheet DS-INMP441-00 Rev 1.1.
 * - I2S Philips format, 24-bit two's complement, MSB-first
 * - 64 SCK cycles per WS stereo frame (fSCK = 64 × fWS)
 * - SCK: 0.5–3.2 MHz, WS: 7.8–50 kHz
 * - Startup: 2^18 SCK cycles (~85 ms @ 3.072 MHz) after power-up
 *
 * Architecture: A dedicated background task reads I2S DMA in 1 ms blocks,
 * computes per-block RMS, and exposes the latest value via a volatile field.
 * read_sample() is non-blocking and safe to call from the 1 kHz fast task.
 */

#include "sensor_hal.h"
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>

static const char *TAG = "inmp441";

// 24-bit two's complement: full scale = 2^23 - 1
#define INMP441_FULL_SCALE          (8388607.0f)

// Max supported sample rate (INMP441 spec: 50 kHz) → max 50 stereo frames/ms
#define INMP441_MAX_FRAMES_PER_MS   50
#define INMP441_MAX_READ_BLOCK      (INMP441_MAX_FRAMES_PER_MS * 2)

// Health monitoring
#define INMP441_ERROR_THRESHOLD     50
#define INMP441_ERROR_LOG_INTERVAL  500

// Background reader task config — priority above fast task (10) to prevent DMA overflow
#define INMP441_READER_STACK        2560
#define INMP441_READER_PRIORITY     11

typedef struct {
    i2s_chan_handle_t rx_handle;
    inmp441_i2s_config_t *cfg;
    bool initialized;
    int frames_per_ms;

    volatile float latest_rms;
    volatile bool data_ready;
    volatile uint32_t consec_errors;
    volatile uint32_t total_errors;
    TaskHandle_t reader_task;
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

/**
 * Background task: continuously reads exactly 1 ms of stereo I2S data,
 * computes RMS for L/R channels, and publishes the larger one.
 * Runs at priority 11 on Core 0 to keep DMA drained in time.
 */
static void inmp441_reader_task(void *pvParameters) {
    const int fpm = s_inmp441.frames_per_ms;
    const size_t read_bytes = (size_t)fpm * 2 * sizeof(uint32_t);
    uint32_t buf[INMP441_MAX_READ_BLOCK];

    while (true) {
        size_t bytes_read = 0;
        esp_err_t ret = i2s_channel_read(s_inmp441.rx_handle, buf, read_bytes,
                                          &bytes_read, pdMS_TO_TICKS(10));

        if (ret != ESP_OK || bytes_read == 0) {
            uint32_t errs = ++s_inmp441.consec_errors;
            s_inmp441.total_errors++;
            if (errs == INMP441_ERROR_THRESHOLD) {
                ESP_LOGE(TAG, "%lu consecutive read failures — check wiring / I2S bus",
                         (unsigned long)errs);
            } else if (errs > INMP441_ERROR_THRESHOLD &&
                       (errs % INMP441_ERROR_LOG_INTERVAL) == 0) {
                ESP_LOGW(TAG, "still failing: %lu consecutive (%lu total)",
                         (unsigned long)errs, (unsigned long)s_inmp441.total_errors);
            }
            continue;
        }

        s_inmp441.consec_errors = 0;

        size_t num_pairs = bytes_read / (2 * sizeof(uint32_t));
        if (num_pairs == 0) continue;

        float sum_sq_l = 0.0f, sum_sq_r = 0.0f;
        for (size_t i = 0; i < num_pairs; i++) {
            float sl = inmp441_sample_to_float(buf[i * 2]);
            float sr = inmp441_sample_to_float(buf[i * 2 + 1]);
            sum_sq_l += sl * sl;
            sum_sq_r += sr * sr;
        }
        float rms_l = sqrtf(sum_sq_l / (float)num_pairs);
        float rms_r = sqrtf(sum_sq_r / (float)num_pairs);

        s_inmp441.latest_rms = (rms_l > rms_r) ? rms_l : rms_r;
        s_inmp441.data_ready = true;
    }
}

bool inmp441_init(SensorContext_t *ctx) {
    if (ctx == NULL || ctx->hw_config == NULL) return false;

    inmp441_i2s_config_t *cfg = (inmp441_i2s_config_t *)ctx->hw_config;

    if (cfg->bck_pin < 0 || cfg->ws_pin < 0 || cfg->data_in_pin < 0) {
        ESP_LOGE(TAG, "Pin config incomplete (bck=%d ws=%d data_in=%d)",
                 cfg->bck_pin, cfg->ws_pin, cfg->data_in_pin);
        return false;
    }

    int sample_rate = cfg->sample_rate_hz;
    if (sample_rate < 7800 || sample_rate > 50000) {
        ESP_LOGW(TAG, "sample_rate %d out of spec (7.8–50 kHz), using 16000", sample_rate);
        sample_rate = 16000;
    }

    int fpm = sample_rate / 1000;
    if (fpm < 8) fpm = 8;

    // DMA: one descriptor per ms of audio; 8 descriptors ≈ 8 ms buffer headroom
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG((i2s_port_t)cfg->i2s_port, I2S_ROLE_MASTER);
    chan_cfg.dma_frame_num = fpm;
    chan_cfg.dma_desc_num  = 8;
    chan_cfg.auto_clear    = true;

    esp_err_t ret = i2s_new_channel(&chan_cfg, NULL, &s_inmp441.rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel failed: %s", esp_err_to_name(ret));
        return false;
    }

    // Philips I2S, 24-bit, stereo (L/R breakout varies by board)
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

    std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_384;

    ret = i2s_channel_init_std_mode(s_inmp441.rx_handle, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_init_std_mode failed: %s", esp_err_to_name(ret));
        i2s_del_channel(s_inmp441.rx_handle);
        s_inmp441.rx_handle = NULL;
        return false;
    }

    ret = i2s_channel_enable(s_inmp441.rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_enable failed: %s", esp_err_to_name(ret));
        i2s_channel_disable(s_inmp441.rx_handle);
        i2s_del_channel(s_inmp441.rx_handle);
        s_inmp441.rx_handle = NULL;
        return false;
    }

    // Datasheet: zero output for the first 2^18 SCK cycles (85 ms @ 3.072 MHz)
    vTaskDelay(pdMS_TO_TICKS(90));

    s_inmp441.cfg           = cfg;
    s_inmp441.frames_per_ms = fpm;
    s_inmp441.initialized   = true;
    s_inmp441.latest_rms    = 0.0f;
    s_inmp441.data_ready    = false;
    s_inmp441.consec_errors = 0;
    s_inmp441.total_errors  = 0;

    BaseType_t xret = xTaskCreatePinnedToCore(
        inmp441_reader_task, "inmp441_rd",
        INMP441_READER_STACK, NULL,
        INMP441_READER_PRIORITY, &s_inmp441.reader_task, 0);
    if (xret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create reader task");
        i2s_channel_disable(s_inmp441.rx_handle);
        i2s_del_channel(s_inmp441.rx_handle);
        s_inmp441.rx_handle   = NULL;
        s_inmp441.initialized = false;
        return false;
    }

    ESP_LOGI(TAG, "Init OK (rate=%d Hz, %d frames/ms, 1 ms RMS window)", sample_rate, fpm);
    return true;
}

bool inmp441_read_sample(SensorContext_t *ctx, float *data_out) {
    if (ctx == NULL || data_out == NULL || !s_inmp441.initialized) return false;

    if (s_inmp441.consec_errors >= INMP441_ERROR_THRESHOLD) {
        return false;
    }

    if (!s_inmp441.data_ready) {
        *data_out = 0.0f;
        return true;
    }

    *data_out = s_inmp441.latest_rms;
    return true;
}
