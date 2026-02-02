/**
 * @file driver_vibration.c
 * @brief SW-420 Vibration Sensor Driver (GPIO interface, 1kHz sampling rate)
 *
 * Hardware: SW-420 Vibration Sensor Module (Handson Technology)
 * - Operating voltage: 3.3V~5V
 * - Digital output D0: LOW when no vibration, HIGH when vibration detected
 * - On-board LM393 Op-Amp for sensitivity adjustment
 *
 * Reference: Vibration Sensor.pdf (Handson Technology User Guide)
 */

#include "sensor_hal.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <stdlib.h>

static const char *TAG = "vibration";

/**
 * @brief Initialize SW-420 vibration sensor
 *
 * Configures GPIO as digital input. The SW-420 module actively drives the
 * output line (LOW=no vibration, HIGH=vibration), so no internal pull is needed.
 *
 * @param ctx Sensor context (hw_config must point to vibration_gpio_config_t)
 * @return true=initialization successful, false=initialization failed
 */
bool vibration_init(SensorContext_t *ctx) {
    if (ctx == NULL || ctx->hw_config == NULL) {
        ESP_LOGE(TAG, "vibration_init: null context or hw_config");
        return false;
    }

    vibration_gpio_config_t *gpio_cfg = (vibration_gpio_config_t *)ctx->hw_config;
    int pin = gpio_cfg->gpio_pin;

    if (pin < 0) {
        ESP_LOGE(TAG, "vibration_init: invalid gpio_pin %d", pin);
        return false;
    }

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "vibration_init: gpio_config failed (pin=%d) err=%d", pin, err);
        return false;
    }

    ESP_LOGI(TAG, "SW-420 initialized on GPIO%d", pin);
    return true;
}

/**
 * @brief Read vibration sensor status
 *
 * SW-420 output logic (per datasheet):
 * - GPIO LOW  (0) = no vibration  -> data_out = 0.0
 * - GPIO HIGH (1) = vibration      -> data_out = 1.0
 *
 * @param ctx Sensor context
 * @param data_out Output: 1.0=vibration detected, 0.0=no vibration
 * @return true=read successful, false=read failed
 */
bool vibration_read_sample(SensorContext_t *ctx, float *data_out) {
    if (ctx == NULL || ctx->hw_config == NULL || data_out == NULL) {
        return false;
    }

    vibration_gpio_config_t *gpio_cfg = (vibration_gpio_config_t *)ctx->hw_config;
    int level = gpio_get_level(gpio_cfg->gpio_pin);

    /* SW-420: HIGH = vibration detected, LOW = no vibration */
    *data_out = (level == 1) ? 1.0f : 0.0f;
    return true;
}
