#include "sensor_hal.h"
#include "esp_log.h"
#include "BNO08x.hpp"

static const char *TAG = "bno085_accel";
static BNO08x *imu = nullptr;

extern "C" bool accel_init(SensorContext_t *ctx) {
    if (ctx == NULL || ctx->hw_config == NULL) {
        ESP_LOGE(TAG, "Invalid context or hw_config");
        return false;
    }

    imu = (BNO08x *)ctx->hw_config;
    imu->rpt.accelerometer.enable(100000UL);
    ESP_LOGI(TAG, "Accelerometer report enabled");

    return true;
}

extern "C" bool accel_read_sample(SensorContext_t *ctx, float *data_out) {
    if (data_out == NULL || imu == nullptr) return false;

    if (!imu->data_available() || !imu->rpt.accelerometer.has_new_data()) {
        return false;
    }

    bno08x_accel_t d = imu->rpt.accelerometer.get();
    data_out[0] = sqrtf(d.x * d.x + d.y * d.y + d.z * d.z);
    return true;
}