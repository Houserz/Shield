#include "sensor_hal.h"
#include "BNO08x.hpp"
#include "esp_log.h"

static const char *TAG = "bno085_mag";
static BNO08x *imu = nullptr;

extern "C" bool mag_init(SensorContext_t *ctx) {
    if (ctx == NULL || ctx->hw_config == NULL) return false;

    imu = (BNO08x *)ctx->hw_config;
    imu->rpt.cal_magnetometer.enable(1000UL);

    ESP_LOGI(TAG, "Magnetometer report enabled");
    return true;
}

extern "C" bool mag_read_sample(SensorContext_t *ctx, float *data_out) {
    if (data_out == NULL || imu == nullptr) return false;

    if (!imu->rpt.cal_magnetometer.has_new_data()) {
        return false;
    }

    bno08x_magf_t d = imu->rpt.cal_magnetometer.get();
    data_out[0] = sqrtf(d.x * d.x + d.y * d.y + d.z * d.z);
    return true;
}