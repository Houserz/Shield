#include "sensor_hal.h"
#include "spike_filter.h"
#include "BNO08x.hpp"
#include "esp_log.h"

static const char *TAG = "bno085_gyro";
static BNO08x *imu = nullptr;

#if ENABLE_SPIKE_FILTER
// Effective gyro rate from BNO085 is ~100 Hz; 5 s calibration => 500 samples.
static constexpr uint16_t kGyroCalibCap = 512;
static float s_buf_gx[kGyroCalibCap];
static float s_buf_gy[kGyroCalibCap];
static float s_buf_gz[kGyroCalibCap];
static SpikeFilter1D s_filt_gx;
static SpikeFilter1D s_filt_gy;
static SpikeFilter1D s_filt_gz;

static void gyro_filters_begin_()
{
    SpikeFilterConfig c{};
    c.fs             = 100.0f;
    c.full_scale     = 34.9066f;      // BNO085 gyro FS (rad/s)
    c.lsb            = 1.0f / 512.0f; // Q9 quantization
    c.sigma_floor_k  = 3.0f;
    c.spike_z        = 5.0f;
    c.calib_seconds  = 5.0f;
    c.calib_capacity = kGyroCalibCap;

    c.calib_buf = s_buf_gx; s_filt_gx.begin(c);
    c.calib_buf = s_buf_gy; s_filt_gy.begin(c);
    c.calib_buf = s_buf_gz; s_filt_gz.begin(c);
}
#endif // ENABLE_SPIKE_FILTER

extern "C" bool gyro_init(SensorContext_t *ctx) {
    if (ctx == NULL || ctx->hw_config == NULL) return false;

    imu = (BNO08x *)ctx->hw_config;
    imu->rpt.uncal_gyro.enable(1000UL);

    ESP_LOGI(TAG, "Uncalibrated Gyroscope enabled");

#if ENABLE_SPIKE_FILTER
    gyro_filters_begin_();
    ESP_LOGI(TAG, "Spike filter armed (keep device still for 5 s)");
#endif

    return true;
}

extern "C" bool gyro_read_sample(SensorContext_t *ctx, float *data_out) {
    if (data_out == NULL || imu == nullptr) return false;

    if (!imu->rpt.uncal_gyro.has_new_data()) {
        return false;
    }

    bno08x_gyro_t d = imu->rpt.uncal_gyro.get_vel();
    data_out[0] = d.x;
    data_out[1] = d.y;
    data_out[2] = d.z;

#if ENABLE_SPIKE_FILTER
    data_out[0] = s_filt_gx.update(data_out[0]);
    data_out[1] = s_filt_gy.update(data_out[1]);
    data_out[2] = s_filt_gz.update(data_out[2]);
    s_filt_gx.debug_log_periodic("gyro-x");
#endif

    return true;
}
