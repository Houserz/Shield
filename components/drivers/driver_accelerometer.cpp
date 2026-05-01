#include "sensor_hal.h"
#include "spike_filter.h"
#include "esp_log.h"
#include "BNO08x.hpp"

static const char *TAG = "bno085_accel";
static BNO08x *imu = nullptr;

#if ENABLE_SPIKE_FILTER
// Effective accel rate from BNO085 is ~250 Hz; 5 s calibration => 1250 samples.
// Add headroom so target == fs*calib_seconds always fits.
static constexpr uint16_t kAccelCalibCap = 1280;
static float s_buf_ax[kAccelCalibCap];
static float s_buf_ay[kAccelCalibCap];
static float s_buf_az[kAccelCalibCap];
static SpikeFilter1D s_filt_ax;
static SpikeFilter1D s_filt_ay;
static SpikeFilter1D s_filt_az;

static void accel_filters_begin_()
{
    SpikeFilterConfig c{};
    c.fs             = 250.0f;
    c.full_scale     = 156.9060f;     // BNO085 accel FS (m/s^2)
    c.lsb            = 1.0f / 256.0f; // Q8 quantization
    c.sigma_floor_k  = 3.0f;
    c.spike_z        = 5.0f;
    c.calib_seconds  = 5.0f;
    c.calib_capacity = kAccelCalibCap;

    c.calib_buf = s_buf_ax; s_filt_ax.begin(c);
    c.calib_buf = s_buf_ay; s_filt_ay.begin(c);
    c.calib_buf = s_buf_az; s_filt_az.begin(c);
}
#endif // ENABLE_SPIKE_FILTER

extern "C" bool accel_init(SensorContext_t *ctx) {
    if (ctx == NULL || ctx->hw_config == NULL) {
        ESP_LOGE(TAG, "Invalid context or hw_config");
        return false;
    }

    imu = (BNO08x *)ctx->hw_config;
    imu->rpt.accelerometer.enable(1000UL);
    ESP_LOGI(TAG, "Accelerometer enabled");

#if ENABLE_SPIKE_FILTER
    accel_filters_begin_();
    ESP_LOGI(TAG, "Spike filter armed (keep device still for 5 s)");
#endif

    return true;
}

extern "C" bool accel_read_sample(SensorContext_t *ctx, float *data_out) {
    if (data_out == NULL || imu == nullptr) return false;

    if (!imu->rpt.accelerometer.has_new_data()) {
        return false;
    }

    bno08x_accel_t d = imu->rpt.accelerometer.get();
    data_out[0] = d.x;
    data_out[1] = d.y;
    data_out[2] = d.z;

#if ENABLE_SPIKE_FILTER
    data_out[0] = s_filt_ax.update(data_out[0]);
    data_out[1] = s_filt_ay.update(data_out[1]);
    data_out[2] = s_filt_az.update(data_out[2]);
    s_filt_ax.debug_log_periodic("accel-x");
#endif

    return true;
}
