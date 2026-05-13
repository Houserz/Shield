#include "sensor_hal.h"
#include "spike_filter.h"
#include "BNO08x.hpp"
#include "esp_log.h"

static const char *TAG = "bno085_gyro";
static BNO08x *imu = nullptr;

static constexpr uint32_t kGyroReportIntervalUs = 10000UL; // BNO085 effective limit is ~100 Hz

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
    imu->rpt.uncal_gyro.enable(kGyroReportIntervalUs);

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

    return true;
}

extern "C" bool gyro_process_sample(SensorContext_t *ctx, const float *raw_in, float *processed_out, uint8_t *flags_out) {
    (void)ctx;
    if (raw_in == NULL || processed_out == NULL || flags_out == NULL) return false;

    processed_out[0] = raw_in[0];
    processed_out[1] = raw_in[1];
    processed_out[2] = raw_in[2];
    *flags_out = DATA_FLAG_PROCESSED_SAME_AS_RAW;

#if ENABLE_SPIKE_FILTER
    bool sx = false;
    bool sy = false;
    bool sz = false;
    processed_out[0] = s_filt_gx.update(raw_in[0], &sx);
    processed_out[1] = s_filt_gy.update(raw_in[1], &sy);
    processed_out[2] = s_filt_gz.update(raw_in[2], &sz);
    *flags_out = DATA_FLAG_FILTER_ACTIVE;
    if (sx || sy || sz) {
        *flags_out |= DATA_FLAG_FILTER_SPIKE;
    }
    if (processed_out[0] == raw_in[0] && processed_out[1] == raw_in[1] && processed_out[2] == raw_in[2]) {
        *flags_out |= DATA_FLAG_PROCESSED_SAME_AS_RAW;
    }
    s_filt_gx.debug_log_periodic("gyro-x");
#endif

    return true;
}
