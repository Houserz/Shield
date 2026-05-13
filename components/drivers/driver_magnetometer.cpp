#include "sensor_hal.h"
#include "spike_filter.h"
#include "BNO08x.hpp"
#include "esp_log.h"

static const char *TAG = "bno085_mag";
static BNO08x *imu = nullptr;

static constexpr uint32_t kMagReportIntervalUs = 10000UL; // BNO085 effective limit is ~100 Hz

#if ENABLE_SPIKE_FILTER
// Effective mag rate from BNO085 is ~100 Hz; 5 s calibration => 500 samples.
static constexpr uint16_t kMagCalibCap = 512;
static float s_buf_mx[kMagCalibCap];
static float s_buf_my[kMagCalibCap];
static float s_buf_mz[kMagCalibCap];
static SpikeFilter1D s_filt_mx;
static SpikeFilter1D s_filt_my;
static SpikeFilter1D s_filt_mz;

static void mag_filters_begin_()
{
    SpikeFilterConfig c{};
    c.fs             = 100.0f;
    c.full_scale     = 1300.0f;      // BNO085 mag FS (uT)
    c.lsb            = 1.0f / 16.0f; // Q4 quantization
    c.sigma_floor_k  = 3.0f;
    c.spike_z        = 5.0f;
    c.calib_seconds  = 5.0f;
    c.calib_capacity = kMagCalibCap;

    c.calib_buf = s_buf_mx; s_filt_mx.begin(c);
    c.calib_buf = s_buf_my; s_filt_my.begin(c);
    c.calib_buf = s_buf_mz; s_filt_mz.begin(c);
}
#endif // ENABLE_SPIKE_FILTER

extern "C" bool mag_init(SensorContext_t *ctx) {
    if (ctx == NULL || ctx->hw_config == NULL) return false;

    imu = (BNO08x *)ctx->hw_config;
    imu->rpt.uncal_magnetometer.enable(kMagReportIntervalUs);

    ESP_LOGI(TAG, "Uncalibrated Magnetometer enabled");

#if ENABLE_SPIKE_FILTER
    mag_filters_begin_();
    ESP_LOGI(TAG, "Spike filter armed (keep device still for 5 s)");
#endif

    return true;
}

extern "C" bool mag_read_sample(SensorContext_t *ctx, float *data_out) {
    if (data_out == NULL || imu == nullptr) return false;

    if (!imu->rpt.uncal_magnetometer.has_new_data()) {
        return false;
    }

    bno08x_magf_t d = imu->rpt.uncal_magnetometer.get_magf();
    data_out[0] = d.x;
    data_out[1] = d.y;
    data_out[2] = d.z;

    return true;
}

extern "C" bool mag_process_sample(SensorContext_t *ctx, const float *raw_in, float *processed_out, uint8_t *flags_out) {
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
    processed_out[0] = s_filt_mx.update(raw_in[0], &sx);
    processed_out[1] = s_filt_my.update(raw_in[1], &sy);
    processed_out[2] = s_filt_mz.update(raw_in[2], &sz);
    *flags_out = DATA_FLAG_FILTER_ACTIVE;
    if (sx || sy || sz) {
        *flags_out |= DATA_FLAG_FILTER_SPIKE;
    }
    if (processed_out[0] == raw_in[0] && processed_out[1] == raw_in[1] && processed_out[2] == raw_in[2]) {
        *flags_out |= DATA_FLAG_PROCESSED_SAME_AS_RAW;
    }
    s_filt_mx.debug_log_periodic("mag-x");
#endif

    return true;
}
