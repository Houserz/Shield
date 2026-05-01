/**
 * @file spike_filter.h
 * @brief Real-time, single-pass spike removal for BNO085 channels.
 *
 * Per-channel pipeline:
 *   1. First N = fs * calib_seconds samples are accumulated for self-calibration
 *      (device must be stationary). bias = median(samples), sigma_ref =
 *      max(1.4826 * MAD, sigma_floor_k * lsb).
 *   2. After calibration, bias and sigma_ref are locked.
 *   3. Each new sample is checked by:
 *        (a) full-scale gate:  |v| > full_scale  -> spike
 *        (b) first-difference z-score:
 *              |v - prev_clean| > spike_z * sqrt(2) * sigma_ref  -> spike
 *      and repaired by zero-order hold (v_clean = prev_clean).
 *
 * Memory model:
 *   - No dynamic allocation. The caller passes a statically allocated
 *     calibration buffer of size >= fs * calib_seconds via cfg.calib_buf.
 *   - The buffer is only used during calibration (re-used in place to compute
 *     MAD), so it can technically be shared across instances if calibrations
 *     never overlap in time. In this project we give each instance its own.
 */
#ifndef SPIKE_FILTER_H
#define SPIKE_FILTER_H

#ifndef ENABLE_SPIKE_FILTER
#define ENABLE_SPIKE_FILTER 1
#endif

// Optional Hampel-window backstop. Default off; flip to 1 to enable.
#ifndef SPIKE_FILTER_ENABLE_HAMPEL
#define SPIKE_FILTER_ENABLE_HAMPEL 0
#endif

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus

struct SpikeFilterConfig {
    float    fs              = 0.0f;        // expected sample rate (Hz)
    float    full_scale      = 0.0f;        // FS in SI units (positive)
    float    lsb             = 0.0f;        // 1 LSB in SI units
    float    sigma_floor_k   = 3.0f;        // sigma_ref >= sigma_floor_k * lsb
    float    spike_z         = 5.0f;        // z-score threshold
    float    calib_seconds   = 5.0f;        // length of stationary self-cal window
    float*   calib_buf       = nullptr;     // caller-owned, capacity floats
    uint16_t calib_capacity  = 0;           // must be >= ceil(fs * calib_seconds)
};

class SpikeFilter1D {
public:
    void  begin(const SpikeFilterConfig& cfg);

    // Feed one new raw sample. Returns the cleaned sample.
    // Optional out flag is true iff this sample was classified as a spike.
    float update(float v_raw, bool* is_spike_out = nullptr);

    bool     is_calibrated() const { return calibrated_; }
    float    bias()          const { return bias_; }
    float    sigma_ref()     const { return sigma_ref_; }
    uint32_t spikes_total()  const { return spikes_total_; }
    uint32_t samples_total() const { return samples_total_; }

    // Throttled (~1 Hz) ESP_LOGI line. Safe to call every update.
    // Pass a stable identifier such as "accel-x".
    void debug_log_periodic(const char* channel_name);

private:
    void finalize_calibration_();

    SpikeFilterConfig cfg_{};
    float*   buf_           = nullptr;
    uint16_t cap_           = 0;
    uint16_t target_        = 0;
    uint16_t count_         = 0;
    bool     calibrated_    = false;
    float    bias_          = 0.0f;
    float    sigma_ref_     = 0.0f;
    float    diff_thresh_   = 0.0f;
    float    prev_clean_    = 0.0f;
    bool     has_prev_      = false;
    uint32_t spikes_total_  = 0;
    uint32_t samples_total_ = 0;
    uint64_t last_log_us_   = 0;

#if SPIKE_FILTER_ENABLE_HAMPEL
    // Reserved for the optional short-window Hampel backstop (W = 21).
    static constexpr uint16_t kHampelW = 21;
    float    hampel_ring_[kHampelW] = {};
    float    hampel_sorted_[kHampelW] = {};
    uint16_t hampel_count_ = 0;
    uint16_t hampel_head_  = 0;
#endif
};

#endif // __cplusplus
#endif // SPIKE_FILTER_H
