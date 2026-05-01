#include "spike_filter.h"

#include <algorithm>
#include <cmath>

#include "esp_log.h"
#include "esp_timer.h"

static const char* SPIKE_TAG = "spike";

static inline float quickselect_median_(float* a, uint16_t n)
{
    if (n == 0) return 0.0f;
    const uint16_t mid = n / 2;
    std::nth_element(a, a + mid, a + n);
    float m = a[mid];
    if ((n & 1u) == 0u) {
        // For even n, take avg of mid and max of lower half for stable result.
        float lo_max = *std::max_element(a, a + mid);
        m = 0.5f * (m + lo_max);
    }
    return m;
}

void SpikeFilter1D::begin(const SpikeFilterConfig& cfg)
{
    cfg_           = cfg;
    buf_           = cfg.calib_buf;
    cap_           = cfg.calib_capacity;

    uint32_t target = (uint32_t)(cfg.fs * cfg.calib_seconds + 0.5f);
    if (target > cap_) target = cap_;
    target_        = (uint16_t)target;

    count_         = 0;
    calibrated_    = false;
    bias_          = 0.0f;
    sigma_ref_     = 0.0f;
    diff_thresh_   = 0.0f;
    prev_clean_    = 0.0f;
    has_prev_      = false;
    spikes_total_  = 0;
    samples_total_ = 0;
    last_log_us_   = 0;

#if SPIKE_FILTER_ENABLE_HAMPEL
    hampel_count_ = 0;
    hampel_head_  = 0;
#endif
}

void SpikeFilter1D::finalize_calibration_()
{
    // Defensive: if the buffer somehow wasn't filled, fall back to safe defaults.
    if (count_ == 0 || buf_ == nullptr) {
        bias_        = 0.0f;
        sigma_ref_   = cfg_.sigma_floor_k * cfg_.lsb;
        diff_thresh_ = cfg_.spike_z * 1.41421356f * sigma_ref_;
        calibrated_  = true;
        return;
    }

    float median = quickselect_median_(buf_, count_);

    // Convert buffer in place to |x - median| for MAD.
    for (uint16_t i = 0; i < count_; ++i) {
        buf_[i] = std::fabs(buf_[i] - median);
    }
    float mad = quickselect_median_(buf_, count_);

    float sigma_raw = 1.4826f * mad;
    float sigma_min = cfg_.sigma_floor_k * cfg_.lsb;
    sigma_ref_ = (sigma_raw > sigma_min) ? sigma_raw : sigma_min;

    bias_        = median;
    diff_thresh_ = cfg_.spike_z * 1.41421356f * sigma_ref_;
    calibrated_  = true;

    ESP_LOGI(SPIKE_TAG,
             "calibrated: bias=%.4f sigma_ref=%.6f diff_thresh=%.6f (n=%u, fs=%.1f)",
             bias_, sigma_ref_, diff_thresh_, (unsigned)count_, cfg_.fs);
}

float SpikeFilter1D::update(float v_raw, bool* is_spike_out)
{
    samples_total_++;

    if (!calibrated_) {
        if (count_ < cap_) {
            buf_[count_++] = v_raw;
        }
        if (count_ >= target_) {
            finalize_calibration_();
            // Seed prev_clean_ with bias to avoid a spurious first-diff trip
            // when the first post-calibration sample arrives.
            prev_clean_ = bias_;
            has_prev_   = true;
        }
        if (is_spike_out) *is_spike_out = false;
        return v_raw; // pass-through during calibration
    }

    bool spike = false;

    // (b) Full-scale gate: physically impossible value.
    if (std::fabs(v_raw) > cfg_.full_scale) {
        spike = true;
    } else if (has_prev_) {
        // (a) First-difference z-score.
        float dv = v_raw - prev_clean_;
        if (std::fabs(dv) > diff_thresh_) {
            spike = true;
        }
    }

#if SPIKE_FILTER_ENABLE_HAMPEL
    // (c) Optional short-window Hampel backstop. Insertion-sort streaming median.
    // Currently disabled by default; see SPIKE_FILTER_ENABLE_HAMPEL.
    // ... left as a placeholder for future tuning ...
#endif

    float v_out = spike ? prev_clean_ : v_raw;
    prev_clean_ = v_out;
    has_prev_   = true;
    if (spike) spikes_total_++;
    if (is_spike_out) *is_spike_out = spike;
    return v_out;
}

void SpikeFilter1D::debug_log_periodic(const char* channel_name)
{
    const uint64_t now_us = (uint64_t)esp_timer_get_time();
    if (last_log_us_ == 0) {
        last_log_us_ = now_us;
        return;
    }
    if (now_us - last_log_us_ < 1000000ULL) return;
    last_log_us_ = now_us;

    if (!calibrated_) {
        ESP_LOGI(SPIKE_TAG, "%s: calibrating %u/%u",
                 channel_name, (unsigned)count_, (unsigned)target_);
    } else {
        ESP_LOGI(SPIKE_TAG,
                 "%s: bias=%.4f sigma_ref=%.6f spikes_total=%u samples=%u",
                 channel_name, bias_, sigma_ref_,
                 (unsigned)spikes_total_, (unsigned)samples_total_);
    }
}
