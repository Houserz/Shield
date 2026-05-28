#include "signal_processing.h"

#include <math.h>
#include <stdint.h>
#include <string.h>
#include "esp_random.h"

static float clampf_(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

float signal_rand_gaussian(void)
{
    static bool has_spare = false;
    static float spare = 0.0f;

    if (has_spare) {
        has_spare = false;
        return clampf_(spare, -1.0f, 1.0f);
    }

    float u = 0.0f;
    float v = 0.0f;
    float s = 0.0f;
    do {
        u = ((float)esp_random() / (float)UINT32_MAX) * 2.0f - 1.0f;
        v = ((float)esp_random() / (float)UINT32_MAX) * 2.0f - 1.0f;
        s = u * u + v * v;
    } while (s >= 1.0f || s == 0.0f);

    float mul = sqrtf(-2.0f * logf(s) / s) * SIGNAL_NOISE_SIGMA;
    spare = v * mul;
    has_spare = true;
    return clampf_(u * mul, -1.0f, 1.0f);
}

float signal_apply_multiplicative_noise(float clean_value)
{
    return clean_value + clean_value * signal_rand_gaussian();
}

void signal_make_noisy(const float *clean, float *noisy, uint8_t axis_count, bool enabled)
{
    if (!clean || !noisy) return;
    if (axis_count == 0) axis_count = 1;
    if (axis_count > 3) axis_count = 3;

    for (uint8_t i = 0; i < 3; i++) {
        if (i >= axis_count) {
            noisy[i] = 0.0f;
        } else if (enabled) {
            noisy[i] = signal_apply_multiplicative_noise(clean[i]);
        } else {
            noisy[i] = clean[i];
        }
    }
}

void signal_lowpass_reset(lowpass_filter_t *filter)
{
    if (!filter) return;
    memset(filter, 0, sizeof(*filter));
}

void signal_lowpass_update(lowpass_filter_t *filter,
                           const float *input,
                           float *output,
                           uint8_t axis_count,
                           uint32_t timestamp_ms,
                           float cutoff_hz)
{
    if (!filter || !input || !output) return;
    if (axis_count == 0) axis_count = 1;
    if (axis_count > 3) axis_count = 3;

    if (!filter->initialized || cutoff_hz <= 0.0f) {
        for (uint8_t i = 0; i < 3; i++) {
            filter->y[i] = (i < axis_count) ? input[i] : 0.0f;
            output[i] = filter->y[i];
        }
        filter->last_timestamp_ms = timestamp_ms;
        filter->initialized = true;
        return;
    }

    uint32_t dt_ms = timestamp_ms - filter->last_timestamp_ms;
    filter->last_timestamp_ms = timestamp_ms;
    if (dt_ms == 0) {
        dt_ms = 1;
    }

    float dt_s = (float)dt_ms / 1000.0f;
    if (dt_s > 1.0f) {
        dt_s = 1.0f;
    }

    const float pi = 3.14159265358979323846f;
    float tau = 1.0f / (2.0f * pi * cutoff_hz);
    float alpha = dt_s / (tau + dt_s);
    alpha = clampf_(alpha, 0.0f, 1.0f);

    for (uint8_t i = 0; i < 3; i++) {
        if (i < axis_count) {
            filter->y[i] += alpha * (input[i] - filter->y[i]);
            output[i] = filter->y[i];
        } else {
            filter->y[i] = 0.0f;
            output[i] = 0.0f;
        }
    }
}
