#ifndef SIGNAL_PROCESSING_H
#define SIGNAL_PROCESSING_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SIGNAL_NOISE_SIGMA 0.4f

typedef struct {
    float y[3];
    uint32_t last_timestamp_ms;
    bool initialized;
} lowpass_filter_t;

float signal_rand_gaussian(void);
float signal_apply_multiplicative_noise(float clean_value);
void signal_make_noisy(const float *clean, float *noisy, uint8_t axis_count, bool enabled);
void signal_lowpass_reset(lowpass_filter_t *filter);
void signal_lowpass_update(lowpass_filter_t *filter,
                           const float *input,
                           float *output,
                           uint8_t axis_count,
                           uint32_t timestamp_ms,
                           float cutoff_hz);

#ifdef __cplusplus
}
#endif

#endif // SIGNAL_PROCESSING_H
