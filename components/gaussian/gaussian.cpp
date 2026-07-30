/**
 * gaussian.cpp
 * Gaussian noise generation with per-sensor tracking for dataset logging.
 *
 * Two APIs:
 *   rand_gaussian()              — untracked, legacy
 *   rand_gaussian_tracked(id)    — stores applied noise value per sensor_id
 *                                  so the acquisition task can log clean+noise
 *
 * The per-sensor register holds up to 3 axis values: 3-axis sensors
 * (accel/gyro/mag) call rand_gaussian_tracked() three times per sample, once
 * per axis; scalar sensors call it once and use index 0 only.
 */
#include <algorithm>
#include <cmath>
#include <cstring>
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "gaussian.h"

// ---------------------------------------------------------------------------
// Global noise injection enable flag
// ---------------------------------------------------------------------------
static volatile bool g_noise_injection_enabled = false;

// ---------------------------------------------------------------------------
// Per-sensor noise register
// Indexed by sensor_id (0 unused; valid IDs 1-15). Each sensor stores up to 3
// axis noise values, with s_noise_count tracking how many
// rand_gaussian_tracked() calls occurred for the current sample.
// Written by rand_gaussian_tracked(), read+cleared by the gaussian_get_last_*
// retrieval functions. Access is interrupt-safe via portENTER_CRITICAL because
// drivers may run from different tasks on different cores.
// ---------------------------------------------------------------------------
static float   s_last_noise[GAUSSIAN_MAX_SENSORS][3] = {{0.0f}};
static uint8_t s_noise_count[GAUSSIAN_MAX_SENSORS]   = {0};
static portMUX_TYPE s_noise_mux = portMUX_INITIALIZER_UNLOCKED;

// ---------------------------------------------------------------------------
// Core Box-Muller generator (shared by both public APIs)
// ---------------------------------------------------------------------------
static float generate_gaussian(void) {
    static bool  has_spare = false;
    static float spare     = 0.0f;

    if (has_spare) {
        has_spare = false;
        return std::clamp(spare, -1.0f, 1.0f);
    }

    float u, v, s;
    do {
        u = (float)esp_random() / (float)UINT32_MAX * 2.0f - 1.0f;
        v = (float)esp_random() / (float)UINT32_MAX * 2.0f - 1.0f;
        s = u * u + v * v;
    } while (s >= 1.0f || s == 0.0f);

    float mul = sqrtf(-2.0f * logf(s) / s);
    spare     = v * mul * 0.4f;
    has_spare = true;
    return std::clamp(u * mul * 0.4f, -1.0f, 1.0f);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

extern "C" float rand_gaussian(void) {
    return generate_gaussian();
}

extern "C" float rand_gaussian_tracked(uint8_t sensor_id) {
    float noise = generate_gaussian();

    if (sensor_id < GAUSSIAN_MAX_SENSORS) {
        portENTER_CRITICAL(&s_noise_mux);
        // Store into the next axis slot for this sample (cap at index 2 so a
        // sensor that somehow calls >3 times overwrites the last axis rather
        // than writing out of bounds).
        uint8_t idx = s_noise_count[sensor_id] < 3 ? s_noise_count[sensor_id] : 2;
        s_last_noise[sensor_id][idx] = noise;
        s_noise_count[sensor_id]++;
        portEXIT_CRITICAL(&s_noise_mux);
    }

    return noise;
}

extern "C" bool gaussian_get_last_noise(uint8_t sensor_id, float *noise_out) {
    if (noise_out == nullptr || sensor_id >= GAUSSIAN_MAX_SENSORS) {
        if (noise_out) *noise_out = 0.0f;
        return false;
    }

    portENTER_CRITICAL(&s_noise_mux);
    uint8_t count = s_noise_count[sensor_id];
    float   value = s_last_noise[sensor_id][0];
    // Clear after read so stale values don't pollute the next sample
    s_last_noise[sensor_id][0] = 0.0f;
    s_last_noise[sensor_id][1] = 0.0f;
    s_last_noise[sensor_id][2] = 0.0f;
    s_noise_count[sensor_id]   = 0;
    portEXIT_CRITICAL(&s_noise_mux);

    *noise_out = (count > 0) ? value : 0.0f;
    return count > 0;
}

extern "C" bool gaussian_get_last_noise_3(uint8_t sensor_id, float noise_out[3]) {
    if (noise_out == nullptr || sensor_id >= GAUSSIAN_MAX_SENSORS) {
        if (noise_out) {
            noise_out[0] = 0.0f;
            noise_out[1] = 0.0f;
            noise_out[2] = 0.0f;
        }
        return false;
    }

    portENTER_CRITICAL(&s_noise_mux);
    uint8_t count = s_noise_count[sensor_id];
    noise_out[0] = s_last_noise[sensor_id][0];
    noise_out[1] = (count > 1) ? s_last_noise[sensor_id][1] : 0.0f;
    noise_out[2] = (count > 2) ? s_last_noise[sensor_id][2] : 0.0f;
    // Clear after read so stale values don't pollute the next sample
    s_last_noise[sensor_id][0] = 0.0f;
    s_last_noise[sensor_id][1] = 0.0f;
    s_last_noise[sensor_id][2] = 0.0f;
    s_noise_count[sensor_id]   = 0;
    portEXIT_CRITICAL(&s_noise_mux);

    return count > 0;
}

extern "C" bool noise_injection_is_enabled(void) {
    return g_noise_injection_enabled;
}

extern "C" void noise_injection_set_enabled(bool enabled) {
    g_noise_injection_enabled = enabled;
}
