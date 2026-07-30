/**
 * gaussian.h
 * Gaussian noise generation with per-sensor tracking for dataset logging.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Maximum number of sensor IDs tracked (IDs 1-9 used by SHIELD)
#define GAUSSIAN_MAX_SENSORS 16

/**
 * @brief Generate a Gaussian random value in [-1, 1].
 *        Uses Box-Muller transform with ESP32-S3 hardware TRNG.
 */
float rand_gaussian(void);

/**
 * @brief Generate a Gaussian random value AND store it in the per-sensor
 *        noise register so it can be retrieved by the acquisition task.
 *
 * @param sensor_id  Sensor ID (1-9, matches SensorContext_t.id)
 * @return           The same noise value that was applied
 */
float rand_gaussian_tracked(uint8_t sensor_id);

/**
 * @brief Retrieve the last noise value stored for a given sensor.
 *        Called by the acquisition task after read_sample() returns.
 *        Clears the stored value to 0.0f after reading.
 *
 * @param sensor_id   Sensor ID
 * @param noise_out   Output: noise value that was last applied
 * @return            true if noise injection was active for this call,
 *                    false if injection was disabled (noise_out = 0.0f)
 */
bool gaussian_get_last_noise(uint8_t sensor_id, float *noise_out);

/**
 * @brief Retrieve up to three per-axis noise values stored for a given sensor.
 *        Used by 3-axis sensors (accel/gyro/mag) which call
 *        rand_gaussian_tracked() three times per sample. Scalar sensors
 *        populate noise_out[0] only; noise_out[1..2] = 0.
 *        Clears the stored values after reading.
 *
 * @param sensor_id   Sensor ID
 * @param noise_out   Output array [3]: axis noise values applied
 * @return            true if noise injection was active for this sample,
 *                    false if disabled (noise_out zeroed)
 */
bool gaussian_get_last_noise_3(uint8_t sensor_id, float noise_out[3]);

/**
 * @brief Check whether noise injection is globally enabled.
 */
bool noise_injection_is_enabled(void);

/**
 * @brief Enable or disable noise injection globally.
 */
void noise_injection_set_enabled(bool enabled);

#ifdef __cplusplus
}
#endif
