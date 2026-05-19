/**
 * gaussian.cpp
 * Wraps existing cpp random functionality to generate random gaussian numbers
 * between [-1, 1] for use in fault injection.
 */
#include <algorithm>
#include <cmath>
#include "esp_random.h"

// Global runtime flag for noise injection control (default: disabled)
static volatile bool g_noise_injection_enabled = false;

// Box-Muller transform using the ESP32-S3 hardware TRNG (esp_random).
// No large mt19937 state needed — avoids blowing the Tmr Svc task stack.
extern "C" float rand_gaussian(void) {
  static bool has_spare = false;
  static float spare;

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
  spare = v * mul * 0.4f;
  has_spare = true;
  return std::clamp(u * mul * 0.4f, -1.0f, 1.0f);
}

extern "C" bool noise_injection_is_enabled(void) {
  return g_noise_injection_enabled;
}

extern "C" void noise_injection_set_enabled(bool enabled) {
  g_noise_injection_enabled = enabled;
}