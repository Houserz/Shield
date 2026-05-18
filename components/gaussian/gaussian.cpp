/**
 * gaussian.cpp
 * Wraps existing cpp random functionality to generate random gaussian numbers
 * between [-1, 1] for use in fault injection.
 */
#include <algorithm>
#include <random>

// Global runtime flag for noise injection control (default: disabled)
static volatile bool g_noise_injection_enabled = false;

extern "C" float rand_gaussian(void) {
  static thread_local std::mt19937 rng{std::random_device{}()};
  static thread_local std::normal_distribution<float> dist(0.0f, 0.4f);
  return std::clamp(dist(rng), -1.0f, 1.0f);
}

extern "C" bool noise_injection_is_enabled(void) {
  return g_noise_injection_enabled;
}

extern "C" void noise_injection_set_enabled(bool enabled) {
  g_noise_injection_enabled = enabled;
}