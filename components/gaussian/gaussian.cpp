/**
 * gaussian.cpp
 * Wraps existing cpp random functionality to generate random gaussian numbers
 * between [-1, 1] for use in fault injection.
 */
#include <algorithm>
#include <random>

extern "C" float rand_gaussian(void) {
  static thread_local std::mt19937 rng{std::random_device{}()};
  static thread_local std::normal_distribution<float> dist(0.0f, 0.4f);
  return std::clamp(dist(rng), -1.0f, 1.0f);
}