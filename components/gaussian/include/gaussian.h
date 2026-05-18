/**
 * gaussian.h
 */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

float rand_gaussian(void);
bool noise_injection_is_enabled(void);
void noise_injection_set_enabled(bool enabled);

#ifdef __cplusplus
}
#endif