/**
 * gaussian.h
 */
#pragma once

#include <stdbool.h>

#define NOISE_INJECTION true   // set to false to disable noise injection

#ifdef __cplusplus
extern "C" {
#endif

float rand_gaussian(void);

#ifdef __cplusplus
}
#endif