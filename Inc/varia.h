
#ifndef VARIA_H_
#define VARIA_H_

#include "main.h"
#include "config.h"
#include "stdint.h"
#include <arm_math.h>

int16_t q31_degree_to_degree(q31_t q31_degree);

int32_t map(int32_t x, int32_t x_min, int32_t x_max, int32_t y_min, int32_t y_max);

uint32_t limit_uint32(uint32_t min, uint32_t max, uint32_t value);
int32_t limit_int32(int32_t min, int32_t max, int32_t value);

uint32_t isqrt(uint32_t x);

#endif // VARIA_H_