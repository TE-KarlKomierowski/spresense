#pragma once
#include <stdint.h>
/* Angle in milli degrees*/
typedef struct {
	int64_t x;
	int64_t y;
	int64_t z;
} abs_angle_t;

#define FACTOR_LARGE 1000000
