// global_defs.h
//
#ifndef GLOBAL_DEFS_H
#define GLOBAL_DEFS_H

// After reset with default fuse settings,
// we have a 20MHz oscillator and a prescaler division factor of 6.
// We also have the same frequency for CPU and peripheral clock.
#define F_CPU (20000000UL)
#define F_CLK_PER F_CPU
#include <stdint.h>
#include <math.h>
#define TIMEBASE_VALUE ((uint8_t) ceil(F_CLK_PER*1.0e-6))

#endif
