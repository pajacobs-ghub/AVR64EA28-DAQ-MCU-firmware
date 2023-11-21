// timerA-free-run.h
// PJ, 2023-11-19.
//
#ifndef MY_TIMERA_FREE_RUN
#define MY_TIMERA_FREE_RUN

#include <xc.h>
#include <stdint.h>

void timerA0_init(uint16_t period_count);
void timerA0_close(void);
void timerA0_wait(void);
#endif
