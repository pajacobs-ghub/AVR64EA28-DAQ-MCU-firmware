// timerA-free-run.c
// PJ, 2023-11-19, Following Section 22 in data sheet.

#include <xc.h>
#include <stdint.h>
#include "global_defs.h"
#include "timerA-free-run.h"

void timerA0_init(uint16_t ticks)
{
    // With a 20MHz main clock, a prescale of DIV16 gives 0.8 microsecond ticks.
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV16_gc | TCA_SINGLE_ENABLE_bm;
    TCA0.SINGLE.PER = ticks-1;
}

void timerA0_close(void)
{
    TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm;
    TCA0.SINGLE.INTFLAGS |= TCA_SINGLE_OVF_bm;
}

void timerA0_wait(void)
{
    // Wait for overflow.
    while (!(TCA0.SINGLE.INTFLAGS & TCA_SINGLE_OVF_bm)) { /* __builtin_avr_wdr(); */ }
    // We reset the flag but leave the timer ticking
    // so that we have accurate periods.
    TCA0.SINGLE.INTFLAGS |= TCA_SINGLE_OVF_bm;
}
