// daq_mcu.c
// Use an AVR64EA28-I/SP as the DAQ-MCU in the two-processor DAQ node.
// Look at the notes in the USQ edaqs workbook pages 47 ff November 2023.
//
// PJ 2023-11-21 First cut adapted from the peripheral demo codes.

// This version string will be printed shortly after MCU reset.
#define VERSION_STR "v0.1 2023-11-21"

#include "global_defs.h"
#include <xc.h>
#include <avr/cpufunc.h>
#include <util/delay.h>
#include "usart.h"
#include "timerA-free-run.h"
#include "stdint.h"
#include <string.h>
#include <stdio.h>

// Do not put FUSE bits setting here, in an effort to avoid changing them.

#define NOUTBUF 64
char output_buf[NOUTBUF];

int main(void)
{
    int n;
    int count = 0;
    int16_t res0, res1;
    
    // Turn off the main clock prescaler so that we run at full 20MHz.
    ccp_write_io((void *) & (CLKCTRL.MCLKCTRLB), (CLKCTRL.MCLKCTRLB & 0xfe));
    CLKCTRL.MCLKTIMEBASE = TIMEBASE_VALUE; // Needed for the ADC
    
    PORTA.DIRSET = PIN7_bm; // Set PA7 to output for SPI CS_A#
    PORTA.OUTSET = PIN7_bm; // CS_A# high
    
    PORTF.DIRSET = PIN0_bm; // Use PF0 to indicate the time for ADC conversion.
    PORTF.OUTCLR = PIN0_bm;
    PORTC.DIRCLR = PIN0_bm; // Input for AIN28
    PORTC.DIRCLR = PIN1_bm; // Input for AIN29
    PORTC.DIRCLR = PIN2_bm; // Input for AIN30
    PORTC.DIRCLR = PIN3_bm; // Input for AIN31
    _delay_ms(10); // Let the pins settle, to reduce garbage on the RX pin.
    usart0_init(115200);
    
    PORTF.OUTSET = PIN0_bm; // Flash PF0 to indicate that we have started.
    _delay_ms(500);
    PORTF.OUTCLR = PIN0_bm;
    n = snprintf(output_buf, NOUTBUF, "\r\nAVR64EA28 DAQ-MCU\r\n%s", VERSION_STR);
    usart0_putstr(output_buf);
    
    // Set up the ADC
    ADC0.CTRLA |= ADC_LOWLAT_bm | ADC_ENABLE_bm;
    ADC0.CTRLB = ADC_PRESC_DIV4_gc; // ADC clock frequency 5MHz
    ADC0.CTRLC = ADC_REFSEL_1V024_gc;
    ADC0.CTRLE = 20; // SAMPDUR of 4 microseconds
    ADC0.CTRLF |= ADC_SAMPNUM_NONE_gc;
    ADC0.PGACTRL = ADC_GAIN_8X_gc | ADC_PGABIASSEL_100PCT_gc | ADC_PGAEN_bm;
    while (ADC0.STATUS & ADC_ADCBUSY_bm) { /* wait for settling */ }
    
    timerA0_init(9765); // 9765*51.2us = 500ms
    timerA0_wait();

    while (1) {
        count++;
        // Select ADC channel and make the conversion.
        PORTF.OUTSET = PIN0_bm;
        ADC0.MUXPOS = ADC_VIA_PGA_gc | ADC_MUXPOS_AIN28_gc; // PC0
        ADC0.MUXNEG = ADC_VIA_PGA_gc | ADC_MUXPOS_AIN29_gc; // PC1
        ADC0.COMMAND = ADC_DIFF_bm | ADC_MODE_SINGLE_12BIT_gc | ADC_START_IMMEDIATE_gc;
        while (!(ADC0.INTFLAGS & ADC_SAMPRDY_bm)) { /* wait */ }
        res0 = ADC0.SAMPLE; // 16-bit value
        ADC0.MUXPOS = ADC_VIA_PGA_gc | ADC_MUXPOS_AIN30_gc; // PC2
        ADC0.MUXNEG = ADC_VIA_PGA_gc | ADC_MUXPOS_AIN31_gc; // PC3
        ADC0.COMMAND = ADC_DIFF_bm | ADC_MODE_SINGLE_12BIT_gc | ADC_START_IMMEDIATE_gc;
        while (!(ADC0.INTFLAGS & ADC_SAMPRDY_bm)) { /* wait */ }
        res1 = ADC0.SAMPLE; // 16-bit value
        PORTF.OUTCLR = PIN0_bm;
        //
        n = snprintf(output_buf, NOUTBUF, "\r\ncount=%d res0=%d res1=%d",
                     count, res0, res1);
        usart0_putstr(output_buf);
        timerA0_wait();
    }
    
    timerA0_close();
    usart0_close();
    return 0;
}
