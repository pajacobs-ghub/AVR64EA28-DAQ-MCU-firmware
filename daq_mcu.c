// daq_mcu.c
// Use an AVR64EA28-I/SP as the DAQ-MCU in the two-processor DAQ node.
// Look at the notes in the USQ edaqs workbook pages 47 ff November 2023.
//
// PJ
// 2023-11-21 First cut adapted from the peripheral demo codes.
// 2023-11-21 Add the basic command interpreter and let it run the show.

// This version string will be printed shortly after MCU reset.
#define VERSION_STR "v0.1 2023-11-21"

#include "global_defs.h"
#include <xc.h>
#include <avr/cpufunc.h>
#include <util/delay.h>
#include "usart.h"
#include "timerA-free-run.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// Do not put FUSE bits setting here, in an effort to avoid changing them.

// String buffer for assembling text for output.
#define NSTRBUF 128
char str_buf[NSTRBUF];
// Text buffer for incoming commands.
// They are expected to be short.
// Be careful, overruns are not handled well.
#define NCMDBUF 64
char cmd_buf[NCMDBUF];

// Parameters controlling the device are stored in virtual registers.
#define NUMREG 4
int16_t vregister[NUMREG]; // working copy in SRAM

void set_registers_to_original_values()
{
    vregister[0] = 1;   // trigger mode
    vregister[1] = 100; // trigger level 1 as a 11-bit count, 0-2048
    vregister[2] = 0;   // sample period in timer ticks 
    vregister[3] = 0;   // number of channels to sample
    // [TODO] up to 12 registers to set MUXPOS and MUXNEG values
}

// [TODO] EEPROM data should start off like the original values above.

char save_registers_to_EEPROM()
{
    return 1; // [TODO] HEFLASH_writeBlock(0, (void*)vregister, NUMREG*2);
}

char restore_registers_from_EEPROM()
{
    return 1; // [TODO] HEFLASH_readBlock((void*)vregister, 0, NUMREG*2);
}

void iopins_init(void)
{
    PORTA.DIRSET = PIN7_bm; // Set PA7 to output for SPI CS_A#
    PORTA.OUTSET = PIN7_bm; // CS_A# high
    
    PORTF.DIRSET = PIN0_bm; // Use PF0 to indicate the time for ADC conversion.
    PORTF.OUTCLR = PIN0_bm;
    PORTC.DIRCLR = PIN0_bm; // Input for AIN28
    PORTC.DIRCLR = PIN1_bm; // Input for AIN29
    PORTC.DIRCLR = PIN2_bm; // Input for AIN30
    PORTC.DIRCLR = PIN3_bm; // Input for AIN31
} // end iopins_init()


void adc0_init(void)
{
    // Set up the ADC
    ADC0.CTRLA |= ADC_LOWLAT_bm | ADC_ENABLE_bm;
    ADC0.CTRLB = ADC_PRESC_DIV4_gc; // ADC clock frequency 5MHz
    ADC0.CTRLC = ADC_REFSEL_1V024_gc;
    ADC0.CTRLE = 20; // SAMPDUR of 4 microseconds
    ADC0.CTRLF |= ADC_SAMPNUM_NONE_gc;
    ADC0.PGACTRL = ADC_GAIN_8X_gc | ADC_PGABIASSEL_100PCT_gc | ADC_PGAEN_bm;
    while (ADC0.STATUS & ADC_ADCBUSY_bm) { /* wait for settling */ }
} // end adc0_init()


void sample_channels(void)
// As a demonstration, sample the analog channels periodically.
{
    int count = 0;
    int16_t res0, res1;
    int nchar;
    //
    adc0_init();
    timerA0_init(9765); // 9765*51.2us = 500ms
    timerA0_wait();
    while (count < 10) {
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
        nchar = snprintf(str_buf, NSTRBUF, "\r\ncount=%d res0=%d res1=%d",
                         count, res0, res1);
        usart0_putstr(str_buf);
        timerA0_wait();
    } 
    timerA0_close();
} // end void sample_channels()


void interpret_command()
{
    char* token_ptr;
    const char * sep_tok = ", ";
    int nchar;
    uint8_t i;
    int16_t v;
    // nchar = snprintf(str_buf, NSTRBUF, "\rCommand text was: ");
    // usart0_putstr(str_buf); usart0_putstr(cmd_buf);
    // nchar = snprintf(str_buf, NSTRBUF, "\r\nNumber of characters in buffer: %u", strlen(cmd_buf));
    // usart0_putstr(str_buf); 
    switch (cmd_buf[0]) {
        case 'v':
            nchar = snprintf(str_buf, NSTRBUF, "%s ok", VERSION_STR);
            usart0_putstr(str_buf);
            break;
        case 'n':
            nchar = snprintf(str_buf, NSTRBUF, "%u ok", NUMREG);
            usart0_putstr(str_buf);
            break;
        case 'p':
            nchar = snprintf(str_buf, NSTRBUF, "\r\nRegister values:");
            usart0_putstr(str_buf);
            for (i=0; i < NUMREG; ++i) {
                nchar = snprintf(str_buf, NSTRBUF, "\r\nreg[%d]=%d", i, vregister[i]);
                usart0_putstr(str_buf);
            }
            nchar = snprintf(str_buf, NSTRBUF, "\r\nok");
            usart0_putstr(str_buf);
            break;
        case 'r':
            // Report a register value.
            token_ptr = strtok(&cmd_buf[1], sep_tok);
            if (token_ptr) {
                // Found some nonblank text, assume register number.
                i = (uint8_t) atoi(token_ptr);
                if (i < NUMREG) {
                    v = vregister[i];
                    nchar = snprintf(str_buf, NSTRBUF, "%d ok", v);
                } else {
                    nchar = snprintf(str_buf, NSTRBUF, "fail");
                }
            } else {
                nchar = snprintf(str_buf, NSTRBUF, "fail");
            }
            usart0_putstr(str_buf);
            break;
        case 's':
            // Set a register value.
            token_ptr = strtok(&cmd_buf[1], sep_tok);
            if (token_ptr) {
                // Found some nonblank text; assume register number.
                // printf("text:\"%s\"", token_ptr);
                i = (uint8_t) atoi(token_ptr);
                if (i < NUMREG) {
                    token_ptr = strtok(NULL, sep_tok);
                    if (token_ptr) {
                        // Assume text is value for register.
                        v = (int16_t) atoi(token_ptr);
                        vregister[i] = v;
                        nchar = snprintf(str_buf, NSTRBUF, "reg[%u] %d ok", i, v);
                    } else {
                        nchar = snprintf(str_buf, NSTRBUF, "fail");
                    }
                } else {
                    nchar = snprintf(str_buf, NSTRBUF, "fail");
                }
            } else {
                nchar = snprintf(str_buf, NSTRBUF, "fail");
            }
            usart0_putstr(str_buf);
            break;
        case 'R':
            if (restore_registers_from_EEPROM()) {
                nchar = snprintf(str_buf, NSTRBUF, "fail");
            } else {
                nchar = snprintf(str_buf, NSTRBUF, "ok");
            }
            usart0_putstr(str_buf);
            break;
        case 'S':
            if (save_registers_to_EEPROM()) {
                nchar = snprintf(str_buf, NSTRBUF, "fail");
            } else {
                nchar = snprintf(str_buf, NSTRBUF, "ok");
            }
            usart0_putstr(str_buf);
            break;
        case 'F':
            set_registers_to_original_values();
            nchar = snprintf(str_buf, NSTRBUF, "ok");
            usart0_putstr(str_buf);
            break;
        case 'g':
            sample_channels();
            break;
        case 'c':
            // Report an ADC value.
            token_ptr = strtok(&cmd_buf[1], sep_tok);
            if (token_ptr) {
                // Found some nonblank text, assume channel number.
                i = (uint8_t) atoi(token_ptr);
                if (i <= 12) {
                    // [TODO] v = read_adc(i);
                    v = 0;
                    nchar = snprintf(str_buf, NSTRBUF, "%d ok", v);
                } else {
                    nchar = snprintf(str_buf, NSTRBUF, "fail");
                }
            } else {
                nchar = snprintf(str_buf, NSTRBUF, "fail");
            }
            usart0_putstr(str_buf);
            break;
        case 'h':
        case '?':
            nchar = snprintf(str_buf, NSTRBUF, "\r\nAVR64EA28 DAQ-MCU commands and registers"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\nCommands:"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n h or ? print this help message"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n v      report version of firmware"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n n      report number of registers"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n p      report register values"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n r <i>  report value of register i"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n s <i> <j>  set register i to value j"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n R      restore register values from HEFlash"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n S      save register values to HEFlash"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n F      set register values to original values"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n g      go and start sampling"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n c <i>  convert analogue channel i"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\nRegisters:"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 0  mode: 1= sample channels and report"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n          2= sample channels, recording data"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 1  trigger level a as a 11-bit count, 0-2047"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 2  sample period in timer ticks"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 3  number of channels to sample"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\nok"); usart0_putstr(str_buf);
            break;
        default:
            nchar = snprintf(str_buf, NSTRBUF, "fail"); usart0_putstr(str_buf);
    } // end switch
} // end interpret_command())


int main(void)
{
    int ncmd, nchar;
    //
    // Turn off the main clock prescaler so that we run at full 20MHz.
    ccp_write_io((void *) & (CLKCTRL.MCLKCTRLB), (CLKCTRL.MCLKCTRLB & 0xfe));
    CLKCTRL.MCLKTIMEBASE = TIMEBASE_VALUE; // Needed for the ADC
    iopins_init();
    _delay_ms(10); // Let the pins settle, to reduce garbage on the RX pin.
    usart0_init(115200);
    //
    nchar = snprintf(str_buf, NSTRBUF, "\r\nAVR64EA28 DAQ-MCU\r\n%s", VERSION_STR);
    usart0_putstr(str_buf);
    //
    // The basic behaviour is to be forever checking for a text command.
    nchar = snprintf(str_buf, NSTRBUF, "\r\ncmd> ");
    usart0_putstr(str_buf);
    while (1) {
        // Characters are echoed as they are typed.
        // Backspace deleting is allowed.
        ncmd = usart0_getstr(cmd_buf, NCMDBUF); 
        if (ncmd) {
            interpret_command();
            nchar = snprintf(str_buf, NSTRBUF, "\r\ncmd> ");
            usart0_putstr(str_buf);
        }
    } // never-ending while
    usart0_close();
    return 0;
}
