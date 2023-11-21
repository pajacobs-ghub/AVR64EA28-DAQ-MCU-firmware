// daq_mcu.c
// Use an AVR64EA28-I/SP as the DAQ-MCU in the two-processor DAQ node.
// Look at the notes in the USQ edaqs workbook pages 47 ff November 2023.
//
// PJ
// 2023-11-21 First cut adapted from the peripheral demo codes.
// 2023-11-21 Add the basic command interpreter and let it run the show.

// This version string will be printed shortly after MCU reset.
#define VERSION_STR "v0.2 2023-11-21"

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

// Bit patterns for selecting analog-input pins.
const uint8_t muxpos_pin[] = {
    ADC_MUXPOS_AIN28_gc, // [0] = PC0
    ADC_MUXPOS_AIN29_gc, // [1] = PC1
    ADC_MUXPOS_AIN30_gc, // [2] = PC2
    ADC_MUXPOS_AIN31_gc, // [3] = PC3
    ADC_MUXPOS_AIN0_gc,  // [4] = PD0
    ADC_MUXPOS_AIN1_gc,  // [5] = PD1
    ADC_MUXPOS_AIN2_gc,  // [6] = PD2
    ADC_MUXPOS_AIN3_gc,  // [7] = PD3
    ADC_MUXPOS_AIN4_gc,  // [8] = PD4
    ADC_MUXPOS_AIN5_gc,  // [9] = PD5
    ADC_MUXPOS_AIN6_gc,  // [10] = PD6
    ADC_MUXPOS_AIN7_gc,  // [11] = PD7
    ADC_MUXPOS_GND_gc,   // [12] = GND
};
const uint8_t muxneg_pin[] = {
    ADC_MUXNEG_AIN28_gc, // [0] = PC0
    ADC_MUXNEG_AIN29_gc, // [1] = PC1
    ADC_MUXNEG_AIN30_gc, // [2] = PC2
    ADC_MUXNEG_AIN31_gc, // [3] = PC3
    ADC_MUXNEG_AIN0_gc,  // [4] = PD0
    ADC_MUXNEG_AIN1_gc,  // [5] = PD1
    ADC_MUXNEG_AIN2_gc,  // [6] = PD2
    ADC_MUXNEG_AIN3_gc,  // [7] = PD3
    ADC_MUXNEG_AIN4_gc,  // [8] = PD4
    ADC_MUXNEG_AIN5_gc,  // [9] = PD5
    ADC_MUXNEG_AIN6_gc,  // [10] = PD6
    ADC_MUXNEG_AIN7_gc,  // [11] = PD7
    ADC_MUXNEG_GND_gc,   // [12] = GND
};

#define MAXNCHAN 6
#define MAXNSAMP 256
int16_t data[MAXNCHAN][MAXNSAMP];

// Parameters controlling the device are stored in virtual registers.
#define NUMREG 22
int16_t vregister[NUMREG]; // working copy in SRAM

void set_registers_to_original_values()
{
    vregister[0] = 1250; // sample period in timer ticks 
    vregister[1] = 6;    // number of channels to sample
    vregister[2] = 256;  // number of samples in record
    vregister[3] = 1;    // trigger mode 0=immediate, 1=internal, 2=external
    vregister[4] = 0;    // trigger channel for internal trigger
    vregister[5] = 100;  // trigger level as an 11-bit count, 0-2048
    vregister[6] = 128;  // number of samples to collect after trigger event
    vregister[7] = 0;    // PGA flag for all channels, 0=direct 1=via_PGA
    vregister[8] = 0;    // PGA gain 0=8X
    vregister[9] = 0;    // V_REF 0=1.024V
    vregister[10] = muxpos_pin[0];  // CH0+ = PC0 by default
    vregister[11] = muxneg_pin[1];  // CH0- = PC1 by default
    vregister[12] = muxpos_pin[2];  // CH1+ = PC2 by default
    vregister[13] = muxneg_pin[3];  // CH1- = PC3 by default
    vregister[14] = muxpos_pin[4];  // CH2+ = PD0 by default
    vregister[15] = muxneg_pin[5];  // CH2- = PD1 by default
    vregister[16] = muxpos_pin[6];  // CH3+ = PD2 by default
    vregister[17] = muxneg_pin[7];  // CH3- = PD3 by default
    vregister[18] = muxpos_pin[8];  // CH4+ = PD4 by default
    vregister[19] = muxneg_pin[9];  // CH4- = PD5 by default
    vregister[20] = muxpos_pin[10]; // CH5+ = PD6 by default
    vregister[21] = muxneg_pin[11]; // CH5- = PD7 by default
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
// As a demonstration, sample the analog channels periodically
// and report values.
{
    uint16_t ticks = (uint16_t)vregister[0];
    uint8_t n_chan = (uint8_t)vregister[1];
    uint16_t n_sample = (uint16_t)vregister[2];
    int16_t res[MAXNCHAN];
    int nchar;
    uint8_t via_bits;
    if (vregister[7]) {
        via_bits = ADC_VIA_PGA_gc;
    } else {
        via_bits = ADC_VIA_DIRECT_gc;
    }
    uint8_t muxpos_bits[6], muxneg_bits[6];
    for (uint8_t ch=0; ch < n_chan; ch++) {
        muxpos_bits[ch] = via_bits | (uint8_t)vregister[10+2*ch];
        muxneg_bits[ch] = via_bits | (uint8_t)vregister[11+2*ch];
    }
    //
    adc0_init();
    timerA0_init(ticks); // period=ticks*0.8us
    timerA0_wait();
    for (uint16_t i = 0; i < n_sample; i++) {
        PORTF.OUTSET = PIN0_bm;
        for (uint8_t ch=0; ch < n_chan; ch++) {
            // Select ADC channel and make the conversion.
            ADC0.MUXPOS = muxpos_bits[ch];
            ADC0.MUXNEG = muxneg_bits[ch];
            ADC0.COMMAND = ADC_DIFF_bm | ADC_MODE_SINGLE_12BIT_gc | ADC_START_IMMEDIATE_gc;
            while (!(ADC0.INTFLAGS & ADC_SAMPRDY_bm)) { /* wait */ }
            res[ch] = ADC0.SAMPLE; // 16-bit value
        }
        // Save the sample for later.
        // Eventually, we want to save to external SPI RAM chips.
        for (uint8_t ch=0; ch < n_chan; ch++) {
            data[ch][i] = res[ch];
        }
        PORTF.OUTCLR = PIN0_bm;
        timerA0_wait();
    } 
    timerA0_close();
    //
    // After all of the collection report the values.
    for (uint16_t i = 0; i < n_sample; i++) {
        nchar = snprintf(str_buf, NSTRBUF, "\r\ncount=%6d %6d %6d %6d %6d %6d %6d",
                         i, data[0][i], data[1][i], data[2][i],
                         data[3][i], data[4][i], data[5][i]);
        usart0_putstr(str_buf);
    }
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
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 0  sample period in timer ticks (0.8us ticks)"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 1  number of channels to sample"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 2  number of samples in record"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 3  trigger mode 0=immediate, 1=internal, 2=external"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 4  trigger channel for internal trigger"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 5  trigger level as an 11-bit count, 0-2047"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 6  number of samples to collect after trigger event"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 7  PGA flag for all channels, 0=direct 1=via_PGA"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 8  PGA gain 0=8X"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 9  V_REF 0=1.024V"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 10 CH0+"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 11 CH0-"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 12 CH1+"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 13 CH1-"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 14 CH2+"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 15 CH2-"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 16 CH3+"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 17 CH3-"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 18 CH4+"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 19 CH4-"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 20 CH5+"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 21 CH5-"); usart0_putstr(str_buf);
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
