// daq_mcu.c
// Use an AVR64EA28-I/SP as the DAQ-MCU in the two-processor DAQ node.
// Look at the notes in the USQ edaqs workbook pages 47 ff November 2023.
//
// PJ
// 2023-11-21 First cut adapted from the peripheral demo codes.
// 2023-11-21 Add the basic command interpreter and let it run the show.
// 2023-12-03 Make use of external memory for sample storage.
// 2023-12-03 EEPROM code for saving and restoring config register values.

// This version string will be printed shortly after MCU reset.
#define VERSION_STR "v0.9 2023-12-04"

#include "global_defs.h"
#include <xc.h>
#include <avr/cpufunc.h>
#include <util/delay.h>
#include "usart.h"
#include "timerA-free-run.h"
#include "spi_sram.h"
#include "eeprom.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#pragma config PERIOD = PERIOD_OFF, WINDOW = WINDOW_OFF
#pragma config SLEEP = SLEEP_DISABLE, ACTIVE = ACTIVE_DISABLE, SAMPFREQ = SAMPFREQ_128HZ, LVL = LVL_BODLEVEL0
#pragma config EESAVE = EESAVE_DISABLE, RSTPINCFG = RSTPINCFG_RESET, UPDIPINCFG = UPDIPINCFG_UPDI, CRCSEL = CRCSEL_CRC16, CRCSRC = CRCSRC_NOCRC
#pragma config SUT = SUT_64MS

// String buffer for assembling text for output.
#define NSTRBUF 128
char str_buf[NSTRBUF];
// Text buffer for incoming commands.
// They are expected to be short.
// Be careful, overruns are not handled well.
#define NCMDBUF 64
char cmd_buf[NCMDBUF];

#define MAXNCHAN 12
int16_t res[MAXNCHAN];

// State of play is indicated by these flags.
uint32_t next_byte_addr_in_SRAM;
uint8_t busy_n;
uint8_t event_n;

static inline void assert_event_pin()
{
    // Actively pull the pin low.
    PORTF.OUTCLR = PIN1_bm;
    PORTF.DIRSET = PIN1_bm;
    event_n = 0;
}

static inline void release_event_pin()
{
    // Release drive on the pin.
    PORTF.DIRSET = PIN1_bm;
    event_n = 1;
}

static inline uint8_t read_event_pin()
{
    return (PORTF.IN & PIN1_bm) ? 1 : 0;   
}

static inline void assert_busy_pin()
{
    // Actively pull the pin low.
    PORTF.OUTCLR = PIN0_bm;
    PORTF.DIRSET = PIN0_bm;
    busy_n = 0;
}

static inline void release_busy_pin()
{
    // Release drive on the pin.
    PORTF.DIRCLR = PIN0_bm;
    busy_n = 1;
}

static inline void sampling_LED_ON()
{
    PORTA.OUTSET = PIN3_bm;
}

static inline void sampling_LED_OFF()
{
    PORTA.OUTCLR = PIN3_bm;
}

// Bit patterns for selecting analog-input pins.
const uint8_t muxpos_pin[] = {
    ADC_MUXPOS_AIN28_gc, // [0] = PC0  (actually decimal 28)
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
    ADC_MUXPOS_AIN7_gc,  // [11] = PD7 (actually decimal 7)
    ADC_MUXPOS_GND_gc,   // [12] = GND (actually 0x30 decimal 48)
};
const uint8_t muxneg_pin[] = {
    ADC_MUXNEG_AIN28_gc, // [0] = PC0  (actually decimal 28)
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
    ADC_MUXNEG_AIN7_gc,  // [11] = PD7 (actually decimal 7)
    ADC_MUXNEG_GND_gc,   // [12] = GND (actually 0x30)
};

// Parameters controlling the device are stored in virtual config registers.
#define NUMREG 34
int16_t vregister[NUMREG]; // working copy in SRAM

const char hint[NUMREG][12] = {
    "PER_TICKS", // 0
    "NCHANNELS", // 1
    "NSAMPLES",  // 2
    "TRIG_MODE", // 3
    "TRIG_CHAN", // 4
    "TRIG_LEVEL",// 5
    "POST_TRIG", // 6
    "PGA_FLAG",  // 7
    "PGA_GAIN",  // 8
    "V_REF",     // 9
    "CH0+",      // 10
    "CH0-",      // 11
    "CH1+",      // 12
    "CH1-",      // 13
    "CH2+",      // 14
    "CH2-",      // 15
    "CH3+",      // 16
    "CH3-",      // 17
    "CH4+",      // 18
    "CH4-",      // 19
    "CH5+",      // 20
    "CH5-",      // 21
    "CH6+",      // 22
    "CH6-",      // 23
    "CH7+",      // 24
    "CH7-",      // 25
    "CH8+",      // 26
    "CH8-",      // 27
    "CH9+",      // 28
    "CH9-",      // 29
    "CH10+",     // 30
    "CH10-",     // 31
    "CH11+",     // 32
    "CH11-"      // 33
};

void set_registers_to_original_values()
{
    vregister[0] = 1250; // sample period in timer ticks 
    vregister[1] = 6;    // number of channels to sample
    vregister[2] = 128;  // number of samples in record
    vregister[3] = 0;    // trigger mode 0=immediate, 1=internal, 2=external
    vregister[4] = 0;    // trigger channel for internal trigger
    vregister[5] = 100;  // trigger level as an 11-bit count, 0-2048
    vregister[6] = 128;  // number of samples to collect after trigger event
    vregister[7] = 0;    // PGA flag for all channels, 0=direct 1=via_PGA
    vregister[8] = 0;    // PGA gain 0=8X
    vregister[9] = 0;    // V_REF 0=1.024V
    // The first 6 channels (0 through 5) default to differential inputs,
    // spread across the full set of 12 analog pins.
    // This is expected to be the usual arrangement.
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
    // The second set of 6 channels default to single-ended inputs
    // across the later set of 6 analog pins.
    // If we happen to want to record 12 single-ended channels,
    // we need change only the selections for channels 0 through 5
    // because channels 6 through 100 are ready to be used.
    vregister[22] = muxpos_pin[6];  // CH6+ = PD2 by default
    vregister[23] = muxneg_pin[12]; // CH6- = GND by default
    vregister[24] = muxpos_pin[7];  // CH7+ = PD3 by default
    vregister[25] = muxneg_pin[12]; // CH7- = GND by default
    vregister[26] = muxpos_pin[8];  // CH8+ = PD4 by default
    vregister[27] = muxneg_pin[12]; // CH8- = GND by default
    vregister[28] = muxpos_pin[9];  // CH9+ = PD5 by default
    vregister[29] = muxneg_pin[12]; // CH9- = GND by default
    vregister[30] = muxpos_pin[10]; // CH10+ = PD6 by default
    vregister[31] = muxneg_pin[12]; // CH10- = GND by default
    vregister[32] = muxpos_pin[11]; // CH11+ = PD7 by default
    vregister[33] = muxneg_pin[12]; // CH11- = GND by default
}


// [TODO] EEPROM data should start off like the original values above.

void save_registers_to_EEPROM()
{
    for (int i=0; i < NUMREG; i++) {
        eeprom_write_byte(2*i, (uint8_t)(vregister[i] & 0x00ff));
        eeprom_write_byte(2*i+1, (uint8_t)((vregister[i] & 0xff00) >> 8));
    }
}

void restore_registers_from_EEPROM()
{
    for (int i=0; i < NUMREG; i++) {
        vregister[i] = (int16_t)eeprom_read_byte(2*i) | ((int16_t)eeprom_read_byte(2*i+1) << 8);
    }
}

void iopins_init(void)
{
    // UART0
    PORTA.DIRSET = PIN0_bm; // 0.TX
    PORTA.DIRCLR = PIN1_bm; // 0.RX
    //
    // SPI connection to external SRAM chip
    PORTA.DIRSET = PIN7_bm; // Set PA7 to output for SPI CS_A#
    PORTA.OUTSET = PIN7_bm; // CS_A# high
    PORTA.DIRSET = PIN4_bm; // 0.MOSI
    PORTA.DIRSET = PIN6_bm; // 0.SCK
    PORTA.DIRCLR = PIN5_bm; // 0.MISO
    // Reserve PF2 for CS_B#.
    PORTF.DIRSET = PIN2_bm;
    PORTF.OUTSET = PIN2_bm;
    //
    // Use PF0 to indicate ready/busy#.
    release_busy_pin();
    // Use PF1 to indicate event#.
    release_event_pin();
    // Use PF3 to indicate period of sampling.
    PORTA.DIRSET = PIN3_bm;
    sampling_LED_OFF();
    //
    // 12 analog-in pins
    PORTC.DIRCLR = PIN0_bm; // Input for AIN28
    PORTC.DIRCLR = PIN1_bm; // Input for AIN29
    PORTC.DIRCLR = PIN2_bm; // Input for AIN30
    PORTC.DIRCLR = PIN3_bm; // Input for AIN31
    PORTD.DIRCLR = PIN0_bm; // Input for AIN0
    PORTD.DIRCLR = PIN1_bm; // Input for AIN1
    PORTD.DIRCLR = PIN2_bm; // Input for AIN2
    PORTD.DIRCLR = PIN3_bm; // Input for AIN3
    PORTD.DIRCLR = PIN4_bm; // Input for AIN4
    PORTD.DIRCLR = PIN5_bm; // Input for AIN5
    PORTD.DIRCLR = PIN6_bm; // Input for AIN6
    PORTD.DIRCLR = PIN7_bm; // Input for AIN7
    //
    return;
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

void adc0_close(void)
{
    ADC0.CTRLA &= ~ADC_ENABLE_bm;
}


uint8_t byte_addr_increment(uint8_t n_chan)
{
    // We choose an increment such that wrap-around within the 23LC1024 chip
    // keeps the sample sets aligned.
    switch (n_chan) {
        case 0: return 2;
        case 1: return 2;
        case 2: return 4;
        case 3: return 8;
        case 4: return 8;
        case 5: return 16;
        case 6: return 16;
        case 7: return 16;
        case 8: return 16;
        case 9: return 32;
        case 10: return 32;
        case 11: return 32;
        case 12: return 32;
        default: return 32;
    }
    return 32;
}

void sample_channels(void)
// Sample the analog channels periodically and store the data in external SRAM.
// 
// For mode=0, we will consider that the trigger event is immediate, at sample 0,
// and the record will stop after a specified number of samples.
// So long as the record does not wrap around, the oldest sample set will start at
// byte address 0.
//
// For mode=1 or 2, we will start sampling into the external SRAM 
// for an indefinite number of samples, while waiting for the trigger event.
// Once the trigger event happens, we will continue the record for a specified
// number of samples.  Because we do not keep a record of the number of times 
// that the SRAM address wraps around, we just assume that the oldest sample
// starts at the next byte address to be used to store a sample.
//
{
    // Get configuration data from virtual registers.
    uint16_t ticks = (uint16_t)vregister[0];
    uint8_t n_chan = (uint8_t)vregister[1];
    uint8_t mode = (uint8_t)vregister[3];
# define MODE_IMMEDIATE 0
# define MODE_INTERNAL 1
# define MODE_EXTERNAL 2
    uint8_t trigger_chan = (uint8_t)vregister[4];
    int16_t trigger_level = vregister[5];
    //
    uint8_t byte_addr_incr = byte_addr_increment(n_chan);
    uint8_t via_bits;
    if (vregister[7]) {
        via_bits = ADC_VIA_PGA_gc;
    } else {
        via_bits = ADC_VIA_DIRECT_gc;
    }
    uint8_t muxpos_bits[MAXNCHAN], muxneg_bits[MAXNCHAN];
    for (uint8_t ch=0; ch < n_chan; ch++) {
        muxpos_bits[ch] = via_bits | (uint8_t)vregister[10+2*ch];
        muxneg_bits[ch] = via_bits | (uint8_t)vregister[11+2*ch];
    }
    //
    adc0_init();
    next_byte_addr_in_SRAM = 0; // Start afresh, at address 0.
    uint8_t post_event = 0;
    uint16_t samples_remaining;
    if (mode == MODE_IMMEDIATE) {
        samples_remaining = (uint16_t)vregister[2];
    } else {
        samples_remaining = (uint16_t)vregister[6];
    }
    timerA0_init(ticks); // period=ticks*0.8us
    timerA0_wait();
    //
    while (samples_remaining > 0) {
        sampling_LED_ON();
        for (uint8_t ch=0; ch < n_chan; ch++) {
            // Select ADC channel and make the conversion.
            ADC0.MUXPOS = muxpos_bits[ch];
            ADC0.MUXNEG = muxneg_bits[ch];
            ADC0.COMMAND = ADC_DIFF_bm | ADC_MODE_SINGLE_12BIT_gc | ADC_START_IMMEDIATE_gc;
            while (!(ADC0.INTFLAGS & ADC_SAMPRDY_bm)) { /* wait */ }
            res[ch] = ADC0.SAMPLE; // 16-bit value
        }
        // Save the sample for later.
        spi0_send_sample_data(res, n_chan, next_byte_addr_in_SRAM);
        // Point to the next available SRAM address.
        next_byte_addr_in_SRAM += byte_addr_incr;
        next_byte_addr_in_SRAM &= 0x0001FFFFUL; // Wrap around at 128kB
        sampling_LED_OFF();
        //
        if (post_event) {
            samples_remaining--;
        } else {
            // Always trigger for the moment.
            post_event = 1;            
        }
        timerA0_wait();
    } 
    timerA0_close();
    adc0_close();
} // end void sample_channels()

#define NSTRBUF1 128
char str_buf1[NSTRBUF1];
#define NSTRBUF2 16
char str_buf2[NSTRBUF2];

char* sample_set_to_str(uint16_t n)
{
    int nchar;
    uint8_t n_chan = (uint8_t)vregister[1];
    uint8_t mode = (uint8_t)vregister[3];
    uint32_t addr = (mode == 0) ? 0 : next_byte_addr_in_SRAM;
    uint8_t byte_addr_incr = byte_addr_increment(n_chan);
    addr += byte_addr_incr * n;
    addr &= 0x0001FFFFUL; // Wrap around at 128kB
    spi0_fetch_sample_data(res, n_chan, addr);
    nchar = snprintf(str_buf1, NSTRBUF1, "%6d", res[0]);
    for (uint8_t i=1; i < n_chan; i++) {
        nchar = snprintf(str_buf2, NSTRBUF2, " %6d", res[i]);
        strncat(str_buf1, str_buf2, NSTRBUF2);
    }
    return str_buf1;
}

void report_values(void)
// Report the previously-collected data to the UART.
// Assume a simple sampling process with immediate event.
{
    uint8_t mode = (uint8_t)vregister[3];
    uint8_t n_chan = (uint8_t)vregister[1];
    uint8_t byte_addr_incr = byte_addr_increment(n_chan);
    uint16_t n_sample = (uint16_t)vregister[2]; // [FIX-ME] for other modes
    int nchar;
    for (uint16_t i = 0; i < n_sample; i++) {
        nchar = snprintf(str_buf, NSTRBUF, "\r\ni=%6d data=%s", i, sample_set_to_str(i));
        usart0_putstr(str_buf);
    }
} // end void report_values()


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
                nchar = snprintf(str_buf, NSTRBUF, "\r\nreg[%d]=%d   (%s)", 
                        i, vregister[i], hint[i]);
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
            restore_registers_from_EEPROM();
            nchar = snprintf(str_buf, NSTRBUF, "ok");
            usart0_putstr(str_buf);
            break;
        case 'S':
            save_registers_to_EEPROM();
            nchar = snprintf(str_buf, NSTRBUF, "ok");
            usart0_putstr(str_buf);
            break;
        case 'F':
            set_registers_to_original_values();
            nchar = snprintf(str_buf, NSTRBUF, "ok");
            usart0_putstr(str_buf);
            break;
        case 'g':
            // The task takes an indefinite time, so let the COMMS_MCU know.
            assert_busy_pin();
            sample_channels();
            release_busy_pin();
            break;
        case 'G':
            // The task takes an indefinite time, so let the COMMS_MCU know.
            assert_busy_pin();
            sample_channels();
            report_values();
            release_busy_pin();
            break;
        case 'P':
            token_ptr = strtok(&cmd_buf[1], sep_tok);
            if (token_ptr) {
                // Found some nonblank text, assume channel number.
                i = (uint16_t) atoi(token_ptr);
                nchar = snprintf(str_buf, NSTRBUF, "%s ok", sample_set_to_str(i));
            } else {
                nchar = snprintf(str_buf, NSTRBUF, "fail");
            }
            usart0_putstr(str_buf);
            break;
        case 'a': {
            uint8_t mode = (uint8_t)vregister[3];
            uint32_t addr = (mode == 0) ? 0 : next_byte_addr_in_SRAM;
            nchar = snprintf(str_buf, NSTRBUF, "%lu ok", addr);
            usart0_putstr(str_buf); }
            break;
        case 'b': {
            uint16_t bincr = byte_addr_increment((uint8_t)vregister[1]);
            nchar = snprintf(str_buf, NSTRBUF, "%u ok", bincr);
            usart0_putstr(str_buf); }
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
            nchar = snprintf(str_buf, NSTRBUF, "\r\n R      restore register values from EEPROM"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n S      save register values to EEPROM"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n F      set register values to original values"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n g      go and start sampling (no report)"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n G      go and start sampling, and then report"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n P <i>  report sample set i (i=0 for oldest data)"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n M <i>  SRAM memory dump from byte address i"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n a      report byte address of oldest data"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n b      report size of a sample set in bytes"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\nRegisters:"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 0  sample period in timer ticks (0.8us ticks)"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 1  number of channels to sample"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 2  number of samples in record (mode=0)"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 3  trigger mode 0=immediate, 1=internal, 2=external"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 4  trigger channel for internal trigger"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 5  trigger level as an 11-bit count, 0-2047"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 6  number of samples to collect after trigger event (mode=1,2_"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 7  PGA flag for all channels, 0=direct 1=via_PGA"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 8  PGA gain 0=8X"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 9  V_REF 0=1.024V"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 10 CH0+   22 CH6+"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 11 CH0-   23 CH6-"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 12 CH1+   24 CH7+"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 13 CH1-   25 CH7-"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 14 CH2+   26 CH8+"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 15 CH2-   27 CH8-"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 16 CH3+   28 CH9+"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 17 CH3-   29 CH9-"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 18 CH4+   30 CH10+"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 19 CH4-   31 CH10-"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 20 CH5+   32 CH11+"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\r\n 21 CH5-   33 CH11-"); usart0_putstr(str_buf);
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
    usart0_init(460800);
    spi0_init();
    //
    nchar = snprintf(str_buf, NSTRBUF, "\r\nAVR64EA28 DAQ-MCU\r\n%s", VERSION_STR);
    usart0_putstr(str_buf);
    //
    restore_registers_from_EEPROM();
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
    spi0_close();
    usart0_close();
    return 0;
}
