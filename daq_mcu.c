// daq_mcu.c
// Use an AVR64EA28-I/SP as the DAQ-MCU in the two-processor DAQ node.
// Look at the notes in the USQ edaqs workbook pages 47 ff November 2023.
//
// PJ
// 2023-11-21 First cut adapted from the peripheral demo codes.
// 2023-11-21 Add the basic command interpreter and let it run the show.
// 2023-12-03 Make use of external memory for sample storage.
// 2023-12-03 EEPROM code for saving and restoring config register values.
// 2024-03-29 Less chatty mode for interfacing with PIC18 COMMS-MCU.
//            Changed to using new-line character at end of output messages.

// This version string will be reported by the version command.
#define VERSION_STR "v0.21 AVR64EA28 DAQ-MCU 2024-04-09"

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
#define NSTRBUF 256
char str_buf[NSTRBUF];
char allow_multiline_response = 0;
// Text buffer for incoming commands.
// They are expected to be short.
// Be careful, overruns are not handled well.
#define NCMDBUF 64
char cmd_buf[NCMDBUF];

#define MAXNCHAN 12
int16_t res[MAXNCHAN];

// State of play is indicated by the following data flags.
// Assume that one 23LC1024 chip is present, with 128kB memory.
// At some point in time, we should write code to probe the 
// external memory chips to see how many are actually present.
// #define TWO_SRAM_CHIPS
#ifdef TWO_SRAM_CHIPS
uint32_t size_of_SRAM_in_bytes = 0x00040000UL;
uint32_t mask_for_SRAM_addr = 0x0003FFFFUL;
#else
uint32_t size_of_SRAM_in_bytes = 0x00020000UL;
uint32_t mask_for_SRAM_addr = 0x0001FFFFUL;
#endif

uint32_t next_byte_addr_in_SRAM;
uint8_t byte_addr_has_wrapped_around;
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
    PORTF.DIRCLR = PIN1_bm;
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
    "TRIG_SLOPE", // 6
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
    vregister[2] = 128;  // number of samples in record after trigger event
    vregister[3] = 0;    // trigger mode 0=immediate, 1=internal, 2=external
    vregister[4] = 0;    // trigger channel for internal trigger
    vregister[5] = 100;  // trigger level as an 11-bit count, 0-2048
    vregister[6] = 1;    // trigger slope 0=sample-below-level 1=sample-above-level
    vregister[7] = 0;    // PGA flag for all channels, 0=direct 1=via_PGA
    vregister[8] = 0;    // PGA gain 0=1X,1=2X,2=4X,3=8X,4=16X
    vregister[9] = 0;    // V_REF 0=1.024V,1=2.048V,3=4.096V
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
    // Reserve PA2 for CS_B#.
    PORTA.DIRSET = PIN2_bm;
    PORTA.OUTSET = PIN2_bm;
    //
    // Use PF0 to indicate ready/busy#.
    release_busy_pin();
    // Use PF1 to indicate event#.
    release_event_pin();
    // Use PA3 to indicate period of sampling.
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
    switch (vregister[9]) {
        case 0:
            ADC0.CTRLC = ADC_REFSEL_VDD_gc;
            break;
        case 1:
            ADC0.CTRLC = ADC_REFSEL_1V024_gc;
            break;
        case 2:
            ADC0.CTRLC = ADC_REFSEL_2V048_gc;
            break;
        case 3:
            ADC0.CTRLC = ADC_REFSEL_4V096_gc;
            break;
        case 4:
            ADC0.CTRLC = ADC_REFSEL_2V500_gc;
            break;
        default:
            ADC0.CTRLC = ADC_REFSEL_4V096_gc;            
    }
    ADC0.CTRLE = 20; // SAMPDUR of 4 microseconds
    ADC0.CTRLF |= ADC_SAMPNUM_NONE_gc;
    uint8_t gain_gc = ADC_GAIN_1X_gc;
    switch (vregister[8]) {
        case 0:
            gain_gc = ADC_GAIN_1X_gc;
            break;
        case 1:
            gain_gc = ADC_GAIN_2X_gc;
            break;
        case 2:
            gain_gc = ADC_GAIN_4X_gc;
            break;
        case 3:
            gain_gc = ADC_GAIN_8X_gc;
            break;
        case 4:
            gain_gc = ADC_GAIN_16X_gc;
            break;
        default:
            gain_gc = ADC_GAIN_1X_gc;
    }
    ADC0.PGACTRL = gain_gc | ADC_PGABIASSEL_100PCT_gc | ADC_PGAEN_bm;
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

uint32_t max_n_samples(void)
{
    uint8_t n_chan = (uint8_t)vregister[1];
    uint8_t byte_addr_incr = byte_addr_increment(n_chan);
    return size_of_SRAM_in_bytes / byte_addr_incr;
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
# define TRIGGER_IMMEDIATE 0
# define TRIGGER_INTERNAL 1
# define TRIGGER_EXTERNAL 2
    uint8_t trigger_chan = (uint8_t)vregister[4];
    int16_t trigger_level = vregister[5];
    uint8_t trigger_slope = (uint8_t)vregister[6];
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
    release_event_pin();
    adc0_init();
    next_byte_addr_in_SRAM = 0; // Start afresh, at address 0.
    byte_addr_has_wrapped_around = 0;
    uint8_t post_event = 0;
    uint16_t samples_remaining = (uint16_t)vregister[2];
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
        if (next_byte_addr_in_SRAM >= size_of_SRAM_in_bytes) {
            next_byte_addr_in_SRAM -= size_of_SRAM_in_bytes;
            byte_addr_has_wrapped_around = 1;
        }
        sampling_LED_OFF();
        //
        if (post_event) {
            // Trigger event has happened.
            samples_remaining--;
        } else {
            // We need to decide about trigger event.
            switch (mode) {
            case TRIGGER_IMMEDIATE:
                post_event = 1;
                assert_event_pin();
                break;
            case TRIGGER_INTERNAL: {
                int16_t s = res[trigger_chan];
                if ((trigger_slope == 1 && s >= trigger_level) ||
                    (trigger_slope == 0 && s <= trigger_level)) {
                    post_event = 1;
                    assert_event_pin();
                } }
                break;
            case TRIGGER_EXTERNAL:
                if (read_event_pin() == 0) {
                    post_event = 1;
                }
            } // end switch
        }
        timerA0_wait();
    } // end while
    timerA0_close();
    adc0_close();
} // end void sample_channels()

void sample_channels_once()
{
    // We temporarily override some of the registers to make this happen.
    uint16_t ticks_save = (uint16_t)vregister[0];
    uint8_t mode_save = (uint8_t)vregister[3];
    uint16_t samples_remaining_save = (uint16_t)vregister[2];
    //
    vregister[0] = (uint16_t)200; // Time enough to do a full scan.
    vregister[3] = 0; // Immediate mode.
    vregister[2] = 1; // One sample set.
    sample_channels();
    //
    // Restore register values.
    vregister[0] = ticks_save;
    vregister[3] = mode_save;
    vregister[2] = samples_remaining_save;
    return;
}

#define NSTRBUF1 128
char str_buf1[NSTRBUF1];
#define NSTRBUF2 16
char str_buf2[NSTRBUF2];

uint32_t oldest_byte_addr_in_SRAM()
{
    return (byte_addr_has_wrapped_around) ? next_byte_addr_in_SRAM : 0;
}

char* sample_set_to_str(uint32_t n)
{
    int nchar;
    uint8_t n_chan = (uint8_t)vregister[1];
    // Start with address of oldest sample, then move to selected sample.
    uint32_t addr = oldest_byte_addr_in_SRAM();
    uint8_t byte_addr_incr = byte_addr_increment(n_chan);
    addr += byte_addr_incr * n;
    // Assume that the size of SRAM is a power of two, so that
    // a bit mask can be used for the valid address bits.
    addr &= mask_for_SRAM_addr;
    spi0_fetch_sample_data(res, n_chan, addr);
    nchar = snprintf(str_buf1, NSTRBUF1, "%d", res[0]);
    for (uint8_t i=1; i < n_chan; i++) {
        nchar = snprintf(str_buf2, NSTRBUF2, " %d", res[i]);
        strncat(str_buf1, str_buf2, NSTRBUF2);
    }
    return str_buf1;
}

#define NBYTES 32
uint8_t bytes[NBYTES];

char* mem_dump_to_str(uint32_t addr)
{
    // Write a 32-byte page of SRAM data into the buffer as pairs of hex digits.
    int nchar;
    spi0_fetch_bytes(bytes, NBYTES, addr);
    nchar = snprintf(str_buf1, NSTRBUF1, "%02x", bytes[0]);
    for (uint8_t i=1; i < NBYTES; i++) {
        nchar = snprintf(str_buf2, NSTRBUF2, "%02x", bytes[i]);
        strncat(str_buf1, str_buf2, NSTRBUF2);
    }
    return str_buf1;
}

void report_values(void)
// Report the previously-collected data to the UART.
// Assume a simple sampling process with immediate event.
{
    uint8_t mode = (uint8_t)vregister[3];
    uint32_t n_sample = (mode == TRIGGER_IMMEDIATE) ? (uint32_t)vregister[2] : max_n_samples();
    int nchar;
    for (uint32_t i = 0; i < n_sample; i++) {
        nchar = snprintf(str_buf, NSTRBUF, "i=%6lu data=%s\n", i, sample_set_to_str(i));
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
    // The following few lines for debug on the end of a TTL-232 cable.
    // nchar = snprintf(str_buf, NSTRBUF, "Command text was: ");
    // usart0_putstr(str_buf);
    // usart0_putstr(cmd_buf); usart0_putch('\n');
    // nchar = snprintf(str_buf, NSTRBUF, "Number of characters in buffer: %u\n", strlen(cmd_buf));
    // usart0_putstr(str_buf);
    switch (cmd_buf[0]) {
        case 'v':
            // Report the version string (see top of source file)
            nchar = snprintf(str_buf, NSTRBUF, "%s ok\n", VERSION_STR);
            usart0_putstr(str_buf);
            break;
        case 'n':
            // Report number of virtual registers.
            nchar = snprintf(str_buf, NSTRBUF, "%u ok\n", NUMREG);
            usart0_putstr(str_buf);
            break;
        case 'Q':
            // Set the reporting mode to allow multi-line responses.
            // This is really only useful when interacting with the AVR 
            // through a TTL-232-5V serial cable.
            // The RS484 COMMS-MCU expects single-line responses.
            allow_multiline_response = 1;
            nchar = snprintf(str_buf, NSTRBUF, "Multi-line response mode. ok\n");
            usart0_putstr(str_buf);
            break;
        case 'q':
            // Set the reporting mode to single-line responses.
            // This is the default mode and the one intended for interaction
            // via the RS485 COMMS-MCU.
            allow_multiline_response = 0;
            nchar = snprintf(str_buf, NSTRBUF, "Single-line response mode. ok\n");
            usart0_putstr(str_buf);
            break;
        case 'p':
            // Print all virtual register values.
            // Should not be used via the RS485 COMMS-MCU.
            if (allow_multiline_response) {
                nchar = snprintf(str_buf, NSTRBUF, "Register values:\n");
                usart0_putstr(str_buf);
                for (i=0; i < NUMREG; ++i) {
                    nchar = snprintf(str_buf, NSTRBUF, "reg[%d]=%d   (%s)\n",
                                     i, vregister[i], hint[i]);
                    usart0_putstr(str_buf);
                }
                nchar = snprintf(str_buf, NSTRBUF, "ok\n");
            } else {
                nchar = snprintf(str_buf, NSTRBUF, "fail: Too many values to show.\n");
            }
            usart0_putstr(str_buf);
            break;
        case 'r':
            // Report a selected register value.
            token_ptr = strtok(&cmd_buf[1], sep_tok);
            if (token_ptr) {
                // Found some nonblank text, assume register number.
                i = (uint8_t) atoi(token_ptr);
                if (i < NUMREG) {
                    v = vregister[i];
                    nchar = snprintf(str_buf, NSTRBUF, "%d ok\n", v);
                } else {
                    nchar = snprintf(str_buf, NSTRBUF, "fail: Invalid register.\n");
                }
            } else {
                nchar = snprintf(str_buf, NSTRBUF, "fail: No register specified.\n");
            }
            usart0_putstr(str_buf);
            break;
        case 's':
            // Set a selected register value.
            token_ptr = strtok(&cmd_buf[1], sep_tok);
            if (token_ptr) {
                // Found some nonblank text; assume register number.
                i = (uint8_t) atoi(token_ptr);
                if (i < NUMREG) {
                    token_ptr = strtok(NULL, sep_tok);
                    if (token_ptr) {
                        // Assume text is value for register.
                        v = (int16_t) atoi(token_ptr);
                        vregister[i] = v;
                        nchar = snprintf(str_buf, NSTRBUF, "reg[%u] %d ok\n", i, v);
                    } else {
                        nchar = snprintf(str_buf, NSTRBUF, "fail: No value given.\n");
                    }
                } else {
                    nchar = snprintf(str_buf, NSTRBUF, "fail: Invalid register.\n");
                }
            } else {
                nchar = snprintf(str_buf, NSTRBUF, "fail: No register specified.\n");
            }
            usart0_putstr(str_buf);
            break;
        case 'R':
            // Read all of the virtual register values from the AVR's EEPROM.
            restore_registers_from_EEPROM();
            nchar = snprintf(str_buf, NSTRBUF, "ok\n");
            usart0_putstr(str_buf);
            break;
        case 'S':
            // Save all of the current values from the virtual registers
            // to the AVR's on-chip EEPROM.
            save_registers_to_EEPROM();
            nchar = snprintf(str_buf, NSTRBUF, "ok\n");
            usart0_putstr(str_buf);
            break;
        case 'F':
            // Set the values of the registers to those values hard-coded
            // into this firmware.  A factory default, so to speak.
            set_registers_to_original_values();
            nchar = snprintf(str_buf, NSTRBUF, "ok\n");
            usart0_putstr(str_buf);
            break;
        case 'g':
            // Start the sampling process.
            // What happens next, and when it happens, depends on the
            // register settings and external signals.
            nchar = snprintf(str_buf, NSTRBUF, "ok\n");
            usart0_putstr(str_buf);
            // The task takes an indefinite time,
            // so let the COMMS_MCU know via busy# pin.
            assert_busy_pin();
            sample_channels();
            release_busy_pin();
            break;
        case 'G':
            // Immediately take many samples and (eventually) report values.
            // Not to be used with the RS485 COMMS-MCU.
            if (allow_multiline_response) {
                nchar = snprintf(str_buf, NSTRBUF, "ok\n");
                usart0_putstr(str_buf);
                // The task takes an indefinite time,
                // so let the COMMS_MCU know via busy# pin.
                assert_busy_pin();
                sample_channels();
                report_values();
                release_busy_pin();
            } else {
                nchar = snprintf(str_buf, NSTRBUF, "fail: Action not available.\n");
                usart0_putstr(str_buf);                
            }
            break;
        case 'I':
            // Immediately take a single sample set and report values.
            sample_channels_once();
            nchar = snprintf(str_buf, NSTRBUF, "%s ok\n", sample_set_to_str(0));
            usart0_putstr(str_buf);
            break;
        case 'P':
            // Report the selected sample set for the configured channels.
            // An index of 0 refers to the oldest sample set.
            token_ptr = strtok(&cmd_buf[1], sep_tok);
            if (token_ptr) {
                // Found some nonblank text, assume sample index.
                uint32_t ii = (uint32_t) atol(token_ptr);
                nchar = snprintf(str_buf, NSTRBUF, "%s ok\n", sample_set_to_str(ii));
            } else {
                nchar = snprintf(str_buf, NSTRBUF, "fail: No index given.\n");
            }
            usart0_putstr(str_buf);
            break;
        case 'a': {
            // Report the byte-address in SRAM of the oldest sample set.
            uint32_t addr = oldest_byte_addr_in_SRAM();
            nchar = snprintf(str_buf, NSTRBUF, "%lu ok\n", addr);
            usart0_putstr(str_buf); }
            break;
        case 'b': {
            // Report the number of bytes in SRAM for each sample set.
            uint16_t bincr = byte_addr_increment((uint8_t)vregister[1]);
            nchar = snprintf(str_buf, NSTRBUF, "%u ok\n", bincr);
            usart0_putstr(str_buf); }
            break;
        case 'm':
            // Report the maximum number of sample seta that the SRAM can hold.
            nchar = snprintf(str_buf, NSTRBUF, "%lu ok\n", max_n_samples());
            usart0_putstr(str_buf);
            break;
        case 'T':
            // Report the total number of bytes in the SRAM chips.
            nchar = snprintf(str_buf, NSTRBUF, "%lu ok\n", size_of_SRAM_in_bytes);
            usart0_putstr(str_buf);
            break;
        case 'M':
            // Report the content of a 32-byte 'page' in SRAM,
            // starting at the specified byte address.
            token_ptr = strtok(&cmd_buf[1], sep_tok);
            if (token_ptr) {
                // Found some nonblank text, assume address.
                uint32_t addr = (uint32_t) atol(token_ptr);
                nchar = snprintf(str_buf, NSTRBUF, "%s ok\n", mem_dump_to_str(addr));
            } else {
                nchar = snprintf(str_buf, NSTRBUF, "fail: No address given.\n");
            }
            usart0_putstr(str_buf);
            break;
        case 'N':
            // Report the total number of 32-byte pages in the SRAM chips.
            nchar = snprintf(str_buf, NSTRBUF, "%lu ok\n", size_of_SRAM_in_bytes/32);
            usart0_putstr(str_buf);
            break;
        case 'z':
            // Release the EVENT# line.
            // Presumably this line has been help low following an internal
            // trigger event during the sampling process.
            release_event_pin();
            nchar = snprintf(str_buf, NSTRBUF, "ok\n");
            usart0_putstr(str_buf);
            break;
        case 'h':
        case '?':
            if (!allow_multiline_response) {
                nchar = snprintf(str_buf, NSTRBUF, "In single-line response mode. Q to get multi-line mode. ok\n");
                usart0_putstr(str_buf);
                break;
            }
            nchar = snprintf(str_buf, NSTRBUF, "AVR64EA28 DAQ-MCU commands and registers\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "Commands:\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " h or ? print this help message\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " v      report version of firmware\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " Q      Change to multi-line response mode\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " q      Change to single-line response mode\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " n      report number of registers\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " p      report register values\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " r <i>  report value of register i\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " s <i> <j>  set register i to value j\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " R      restore register values from EEPROM\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " S      save register values to EEPROM\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " F      set register values to original values\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " g      go and start sampling (no report)\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " G      go and start sampling, and then report\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " I      sample channels once and report\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " P <i>  report sample set i (i=0 for oldest data)\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " M <i>  dump SRAM memory from byte address i\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " a      report byte address of oldest data\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " b      report size of a sample set in bytes\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " m      report max number of samples in SRAM\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " T      report total size of SRAM in bytes\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " N      report total number of 32-bit pages in SRAM\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " z      release EVENTn line\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "Registers:\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " 0  sample period in timer ticks (0.8us ticks)\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " 1  number of channels to sample\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " 2  number of samples after trigger event\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " 3  trigger mode 0=immediate, 1=internal, 2=external\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " 4  trigger channel for internal trigger\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " 5  trigger level as an 11-bit count, 0-2047\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " 6  trigger slope 0=below-level, 1=above-level\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " 7  PGA flag for all channels, 0=direct 1=via_PGA\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " 8  PGA gain 0=1X, 1=2X, 2=4X, 3=8X, 4=16X\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " 9  V_REF 0=VDD, 1=1V024, 2=2V048, 3=4V096, 4=2V500\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " 10 CH0+   22 CH6+\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " 11 CH0-   23 CH6-\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " 12 CH1+   24 CH7+\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " 13 CH1-   25 CH7-\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " 14 CH2+   26 CH8+\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " 15 CH2-   27 CH8-\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " 16 CH3+   28 CH9+\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " 17 CH3-   29 CH9-\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " 18 CH4+   30 CH10+\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " 19 CH4-   31 CH10-\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " 20 CH5+   32 CH11+\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, " 21 CH5-   33 CH11-\n"); usart0_putstr(str_buf);
            nchar = snprintf(str_buf, NSTRBUF, "ok\n"); usart0_putstr(str_buf);
            break;
        default:
            nchar = snprintf(str_buf, NSTRBUF, "fail: Unknown command.\n"); usart0_putstr(str_buf);
    } // end switch
} // end interpret_command())


int main(void)
{
    int ncmd, nchar;
    //
    // Turn off the main clock prescaler so that we run at full 20MHz.
    ccp_write_io((void *) &(CLKCTRL.MCLKCTRLB), (CLKCTRL.MCLKCTRLB & 0xfe));
    CLKCTRL.MCLKTIMEBASE = TIMEBASE_VALUE; // Needed for the ADC
    iopins_init();
    _delay_ms(10); // Let the pins settle, to reduce garbage on the RX pin.
    usart0_init(230400);
    spi0_init();
    // 2024-03-29 Change to a less chatty mode where the AVR only outputs
    // text in response to incoming commands.
    restore_registers_from_EEPROM();
    // Flash the Green LED twice to signal that the MCU is ready.
    for (int8_t i=0; i < 2; ++i) {
        sampling_LED_ON();
        _delay_ms(250);
        sampling_LED_OFF();
        _delay_ms(250);
    }
    //
    // The basic behaviour is to be forever checking for a text command.
    while (1) {
        // Characters are NOT echoed as they are typed.
        // Backspace deleting is allowed.
        ncmd = usart0_getstr(cmd_buf, NCMDBUF);
        if (ncmd) {
            interpret_command();
        }
    } // never-ending while
    spi0_close();
    usart0_close();
    return 0;
}
