// spi_sram.c
// Send bytes to and retrieve bytes from the external memory.
// PJ 2023-12-03
//
// The external memory consists of one 23LC1024-I/P SRAM chip
// attached to SPI0.  Eventually, we may attach another chip.
// With 128kB storage, the byte address within a chip is given
// by 17 bits.  We could use bit 18 to indicate that we need 
// to select the second chip, however, we ignore that for the moment.
//
// Pin Assignments
// CS#    == PA7
// 0.SCK  == PA6
// 0.MISO == PA5
// 0.MOSI == PA4

#include "global_defs.h"
#include <xc.h>
#include <stdint.h>

void spi0_init()
{
    // Manually drive the SS# pin for the SRAM chip.
    PORTA.OUTSET |= PIN7_bm;
    SPI0.CTRLA = SPI_MASTER_bm | SPI_CLK2X_bm | SPI_PRESC_DIV4_gc;
    SPI0.CTRLB = SPI_SSD_bm | SPI_MODE_0_gc;
    SPI0.CTRLA |= SPI_ENABLE_bm;
}

static inline
uint8_t spi0_exchange(uint8_t b1)
{
    // On entry, we assume that previous communications have finished.
    uint8_t b2;
    if (SPI0.INTFLAGS & SPI_WRCOL_bm) {
        // Clear the wreckage from a write collision.
        b2 = SPI0.DATA;
    }
    SPI0.DATA = b1;
    while (!(SPI0.INTFLAGS & SPI_IF_bm)) { /* [TODO] clear WDT */ }
    b2 = SPI0.DATA; // This should have cleared flag.
    return b2;
}

void spi0_close()
{
    SPI0.CTRLA &= ~SPI_ENABLE_bm;
}

void spi0_send_sample_data(int16_t data[], uint8_t n, uint32_t addr)
// data : array of values to send
// n    : number of data elements to send
// addr : starting byte address within SRAM chip
{
    uint8_t b1, b2;
    if (n == 0) return; // Nothing to do.
    PORTA.OUTCLR |= PIN7_bm;
    b1 = 0b10; // Write command.
    b2 = spi0_exchange(b1);
    b1 = (uint8_t) (addr >> 16);
    b2 = spi0_exchange(b1);
    b1 = (uint8_t) (addr >> 8);
    b2 = spi0_exchange(b1);
    b1 = (uint8_t) addr;
    b2 = spi0_exchange(b1);
    for (uint8_t i=0; i < n; i++) {
        b1 = (uint8_t) (data[i] >> 8); // High byte first.
        b2 = spi0_exchange(b1);
        b1 = (uint8_t) (data[i]); // Low byte second.
        b2 = spi0_exchange(b1);
    }
    PORTA.OUTSET |= PIN7_bm;
}

void spi0_fetch_sample_data(int16_t data[], uint8_t n, uint32_t addr)
// data : array of values to fetch
// n    : number of data elements to send
// addr : starting byte address within SRAM chip
{
    uint8_t b1, b2, b3;
    if (n == 0) return; // Nothing to do.
    PORTA.OUTCLR |= PIN7_bm;
    b1 = 0b11; // Read command.
    b2 = spi0_exchange(b1);
    b1 = (uint8_t) (addr >> 16);
    b2 = spi0_exchange(b1);
    b1 = (uint8_t) (addr >> 8);
    b2 = spi0_exchange(b1);
    b1 = (uint8_t) addr;
    b2 = spi0_exchange(b1);
    b1 = 0;
    for (uint8_t i=0; i < n; i++) {
        b2 = spi0_exchange(b1); // High byte first.
        b3 = spi0_exchange(b1); // Low byte second.
        data[i] = (int16_t) ((uint16_t)b2 << 8) | b3;
    }   
    PORTA.OUTSET |= PIN7_bm;    
}
