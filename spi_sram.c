// spi_sram.c
// Send bytes to and retrieve bytes from the external memory.
// PJ 2023-12-03 Single-chip implementation
//    2025-04-18 The time has come to accommodate a second chip.
//
// The external memory consists of one or two 23LC1024-I/P SRAM chips
// attached to SPI0.  PA7 is SSn for chip 0, PA2 is SSn for chip 1. 
// With 128kB storage, the byte address within a chip is given
// by 17 bits.  We use bit 18 to select the second chip.
//
// Pin Assignments
// CS_A#  == PA7
// CS_B#  == PA2
// 0.SCK  == PA6
// 0.MISO == PA5
// 0.MOSI == PA4

#include "global_defs.h"
#include <xc.h>
#include <stdint.h>

void spi0_init()
{
    // Manually drive the SS# pins for the SRAM chips.
    PORTA.OUTSET |= PIN2_bm | PIN7_bm;
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

uint8_t detect_SRAM_chips()
// Return bits 0 and 1 set to indicate presence of SRAM chips.
{
    uint8_t chips = 0;
    uint8_t b1, b2;
    // Test first chip for 0k <= addr < 128k
    // Write a two-byte value.
    PORTA.OUTCLR |= PIN7_bm;
    b2 = spi0_exchange(0b10); // Write command.
    b2 = spi0_exchange(0); // to address 0
    b2 = spi0_exchange(0);
    b2 = spi0_exchange(0);
    b2 = spi0_exchange(0xa);
    b2 = spi0_exchange(0x5);
    PORTA.OUTSET |= PIN7_bm;
    NOP(); NOP(); NOP(); NOP();
    // and read it back.
    PORTA.OUTCLR |= PIN7_bm;
    b2 = spi0_exchange(0b11); // Read command.
    b2 = spi0_exchange(0); // from address 0
    b2 = spi0_exchange(0);
    b2 = spi0_exchange(0);
    b1 = spi0_exchange(0);
    b2 = spi0_exchange(0);
    PORTA.OUTSET |= PIN7_bm;
    if (b1 == 0xa && b2 == 0x5) {
        // We have read back the correct values,
        // So we assume the the first chip is present.
        chips |= 0x01;
    }
    // Test second chip for 128k <= addr < 256k
    // Write a two-byte value.
    PORTA.OUTCLR |= PIN2_bm;
    b2 = spi0_exchange(0b10); // Write command.
    b2 = spi0_exchange(0); // to address 0
    b2 = spi0_exchange(0);
    b2 = spi0_exchange(0);
    b2 = spi0_exchange(0xa);
    b2 = spi0_exchange(0x5);
    PORTA.OUTSET |= PIN2_bm;
    NOP(); NOP(); NOP(); NOP();
    // and read it back.
    PORTA.OUTCLR |= PIN2_bm;
    b2 = spi0_exchange(0b11); // Read command.
    b2 = spi0_exchange(0); // from address 0
    b2 = spi0_exchange(0);
    b2 = spi0_exchange(0);
    b1 = spi0_exchange(0);
    b2 = spi0_exchange(0);
    PORTA.OUTSET |= PIN2_bm;
    if (b1 == 0xa && b2 == 0x5) {
        // We have read back the correct values,
        // So we assume the the first chip is present.
        chips |= 0x02;
    }
    return chips;
}

void spi0_send_sample_data(int16_t data[], uint8_t n, uint32_t addr)
// data : array of values to send
// n    : number of data elements to send
// addr : starting byte address within the SRAM chips
//
// Note that we store the 16-bit data into the external SRAM with
// big-endian byte order.  This gives an easy-to-read byte stream
// when formatted in hexadecimal. 
{
    uint8_t b1, b2;
    if (n == 0) return; // Nothing to do.
    // We select which SRAM chip using bit17 of the provided address.
    if (addr & 0x00020000UL) {
        // Select second chip for 128k <= addr < 256k
        PORTA.OUTCLR |= PIN2_bm;
    } else {
        // Select first chip for 0k <= addr < 128k
        PORTA.OUTCLR |= PIN7_bm;
    }
    b1 = 0b10; // Write command.
    b2 = spi0_exchange(b1);
    b1 = (uint8_t) ((addr >> 16) & 0x01); // Upper byte, keep only bit16 of addr.
    b2 = spi0_exchange(b1);
    b1 = (uint8_t) (addr >> 8); // High byte. 
    b2 = spi0_exchange(b1);
    b1 = (uint8_t) addr; // Low byte.
    b2 = spi0_exchange(b1);
    for (uint8_t i=0; i < n; i++) {
        b1 = (uint8_t) (data[i] >> 8); // High byte first.
        b2 = spi0_exchange(b1);
        b1 = (uint8_t) (data[i]); // Low byte second.
        b2 = spi0_exchange(b1);
    }
    PORTA.OUTSET |= PIN2_bm | PIN7_bm;
}

void spi0_fetch_sample_data(int16_t data[], uint8_t n, uint32_t addr)
// data : array of values to fetch
// n    : number of data elements to send
// addr : starting byte address within SRAM chip
{
    uint8_t b1, b2, b3;
    if (n == 0) return; // Nothing to do.
    // We select which SRAM chip using bit17 of the provided address.
    if (addr & 0x00020000UL) {
        // Select second chip for 128k <= addr < 256k
        PORTA.OUTCLR |= PIN2_bm;
    } else {
        // Select first chip for 0k <= addr < 128k
        PORTA.OUTCLR |= PIN7_bm;
    }
    b1 = 0b11; // Read command.
    b2 = spi0_exchange(b1);
    b1 = (uint8_t) ((addr >> 16) & 0x01); // Upper byte, keep only bit16 of addr.
    b2 = spi0_exchange(b1);
    b1 = (uint8_t) (addr >> 8); // High byte.
    b2 = spi0_exchange(b1);
    b1 = (uint8_t) addr; // Low byte.
    b2 = spi0_exchange(b1);
    b1 = 0;
    for (uint8_t i=0; i < n; i++) {
        b2 = spi0_exchange(b1); // High byte first.
        b3 = spi0_exchange(b1); // Low byte second.
        data[i] = (int16_t) ((uint16_t)b2 << 8) | b3;
    }   
    PORTA.OUTSET |= PIN2_bm | PIN7_bm;    
}

void spi0_fetch_bytes(uint8_t bytes[], uint8_t n, uint32_t addr)
// bytes : array to put the incoming data
// n     : number of bytes to fetch
// addr  : starting byte address within the SRAM chip
{
    uint8_t b1, b2;
    if (n == 0) return; // Nothing to do.
    // We select which SRAM chip using bit17 of the provided address.
    if (addr & 0x00020000UL) {
        // Select second chip for 128k <= addr < 256k
        PORTA.OUTCLR |= PIN2_bm;
    } else {
        // Select first chip for 0k <= addr < 128k
        PORTA.OUTCLR |= PIN7_bm;
    }
    b1 = 0b11; // Read command.
    b2 = spi0_exchange(b1);
    b1 = (uint8_t) ((addr >> 16) & 0x01); // Upper byte, keep only bit16 of addr.
    b2 = spi0_exchange(b1);
    b1 = (uint8_t) (addr >> 8); // High byte.
    b2 = spi0_exchange(b1);
    b1 = (uint8_t) addr; // Low byte.
    b2 = spi0_exchange(b1);
    b1 = 0;
    for (uint8_t i=0; i < n; i++) {
        bytes[i] = spi0_exchange(b1);
    }   
    PORTA.OUTSET |= PIN7_bm | PIN2_bm;    
}
