// spi_sram.h
// PJ 2023-12-03

#ifndef SPI_SRAM_H
#define	SPI_SRAM_H

#include "global_defs.h"
#include <xc.h>
#include <stdint.h>

void spi0_init();
uint8_t spi0_exchange(uint8_t byte);
void spi0_close();
uint8_t detect_SRAM_chips();

void spi0_send_sample_data(int16_t data[], uint8_t n, uint32_t addr);
void spi0_fetch_sample_data(int16_t data[], uint8_t n, uint32_t addr);
void spi0_fetch_bytes(uint8_t bytes[], uint8_t n, uint32_t addr);

#endif
