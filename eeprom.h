// eeprom.h
// PJ 2023-12-03

#ifndef EEPROM_H

#include <xc.h>
#include <stdint.h>

void eeprom_write_byte(uint16_t addr, uint8_t data);
uint8_t eeprom_read_byte(uint16_t addr);

#define	EEPROM_H

#endif
