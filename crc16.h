// crc16.h
// CRC-16 calculation using CCITT polynomial 0x11021.
// Peter J. 2026-04-10

#ifndef CRC16_H
#define CRC16_H

#include <stdint.h>

uint16_t crc16b(const uint8_t data[], uint16_t n, uint16_t remainder);
uint16_t crc16w(const uint16_t data[], uint16_t n, uint16_t remainder);

#endif
