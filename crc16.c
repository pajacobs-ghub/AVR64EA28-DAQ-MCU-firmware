// CRC-16 calculation using CCITT polynomial 0x11021.
// Peter J. 2026-04-10
//
// Code modelled on that in
//   Hexmate User's Guide
//   Section 4.14: CRC Algorithms
//   Microchip Document DS-50003033C

#include <stdint.h>

// Process an array of bytes.
uint16_t crc16b(const uint8_t data[], uint16_t n, uint16_t remainder)
{
    for (uint16_t i = 0; i < n; i++) {
        remainder ^= ((uint16_t)data[i] << 8);
        for (uint8_t ibit = 8; ibit > 0; ibit--) {
            if (remainder & 0x8000) {
                remainder = (remainder << 1) ^ 0x1021;
            } else {
                remainder <<= 1;
            }
        }
    }
    return remainder;
}

// Process an array of 16-bit words.
uint16_t crc16w(const uint16_t data[], uint16_t n, uint16_t remainder)
{
    for (uint16_t i = 0; i < n; i++) {
        remainder ^= (uint16_t)data[i];
        for (uint8_t ibit = 16; ibit > 0; ibit--) {
            if (remainder & 0x8000) {
                remainder = (remainder << 1) ^ 0x1021;
            } else {
                remainder <<= 1;
            }
        }
    }
    return remainder;
}
