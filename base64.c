// base64.c
// Service functions for base64-encoding of bytes into packed strings.
//
// Peter J. 2026-11-20 Consolidate and generalize scattered code.

#include <stdint.h>
#include <string.h>
#include "base64.h"

// Encode n bytes into a base64-encoded string.
//
// Returns the number of characters in the string (not including
// terminating null character).
int bytes_to_base64(const uint8_t bytes[], int n, char *b64str, int max_m)
{
    // First, compute the expected number of base64 characters to be produced.
    // Every three bytes (24 bits combined) needs 4 base64 characters.
    int threes = n / 3;
    int remaining = n % 3;
    int m_expected = threes * 4;
    if (remaining) {
        m_expected += (remaining == 2) ? 3 : 2;
    }
    if (m_expected > max_m) {
        // Cannot fit result, so do nothing.
        b64str[0] = 0;
        return 0;
    }
    // Begin real work of encoding.
    int m = 0;
    // Start by working through sets of 3 bytes,
    // writing 4 base64 characters each set.
    for (int i3 = 0; i3 < threes; i3++) {
        uint8_t b0 = bytes[i3*3];
        uint8_t b1 = bytes[i3*3+1];
        uint8_t b2 = bytes[i3*3+2];
        uint8_t sixbits = (uint8_t) ((b0 & 0xfc) >> 2);
        b64str[m] = base64_alphabet[sixbits];
        m++;
        sixbits = (uint8_t) (((b0 & 0x03) << 4) | ((b1 & 0xf0) >> 4));
        b64str[m] = base64_alphabet[sixbits];
        m++;
        sixbits = (uint8_t) (((b1 & 0x0f) << 2) | ((b2 & 0xc0) >> 6));
        b64str[m] = base64_alphabet[sixbits];
        m++;
        sixbits = (uint8_t) (b2 & 0x3f);
        b64str[m] = base64_alphabet[sixbits];
        m++;
    }
    if (remaining == 2) {
        // Two bytes encode to 3 characters with the two trailing bits being zero.
        uint8_t b0 = bytes[threes*3];
        uint8_t b1 = bytes[threes*3+1];
        uint8_t sixbits = (uint8_t) (b0 >> 2);
        b64str[m] = base64_alphabet[sixbits];
        m++;
        sixbits = (uint8_t) (((b0 & 0x03) << 4) | ((b1 & 0xf0) >> 4));
        b64str[m] = base64_alphabet[sixbits];
        m++;
        sixbits = (uint8_t) ((b1 & 0x0f) << 2);
        b64str[m] = base64_alphabet[sixbits];
        m++;
    }
    if (remaining == 1) {
        // One byte encodes to 2 characters with the four trailing bits being zero.
        uint8_t b0 = bytes[threes*3];
        uint8_t sixbits = (uint8_t) (b0 >> 2);
        b64str[m] = base64_alphabet[sixbits];
        m++;
        sixbits = (uint8_t) ((b0 & 0x03) << 4);
        b64str[m] = base64_alphabet[sixbits];
        m++;
    }
    b64str[m] = 0; // Terminate the string with a null character.
    return m;
}

// Decode a base64-encoded (null-terminated) string into a byte array.
//
// Returns the number of bytes decoded.
int base64_to_bytes(char* b64str, uint8_t bytes[], int max_n)
{
    // Work out the expected number of bytes to decode.
    // Each 4 base64 characters (24 bits) results in 3 bytes.
    int m = (int)strlen(b64str);
    int fours = m / 4;
    int remaining = m % 4;
    int n_expected = fours * 3;
    if (remaining == 3) {
        n_expected += 2;
    } else if (remaining == 2) {
        n_expected += 1;
    }
    if (n_expected > max_n) {
        return 0;
    }
    // Begin real work of decoding.
    int n = 0;
    for (int i4 = 0; i4 < fours; i4++) {
        uint8_t sixb_0 = six_bits_from_char(b64str[i4*4]);
        uint8_t sixb_1 = six_bits_from_char(b64str[i4*4+1]);
        uint8_t sixb_2 = six_bits_from_char(b64str[i4*4+2]);
        uint8_t sixb_3 = six_bits_from_char(b64str[i4*4+3]);
        bytes[n] = (uint8_t)((sixb_0 << 2) | ((sixb_1 & 0x30) >> 4));
        n++;
        bytes[n] = (uint8_t)(((sixb_1 & 0x0f) << 4) | ((sixb_2 & 0x3c) >> 2));
        n++;
        bytes[n] = (uint8_t)(((sixb_2 & 0x03) << 6) | (sixb_3 & 0x3f));
        n++;
    }
    if (remaining == 3) {
        uint8_t sixb_0 = six_bits_from_char(b64str[fours*4]);
        uint8_t sixb_1 = six_bits_from_char(b64str[fours*4+1]);
        uint8_t sixb_2 = six_bits_from_char(b64str[fours*4+2]);
        bytes[n] = (uint8_t)((sixb_0 << 2) | ((sixb_1 & 0x30) >> 4));
        n++;
        bytes[n] = (uint8_t)(((sixb_1 & 0x0f) << 4) | ((sixb_2 & 0x3c) >> 2));
        n++;
    }
    if (remaining == 2) {
        uint8_t sixb_0 = six_bits_from_char(b64str[fours*4]);
        uint8_t sixb_1 = six_bits_from_char(b64str[fours*4+1]);
        bytes[n] = (uint8_t)((sixb_0 << 2) | ((sixb_1 & 0x30) >> 4));
        n++;
    }
    return n;
}

// This is effectively a look-up table,
// returning 6 bits of the byte to be assembled.
uint8_t six_bits_from_char(char ch) {
    switch (ch) {
    case 'A': return 0;
    case 'B': return 1;
    case 'C': return 2;
    case 'D': return 3;
    case 'E': return 4;
    case 'F': return 5;
    case 'G': return 6;
    case 'H': return 7;
    case 'I': return 8;
    case 'J': return 9;
    case 'K': return 10;
    case 'L': return 11;
    case 'M': return 12;
    case 'N': return 13;
    case 'O': return 14;
    case 'P': return 15;
    case 'Q': return 16;
    case 'R': return 17;
    case 'S': return 18;
    case 'T': return 19;
    case 'U': return 20;
    case 'V': return 21;
    case 'W': return 22;
    case 'X': return 23;
    case 'Y': return 24;
    case 'Z': return 25;
    case 'a': return 26;
    case 'b': return 27;
    case 'c': return 28;
    case 'd': return 29;
    case 'e': return 30;
    case 'f': return 31;
    case 'g': return 32;
    case 'h': return 33;
    case 'i': return 34;
    case 'j': return 35;
    case 'k': return 36;
    case 'l': return 37;
    case 'm': return 38;
    case 'n': return 39;
    case 'o': return 40;
    case 'p': return 41;
    case 'q': return 42;
    case 'r': return 43;
    case 's': return 44;
    case 't': return 45;
    case 'u': return 46;
    case 'v': return 47;
    case 'w': return 48;
    case 'x': return 49;
    case 'y': return 50;
    case 'z': return 51;
    case '0': return 52;
    case '1': return 53;
    case '2': return 54;
    case '3': return 55;
    case '4': return 56;
    case '5': return 57;
    case '6': return 58;
    case '7': return 59;
    case '8': return 60;
    case '9': return 61;
    case '+': return 62;
    case '/': return 63;
    }
    return 255;
}
