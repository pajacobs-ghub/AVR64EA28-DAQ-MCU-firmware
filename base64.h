// base64.h
// Service functions for base64-encoding of bytes into packed strings.
//
// Peter J. 2026-11-20 Consolidate scattered code.

#include <stdint.h>

#ifndef BASE64_H
#define BASE64_H

//   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15
static const char base64_alphabet[64] = {
	'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P',
	'Q','R','S','T','U','V','W','X','Y','Z','a','b','c','d','e','f',
	'g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v',
	'w','x','y','z','0','1','2','3','4','5','6','7','8','9','+','/'
};

int bytes_to_base64(const uint8_t bytes[], int n, char *b64str, int max_m);
int base64_to_bytes(char* b64str, uint8_t bytes[], int max_n);
uint8_t six_bits_from_char(char ch);

#endif
