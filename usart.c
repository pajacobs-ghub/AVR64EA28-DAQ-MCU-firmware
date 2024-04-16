// usart.c
// Functions to provide a shim between the C standard library functions
// and USART0 peripheral device on the PIC16F1/PIC18FxxQ10 microcontroller.
// Follow the description given in Microchip Technical Brief 3216
// by Alexandru Niculae
// Getting Started with Universal Synchronous and Asynchronous
// Receiver and Transmitter.
//
// Pin Assignments
// 0.TXD == PA0
// 0.RXD == PA1
//
// PJ,
// 2023-11-19 Initial cut looks like the functions that we use for PIC MCUs.

#include <xc.h>
#include "global_defs.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

void usart0_init(long baud)
{
    unsigned int brg_value;
    // Asynchronous, normal mode (16 samples per bit)
    // with 8-bit data, no parity, 1 stop bit (CRTLC default)
    // Use floating-point arithmetic to effectively round to nearest int.
    brg_value = (uint16_t) (4.0*(float)F_CLK_PER/(float)baud + 0.5);
    USART0.BAUD = brg_value;
    // For 3.333333MHz, 115200 baud, expect value of  116
    //                    9600 baud                  1389
    USART0.CTRLB |= USART_TXEN_bm | USART_RXEN_bm;
    // Default pin locations for USART0
    // PA0 = 0.TXD
    // PA1 = 0.RXD
    return;
}

void usart0_putch(char data)
{
    // Wait until shift-register empty, then send data.
    while (!(USART0.STATUS & USART_DREIF_bm)) { /* CLRWDT(); */ }
    USART0.TXDATAL = data;
    return;
}

void usart0_putstr(char* str)
{
    for (size_t i=0; i < strlen(str); i++) usart0_putch(str[i]); 
    return;
}

void usart0_flush_rx(void)
{
    char c_discard;
    while (USART0.STATUS & USART_RXCIF_bm) { c_discard = USART0.RXDATAL; }
    return;
}

char usart0_getch(void)
{
    char c;
    // Block until a character is available in buffer.
    while (!(USART0.STATUS & USART_RXCIF_bm)) { /* CLRWDT(); */ }
    // Get the data that came in.
    c = USART0.RXDATAL;
    return c;
}

char usart0_getche(void)
{
    char data = usart0_getch();
    usart0_putch(data); // echo the character
    return data;
}

int usart0_getstr(char* buf, int nbuf)
// Read (without echo) a line of characters into the buffer,
// stopping when we see a return character.
// Returns the number of characters collected,
// excluding the terminating null char.
{
    int i = 0;
    char c;
    uint8_t done = 0;
    while (!done) {
        c = usart0_getch();
        if (c != '\n' && c != '\r' && c != '\b' && i < (nbuf-1)) {
            // Append a normal character.
            buf[i] = c;
            i++;
        }
        if (c == '\r') {
            // Stop collecting on receiving a carriage-return character.
            done = 1;
            buf[i] = '\0';
        }
        if (c == '\b' && i > 0) {
            // Backspace.
            i--;
        }
    }
    return i;
}

void usart0_close(void)
{
    USART0.CTRLB &= !(USART_TXEN_bm | USART_RXEN_bm);
    return;
}
