// usart.h
// PJ, 2023-11-19 Looks like the functions we use for PIC MCUs.

#ifndef MY_USART
#define MY_USART
void usart0_init(long baud);
void usart0_putch(char data);
void usart0_putstr(char* str);
void usart0_flush_rx(void);
char usart0_getch(void);
char usart0_getche(void);
int usart0_getstr(char* buf, int nbuf);
void usart0_close(void);

#define XON 0x11
#define XOFF 0x13

#endif
