#include "uart.h"

void uart_putchar(char c)
{
    while (!(UART_LSR & 0x20));
    UART_THR = c;
}

void uart_print(const char *s)
{
    while (*s) uart_putchar(*s++);
}

void uart_print_int(uint32_t n)
{
    char buf[12];
    int i = 0;
    if (n == 0) { uart_putchar('0'); return; }
    while (n) { buf[i++] = '0' + (n % 10); n /= 10; }
    while (i--) uart_putchar(buf[i]);
}
