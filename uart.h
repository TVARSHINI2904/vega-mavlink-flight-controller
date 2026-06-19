#ifndef UART_H
#define UART_H

#include <stdint.h>

#define UART_THR      (*(volatile uint8_t*)0x10000000)
#define UART_RHR      (*(volatile uint8_t*)0x10000000)
#define UART_LSR      (*(volatile uint8_t*)0x10000005)
#define UART_RX_READY (UART_LSR & 0x01)

void uart_putchar(char c);
void uart_print(const char *s);
void uart_print_int(uint32_t n);

#endif
