#ifndef UART_H
#define UART_H

#include <stdio.h>
#include "ringbuffer.h"

void setup_uart(unsigned int rate);
void uart_putc(char *c);
void uart_puts(char *s);
int uart_getc(void);
void uart_puti(int i);
int uart_putchar(char c, FILE *stream);
int uart_getchar(FILE *stream);
void uart_setup_stdout(void);
void uart_stream_update(ringbuffer_t *buffer);

#endif

