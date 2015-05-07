#include <stdlib.h>
#include "uart.h"
#include <avr/interrupt.h>

#define UART_UBRR_CALC(BAUD_,FREQ_) ((FREQ_)/((BAUD_)*16L)-1)

static volatile	char read[RINGBUFFER_LEN];
static volatile	char *in_read_ptr;
static volatile	char *out_read_ptr;

ISR(USART_RXC_vect) {
	// update read from uart
	if (UCSRA & (1<<RXC)) {
		*in_read_ptr = UDR;

		if (in_read_ptr < read + RINGBUFFER_LEN - 1) in_read_ptr++;
		else in_read_ptr = (char *)read;
	}
}


void setup_uart(unsigned int rate) {
	UCSRB |= (1<<TXEN) | (1<<RXEN); // UART TX & RX
	UCSRB |= (1<<RXCIE); 		// RX Interrupt
	UCSRC |= (3<<UCSZ0); // Asynchron 8N1

	UBRRH = (uint8_t)(UART_UBRR_CALC(rate, F_CPU) >> 8);
	UBRRL = (uint8_t)UART_UBRR_CALC(rate, F_CPU);

	in_read_ptr = out_read_ptr = read;
}


void uart_putc(char *c) {
	while (!(UCSRA & (1<<UDRE)));
	UDR = *c;
}


int uart_putchar(char c, FILE *stream)
{
	uart_putc(&c);
	return 0;
}


void uart_puts(char *s) {
	char *c;

	for (c = s; *c != '\0'; c++)
		uart_putc(c);
}


int uart_getchar(FILE *stream)
{
	return uart_getc();
}


int uart_getc() {
	while (!(UCSRA & (1<<RXC)));
	return UDR;
}


void uart_setup_stdout() {
	fdevopen(uart_putchar, uart_getchar);
}


void uart_stream_update(ringbuffer_t *buffer) {
	// update write to uart
	if (buffer->out_write_ptr != buffer->in_write_ptr) {
		if (UCSRA & (1<<UDRE)) {
			UDR = *buffer->out_write_ptr;
			if (buffer->out_write_ptr < buffer->write + RINGBUFFER_LEN - 1) buffer->out_write_ptr++;
			else buffer->out_write_ptr = buffer->write;
		}
	}

	// update read from uart	
	if (in_read_ptr != out_read_ptr) {
		*buffer->in_read_ptr = *out_read_ptr;

		// update newline chars
		if (buffer->block_read == *buffer->in_read_ptr) buffer->newlines++;

		if (buffer->in_read_ptr < buffer->read + RINGBUFFER_LEN - 1) buffer->in_read_ptr++;
		else buffer->in_read_ptr = buffer->read;

		// move pointer
		if (out_read_ptr < read + RINGBUFFER_LEN - 1) out_read_ptr++;
		else out_read_ptr = read;
	}
}

