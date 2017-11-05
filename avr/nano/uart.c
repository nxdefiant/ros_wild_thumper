#include <stdlib.h>
#include "uart.h"
#include <avr/interrupt.h>

#define UART_UBRR_CALC(BAUD_,FREQ_) ((FREQ_)/((BAUD_)*16L)-1)

static volatile	char read[RINGBUFFER_LEN];
static volatile	char *in_read_ptr;
static volatile	char *out_read_ptr;

ISR(USART_RX_vect) {
	// update read from uart
	if (UCSR0A & (1<<RXC0)) {
		*in_read_ptr = UDR0;

		if (in_read_ptr < read + RINGBUFFER_LEN - 1) in_read_ptr++;
		else in_read_ptr = (char *)read;
	}
}


void setup_uart(unsigned int rate) {
	UCSR0B = (1<<TXEN0) | (1<<RXEN0); // UART TX & RX
	UCSR0B |= (1<<RXCIE0); 		// RX Interrupt
	UCSR0C = (3<<UCSZ00); // Asynchron 8N1

	UBRR0H = (uint8_t)(UART_UBRR_CALC(rate, F_CPU) >> 8);
	UBRR0L = (uint8_t)UART_UBRR_CALC(rate, F_CPU);

	in_read_ptr = out_read_ptr = read;
}


void uart_putc(char *c) {
	while (!(UCSR0A & (1<<UDRE0)));
	UDR0 = *c;
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
	while (!(UCSR0A & (1<<RXC0)));
	return UDR0;
}


void uart_setup_stdout() {
	fdevopen(uart_putchar, uart_getchar);
}


void uart_stream_update(ringbuffer_t *buffer) {
	// update write to uart
	if (buffer->out_write_ptr != buffer->in_write_ptr) {
		if (UCSR0A & (1<<UDRE0)) {
			UDR0 = *buffer->out_write_ptr;
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

