#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include "uart.h"

/*
 * I2C Register Map (8 Bit)
 * 0x00 Register select
 * 0x01 Distance left MSB
 * 0x02 Distance left LSB
 * 0x03 Distance right MSB
 * 0x04 Distance right LSB
 * 0x05 Distance forward MSB
 * 0x06 Distance forward LSB
 * 0x07 Distance backward MSB
 * 0x08 Distance backward LSB
 * 0x09 Voltage MSB
 * 0x0A Voltage LSB
 *
 * 0xff Bootloader
 */


#define TWI_ACK		TWCR = (1<<TWEA) | (1<<TWINT) | (1<<TWEN) | (1<<TWIE)
#define TWI_RESET	TWCR &= ~((1 << TWSTO) | (1 << TWEN)); TWI_ACK
#define TWI_NAK		TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE)

static volatile uint8_t ireg=0;
static volatile uint8_t bootloader=0;
static volatile uint16_t dist_left=0;
static volatile uint16_t dist_right=0;
static volatile uint16_t dist_forward=0;
static volatile uint16_t dist_backward=0;
static volatile uint8_t start_dist_fwd=0;
static volatile uint8_t start_dist_bwd=0;
static volatile uint16_t voltage=0;

ISR(TWI_vect)
{
	static int16_t tmp16=0;

	switch (TWSR & 0xF8)
	{  
		case 0x60: // start write
			TWI_ACK;
			ireg = 0;
			break;
		case 0x80: // write
			switch(ireg) {
				case 0x00: // register select
					ireg = TWDR;

					if (ireg == 0x05) start_dist_fwd=1;
					if (ireg == 0x07) start_dist_bwd=1;

					ireg--; // because we do ireg++ below
					TWI_ACK;
					break;
				case 0xff: // bootloader
					bootloader = TWDR;
				default:
					TWI_NAK;
			}
			ireg++;
			break;
		case 0xA8: // start read
		case 0xB8: // read
			switch(ireg) {
				case 0x01: // Distance left MSB
					tmp16 = dist_left;
					TWDR = tmp16>>8;
					TWI_ACK;
					break;
				case 0x02: // Distance right LSB
					TWDR = tmp16;
					TWI_ACK;
					break;
				case 0x03: // Distance right MSB
					tmp16 = dist_right;
					TWDR = tmp16>>8;
					TWI_ACK;
					break;
				case 0x04: // Distance right LSB
					TWDR = tmp16;
					TWI_ACK;
					break;
				case 0x05: // Distance forward MSB
					tmp16 = dist_forward;
					TWDR = tmp16>>8;
					TWI_ACK;
					break;
				case 0x06: // Distance forward LSB
					TWDR = tmp16;
					TWI_ACK;
					break;
				case 0x07: // Distance backward MSB
					tmp16 = dist_backward;
					TWDR = tmp16>>8;
					TWI_ACK;
					break;
				case 0x08: // Distance backward LSB
					TWDR = tmp16;
					TWI_ACK;
					break;
				case 0x09: // Voltage MSB
					tmp16 = voltage;
					TWDR = tmp16>>8;
					TWI_ACK;
					break;
				case 0x0A: // Voltage LSB
					TWDR = tmp16;
					TWI_ACK;
					break;
				default:
					TWDR = 0;
					TWI_NAK;
			}
			ireg++;
			break;
		default:
			TWI_RESET;
	}
}


uint16_t ReadChannel(uint8_t mux) {
	uint8_t i;
	uint16_t result;

	ADCSRA = (1<<ADEN) | (1<<ADPS1) | (1<<ADPS0);    // Frequenzvorteiler 
	// setzen auf 8 (1) und ADC aktivieren (1)

	ADMUX = mux;                      // Kanal waehlen
	ADMUX |= (1<<REFS0);

	/* nach Aktivieren des ADC wird ein "Dummy-Readout" empfohlen, man liest
	   also einen Wert und verwirft diesen, um den ADC "warmlaufen zu lassen" */
	ADCSRA |= (1<<ADSC);              // eine ADC-Wandlung 
	while ( ADCSRA & (1<<ADSC)  ) {
		     // auf Abschluss der Konvertierung warten 
	}
	result = ADCW;  // ADCW muss einmal gelesen werden,
	// sonst wird Ergebnis der nächsten Wandlung
	// nicht übernommen.

	/* Eigentliche Messung - Mittelwert aus 5 aufeinanderfolgenden Wandlungen */
	result = 0;
	for( i=0; i<5; i++ )
	{
		ADCSRA |= (1<<ADSC);            // eine Wandlung "single conversion"
		while ( ADCSRA & (1<<ADSC) ) {
			   // auf Abschluss der Konvertierung warten
		}
		result += ADCW;		    // Wandlungsergebnisse aufaddieren
	}

	ADCSRA &= ~(1<<ADEN);             // ADC deaktivieren (2)

	result /= 5;                     // Summe durch 5 teilen = arithm. Mittelwert
	
	return result;
}


static unsigned short get_distance(uint8_t i) {
	return ReadChannel(i);
}


static unsigned short get_voltage(void) {
	return ReadChannel(2)*1.46;
}


ISR(INT0_vect) {
	static uint16_t t_start=0;
	uint16_t t_now = TCNT1;
	uint16_t t_diff;

	if (bit_is_set(PIND, 2)) { // high level
		// start timer
		t_start = t_now;
	} else {
		t_diff = t_now - t_start;
		dist_forward = t_diff*2.7586 + 0.5; // t [µs] / 580 = mm
		// disable this interrupt
		EIMSK |= (1 << INT0);
	}
}


ISR(INT1_vect) {
	static uint16_t t_start=0;
	uint16_t t_now = TCNT1;
	uint16_t t_diff;

	if (bit_is_set(PIND, 3)) { // high level
		// start timer
		t_start = t_now;
	} else {
		t_diff = t_now - t_start;
		dist_backward = t_diff*2.7586 + 0.5; // t [µs] / 580 = mm
		// disable this interrupt
		EIMSK |= (1 << INT1);
	}
}


int main(void) {
	bootloader = 0x00;
	setup_uart(9600);
	uart_setup_stdout();

	// I2C
	TWAR = 0x52;
	TWI_RESET;

	// Timer 1: Normal mode, Top: 0xffff, Prescaler: F_CPU/256=62500Hz
	TCCR1A = 0x0;
	TCCR1B = (1 << CS12);

	// External Interrupts
	EICRA = (1 << ISC10) | (1 << ISC00);

	printf("\r\nStart\r\n");

	set_sleep_mode(SLEEP_MODE_IDLE);
	sei();
	while(1) {
		switch(ireg) {
			case 0x01: // ir left
				dist_left = get_distance(0);
				break;
			case 0x03: // ir right
				dist_right = get_distance(1);
				break;
			case 0x09: // voltage
				voltage = get_voltage();
				break;
			case 0xff: // Magic reg that starts the bootloader
				if (bootloader == 0xa5) {
					cli();
					{
						void (*start)(void) = (void*)0x1800;
						start();
					}
				}
				break;
		}

		if (start_dist_fwd) {
			start_dist_fwd = 0;
			dist_forward = 0;

			DDRD |= (1 << 2);
			PORTD |= (1 << 2);
			_delay_us(10);
			PORTD &= ~(1 << 2);
			DDRD &= ~(1 << 2);
			// wait for interrupt
			EIMSK |= (1 << INT0);
		}
		if (start_dist_bwd) {
			start_dist_bwd = 0;
			dist_backward = 0;

			DDRD |= (1 << 3);
			PORTD |= (1 << 3);
			_delay_us(10);
			PORTD &= ~(1 << 3);
			DDRD &= ~(1 << 3);
			// wait for interrupt
			EIMSK |= (1 << INT1);
		}

		sleep_mode();
	}

	return 0;
}
