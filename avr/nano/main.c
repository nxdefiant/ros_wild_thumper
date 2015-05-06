#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "uart.h"

/*
 * I2C Register Map (8 Bit)
 * 0x00 Register select
 * 0x01 Distance left MSB
 * 0x02 Distance left LSB
 * 0x03 Distance right MSB
 * 0x04 Distance right LSB
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
	ADMUX |= (1<<REFS1) | (1<<REFS0); // interne Referenzspannung nutzen 

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


int main(void) {
	bootloader = 0x00;
	setup_uart(9600);
	uart_setup_stdout();

	// I2C
	TWAR = 0x52;
	TWI_RESET;

	printf("\r\nStart\r\n");

	set_sleep_mode(SLEEP_MODE_IDLE);
	sei();
	while(1) {
		switch(ireg) {
			case 1: // ir left
				dist_left = get_distance(0);
				break;
			case 3: // ir right
				dist_right = get_distance(1);
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

		sleep_mode();
	}

	return 0;
}
