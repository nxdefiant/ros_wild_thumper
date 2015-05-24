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
 * 0x01 Motor 1 PWM MSB
 * 0x02 Motor 1 PWM LSB
 * 0x03 Motor 2 PWM MSB
 * 0x04 Motor 2 PWM LSB
 * 0x05 Motor 3 PWM MSB
 * 0x06 Motor 3 PWM LSB
 * 0x07 Motor 4 PWM MSB
 * 0x08 Motor 4 PWM LSB
 * free
 * 0x10 Hall 1 MSB
 * 0x11 Hall 1 LSB
 * 0x12 Hall 2 MSB
 * 0x13 Hall 2 LSB
 * 0x14 Hall 3 MSB
 * 0x15 Hall 3 LSB
 * 0x16 Hall 4 MSB
 * 0x17 Hall 4 LSB
 * free
 * 0x20 Motor 1 speed wish MSB
 * 0x21 Motor 1 speed wish LSB
 * 0x22 Motor 2 speed wish MSB
 * 0x23 Motor 2 speed wish LSB
 * 0x24 Motor 3 speed wish MSB
 * 0x25 Motor 3 speed wish LSB
 * 0x26 Motor 4 speed wish MSB
 * 0x27 Motor 4 speed wish LSB
 * free
 * 0x30 Motor 1 speed MSB
 * 0x31 Motor 1 speed LSB
 * 0x32 Motor 2 speed MSB
 * 0x33 Motor 2 speed LSB
 * 0x34 Motor 3 speed MSB
 * 0x35 Motor 3 speed LSB
 * 0x36 Motor 4 speed MSB
 * 0x37 Motor 4 speed LSB
 * free
 * 0x90 Motor 1 switch
 * 0x91 Motor 2 switch
 * 0x92 Motor 3 switch
 * 0x93 Motor 4 switch
 * free
 * 0xff Bootloader
 */


#define TWI_ACK		TWCR = (1<<TWEA) | (1<<TWINT) | (1<<TWEN) | (1<<TWIE)
#define TWI_RESET	TWCR &= ~((1 << TWSTO) | (1 << TWEN)); TWI_ACK
#define TWI_NAK		TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE)

#define KP 0.009
#define KI 0.051429
#define KD 0.000378
#define TIMER1_T 0.01

enum mode {
	MOTOR_MANUAL,
	MOTOR_PID
};

static volatile uint8_t ireg=0;
static volatile uint8_t bootloader=0;
static volatile int16_t motor1=0;
static volatile int16_t motor2=0;
static volatile int16_t motor3=0;
static volatile int16_t motor4=0;
static volatile int16_t pos1=0;
static volatile int16_t pos2=0;
static volatile int16_t pos3=0;
static volatile int16_t pos4=0;
static volatile enum mode motor1_mode=MOTOR_MANUAL;
static volatile enum mode motor2_mode=MOTOR_MANUAL;
static volatile enum mode motor3_mode=MOTOR_MANUAL;
static volatile enum mode motor4_mode=MOTOR_MANUAL;
static volatile uint8_t motor1_switch=0;
static volatile uint8_t motor2_switch=0;
static volatile uint8_t motor3_switch=0;
static volatile uint8_t motor4_switch=0;
static volatile int16_t speed1_wish=0;
static volatile int16_t speed2_wish=0;
static volatile int16_t speed3_wish=0;
static volatile int16_t speed4_wish=0;
static volatile uint8_t run_pid=0;
static int16_t speed1=0;
static int16_t speed2=0;
static int16_t speed3=0;
static int16_t speed4=0;

ISR(TWI_vect)
{
	static uint8_t tmp=0;
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
				case 0x01: // Motor 1 MSB
					tmp = TWDR;
					TWI_ACK;
					break;
				case 0x02: // Motor 1 LSB
					motor1 = tmp<<8 | TWDR;
					motor1_mode = MOTOR_MANUAL;
					TWI_ACK;
					break;
				case 0x03: // Motor 2 MSB
					tmp = TWDR;
					TWI_ACK;
					break;
				case 0x04: // Motor 2 LSB
					motor2 = tmp<<8 | TWDR;
					motor2_mode = MOTOR_MANUAL;
					TWI_ACK;
					break;
				case 0x05: // Motor 3 MSB
					tmp = TWDR;
					TWI_ACK;
					break;
				case 0x06: // Motor 3 LSB
					motor3 = tmp<<8 | TWDR;
					motor3_mode = MOTOR_MANUAL;
					TWI_ACK;
					break;
				case 0x07: // Motor 4 MSB
					tmp = TWDR;
					TWI_ACK;
					break;
				case 0x08: // Motor 4 LSB
					motor4 = tmp<<8 | TWDR;
					motor4_mode = MOTOR_MANUAL;
					TWI_ACK;
					break;
				case 0x20: // Motor 1 speed wish MSB
					tmp = TWDR;
					TWI_ACK;
					break;
				case 0x21: // Motor 1 speed wish LSB
					speed1_wish = tmp<<8 | TWDR;
					motor1_mode = MOTOR_PID;
					TWI_ACK;
					break;
				case 0x22: // Motor 2 speed wish MSB
					tmp = TWDR;
					TWI_ACK;
					break;
				case 0x23: // Motor 2 speed wish LSB
					speed2_wish = tmp<<8 | TWDR;
					motor2_mode = MOTOR_PID;
					TWI_ACK;
					break;
				case 0x24: // Motor 3 speed wish MSB
					tmp = TWDR;
					TWI_ACK;
					break;
				case 0x25: // Motor 3 speed wish LSB
					speed3_wish = tmp<<8 | TWDR;
					motor3_mode = MOTOR_PID;
					TWI_ACK;
					break;
				case 0x26: // Motor 4 speed wish MSB
					tmp = TWDR;
					TWI_ACK;
					break;
				case 0x27: // Motor 4 speed wish LSB
					speed4_wish = tmp<<8 | TWDR;
					motor4_mode = MOTOR_PID;
					TWI_ACK;
					break;
				case 0x90: // Motor 1 switch
					motor1_switch = TWDR;
					TWI_ACK;
					break;
				case 0x91: // Motor 2 switch
					motor2_switch = TWDR;
					TWI_ACK;
					break;
				case 0x92: // Motor 3 switch
					motor3_switch = TWDR;
					TWI_ACK;
					break;
				case 0x93: // Motor 4 switch
					motor4_switch = TWDR;
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
				case 0x02: // Motor 1 PWM
					TWDR = OCR1A;
					TWI_ACK;
					break;
				case 0x04: // Motor 2 PWM
					TWDR = OCR1B;
					TWI_ACK;
					break;
				case 0x06: // Motor 3 PWM
					TWDR = OCR2;
					TWI_ACK;
					break;
				case 0x08: // Motor 4 PWM
					TWDR = OCR0;
					TWI_ACK;
					break;
				case 0x10: // Hall 1 MSB
					tmp16 = pos1;
					TWDR = tmp16>>8;
					TWI_ACK;
					break;
				case 0x11: // Hall 1 LSB
					TWDR = tmp16;
					TWI_ACK;
					break;
				case 0x12: // Hall 2 MSB
					tmp16 = pos2;
					TWDR = tmp16>>8;
					TWI_ACK;
					break;
				case 0x13: // Hall 2 LSB
					TWDR = tmp16;
					TWI_ACK;
					break;
				case 0x14: // Hall 3 MSB
					tmp16 = pos3;
					TWDR = tmp16>>8;
					TWI_ACK;
					break;
				case 0x15: // Hall 3 LSB
					TWDR = tmp16;
					TWI_ACK;
					break;
				case 0x16: // Hall 4 MSB
					tmp16 = pos4;
					TWDR = tmp16>>8;
					TWI_ACK;
					break;
				case 0x17: // Hall 4 LSB
					TWDR = tmp16;
					TWI_ACK;
					break;
				case 0x20: // Motor 1 speed wish MSB
					TWDR = speed1_wish>>8;
					TWI_ACK;
					break;
				case 0x21: // Motor 1 speed wish LSB
					TWDR = speed1_wish;
					TWI_ACK;
					break;
				case 0x22: // Motor 2 speed wish MSB
					TWDR = speed2_wish>>8;
					TWI_ACK;
					break;
				case 0x23: // Motor 2 speed wish LSB
					TWDR = speed2_wish;
					TWI_ACK;
					break;
				case 0x24: // Motor 3 speed wish MSB
					TWDR = speed3_wish>>8;
					TWI_ACK;
					break;
				case 0x25: // Motor 3 speed wish LSB
					TWDR = speed3_wish;
					TWI_ACK;
					break;
				case 0x26: // Motor 4 speed wish MSB
					TWDR = speed4_wish>>8;
					TWI_ACK;
					break;
				case 0x27: // Motor 4 speed wish LSB
					TWDR = speed4_wish;
					TWI_ACK;
					break;
				case 0x30: // Motor 1 speed MSB
					TWDR = speed1>>8;
					TWI_ACK;
					break;
				case 0x31: // Motor 1 speed LSB
					TWDR = speed1;
					TWI_ACK;
					break;
				case 0x32: // Motor 2 speed MSB
					TWDR = speed2>>8;
					TWI_ACK;
					break;
				case 0x33: // Motor 2 speed LSB
					TWDR = speed2;
					TWI_ACK;
					break;
				case 0x34: // Motor 3 speed MSB
					TWDR = speed3>>8;
					TWI_ACK;
					break;
				case 0x35: // Motor 3 speed LSB
					TWDR = speed3;
					TWI_ACK;
					break;
				case 0x36: // Motor 4 speed MSB
					TWDR = speed4>>8;
					TWI_ACK;
					break;
				case 0x37: // Motor 4 speed LSB
					TWDR = speed4;
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


static void update_hall1(void) {
	unsigned char status = (PINA >> 0) & 0x3;
	static unsigned char oldstatus=0;
	unsigned char diff, new;

	new = 0;
	if (status & 0x1)
		new = 0x3;
	if (status & 0x2)
		new ^= 0x1;					// convert gray to binary
	diff = oldstatus - new;				// difference last - new
	if (diff & 0x1) {				// bit 0 = value (1)
		oldstatus = new;					// store new as next last
		if (motor1_switch) pos1 += (diff & 2) - 1;		// bit 1 = direction (+/-)
		else pos1 -= (diff & 2) - 1;
	}
}


static void update_hall2(void) {
	unsigned char status = (PINA >> 4) & 0x3;
	static unsigned char oldstatus=0;
	unsigned char diff, new;

	new = 0;
	if (status & 0x1)
		new = 0x3;
	if (status & 0x2)
		new ^= 0x1;					// convert gray to binary
	diff = oldstatus - new;				// difference last - new
	if (diff & 0x1) {				// bit 0 = value (1)
		oldstatus = new;					// store new as next last
		if (motor2_switch) pos2 -= (diff & 2) - 1;		// bit 1 = direction (+/-)
		else pos2 += (diff & 2) - 1;
	}
}


static void update_hall3(void) {
	unsigned char status = (PINA >> 2) & 0x3;
	static unsigned char oldstatus=0;
	unsigned char diff, new;

	new = 0;
	if (status & 0x1)
		new = 0x3;
	if (status & 0x2)
		new ^= 0x1;					// convert gray to binary
	diff = oldstatus - new;				// difference last - new
	if (diff & 0x1) {				// bit 0 = value (1)
		oldstatus = new;					// store new as next last
		if (motor3_switch) pos3 -= (diff & 2) - 1;		// bit 1 = direction (+/-)
		else pos3 += (diff & 2) - 1;
	}
}


static void update_hall4(void) {
	unsigned char status = (PINA >> 6) & 0x3;
	static unsigned char oldstatus=0;
	unsigned char diff, new;

	new = 0;
	if (status & 0x1)
		new = 0x3;
	if (status & 0x2)
		new ^= 0x1;					// convert gray to binary
	diff = oldstatus - new;				// difference last - new
	if (diff & 0x1) {				// bit 0 = value (1)
		oldstatus = new;					// store new as next last
		if (motor4_switch) pos4 += (diff & 2) - 1;		// bit 1 = direction (+/-)
		else pos4 -= (diff & 2) - 1;
	}
}


static void update_motor(void) {
	static int16_t m1_old=SHRT_MIN;
	static int16_t m2_old=SHRT_MIN;
	static int16_t m3_old=SHRT_MIN;
	static int16_t m4_old=SHRT_MIN;

	if (m1_old != motor1) { // update only when changed
		if (motor1 == 0) {
			// stop
			PORTC |= (1 << 3) | (1 << 2);
		} else if ((!motor1_switch && motor1 > 0) || (motor1_switch && motor1 < 0)) {
			// forward
			PORTC &= ~(1 << 3) & ~(1 << 2);
		} else { // motor1 < 0
			// backward
			PORTC &= ~(1 << 2);
			PORTC |=  (1 << 3);
		}

		m1_old = motor1;
		OCR1A = abs(motor1);
	}

	if (m2_old != motor2) { // update only when changed
		if (motor2 == 0) {
			// stop
			PORTC |= (1 << 5) | (1 << 4);
		} else if ((!motor2_switch && motor2 > 0) || (motor2_switch && motor2 < 0)) {
			// forward
			PORTC &= ~(1 << 5) & ~(1 << 4);
		} else { // motor2 < 0
			// backward
			PORTC &= ~(1 << 4);
			PORTC |=  (1 << 5);
		}

		m2_old = motor2;
		OCR1B = abs(motor2);
	}

	if (m3_old != motor3) { // update only when changed
		if (motor3 == 0) {
			// stop
			PORTC |= (1 << 7) | (1 << 6);
		} else if ((!motor3_switch && motor3 > 0) || (motor3_switch && motor3 < 0)) {
			// forward
			PORTC &= ~(1 << 7) & ~(1 << 6);
		} else { // motor3 < 0
			// backward
			PORTC &= ~(1 << 6);
			PORTC |=  (1 << 7);
		}

		m3_old = motor3;
		OCR2 = abs(motor3);
	}

	if (m4_old != motor4) { // update only when changed
		if (motor4 == 0) {
			// stop
			PORTD |= (1 << 3) | (1 << 2);
		} else if ((!motor4_switch && motor4 > 0) || (motor4_switch && motor4 < 0)) {
			// forward
			PORTD &= ~(1 << 3) & ~(1 << 2);
		} else { // motor4 < 0
			// backward
			PORTD &= ~(1 << 2);
			PORTD |=  (1 << 3);
		}

		m4_old = motor4;
		OCR0 = abs(motor4);
	}
}


void update_pid(void) {
	static int16_t pos1_last=0;
	static int16_t pos2_last=0;
	static int16_t pos3_last=0;
	static int16_t pos4_last=0;
	static int16_t eold1=0;
	static int16_t eold2=0;
	static int16_t eold3=0;
	static int16_t eold4=0;
	static int32_t esum1=0;
	static int32_t esum2=0;
	static int32_t esum3=0;
	static int32_t esum4=0;

	speed1 = (pos1 - pos1_last)/TIMER1_T;
	pos1_last = pos1;
	speed2 = (pos2 - pos2_last)/TIMER1_T;
	pos2_last = pos2;
	speed3 = (pos3 - pos3_last)/TIMER1_T;
	pos3_last = pos3;
	speed4 = (pos4 - pos4_last)/TIMER1_T;
	pos4_last = pos4;

	if (motor1_mode == MOTOR_PID) {
		if (speed1_wish == 0) {
			motor1 = 0;
		} else {
			int16_t e = speed1_wish - speed1;
			esum1+=e;
			motor1 += KP*e + KI*TIMER1_T*esum1 + KD/TIMER1_T*(e - eold1);
			eold1 = e;

                        if (motor1 > 255) motor1 = 255;
			else if (motor1 < -255) motor1 = -255;
		}
	}
	if (motor2_mode == MOTOR_PID) {
		if (speed2_wish == 0) {
			motor2 = 0;
		} else {
			int16_t e = speed2_wish - speed2;
			esum2+=e;
			motor2 += KP*e + KI*TIMER1_T*esum2 + KD/TIMER1_T*(e - eold2);
			eold2 = e;

                        if (motor2 > 255) motor2 = 255;
			else if (motor2 < -255) motor2 = -255;
		}
	}
	if (motor3_mode == MOTOR_PID) {
		if (speed3_wish == 0) {
			motor3 = 0;
		} else {
			int16_t e = speed3_wish - speed3;
			esum3+=e;
			motor3 += KP*e + KI*TIMER1_T*esum3 + KD/TIMER1_T*(e - eold3);
			eold3 = e;

                        if (motor3 > 255) motor3 = 255;
			else if (motor3 < -255) motor3 = -255;
		}
	}
	if (motor4_mode == MOTOR_PID) {
		if (speed4_wish == 0) {
			motor4 = 0;
		} else {
			int16_t e = speed4_wish - speed4;
			esum4+=e;
			motor4 += KP*e + KI*TIMER1_T*esum4 + KD/TIMER1_T*(e - eold4);
			eold4 = e;

                        if (motor4 > 255) motor4 = 255;
			else if (motor4 < -255) motor4 = -255;
		}
	}
}


ISR(TIMER1_OVF_vect) {
	update_hall1();
	update_hall2();
	update_hall3();
	update_hall4();
	
	run_pid++;
}


int main(void) {
	// Outputs
	DDRB = (1 << 3);
	DDRC = (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | (1 << 3) | (1 << 2);
	DDRD = (1 << 7) | (1 << 5) | (1 << 4) | (1 << 3) | (1 << 2);

	bootloader = 0x00;
	setup_uart(9600);
	uart_setup_stdout();

	// I2C
	TWAR = 0x50;
	TWI_RESET;

	// Motor 1 & 2
	// Timer 1: Fast PWM inverting mode, Top=256 => 15.625kHz
	// Prescaler=1
	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << COM1A0) | (1 << COM1B0) | (1 << WGM10);
	TCCR1B = (1 << WGM12) | (1 << CS10);
	OCR1A = 0;
	OCR1B = 0;

	// Motor 3
	// Timer 2: Fast PWM inverting mode, Top=256 => 15.625kHz
	// Prescaler=1
	TCCR2 = (1 << WGM21) | (1 << WGM20) | (1 << COM21) | (1 << COM20) | (1 << CS20);
	OCR2 = 0;

	// Motor 4
	// Timer 0: Fast PWM inverting mode, Top=256 => 15.625kHz
	// Prescaler=1
	TCCR0 = (1 << WGM01) | (1 << WGM00) | (1 << COM01) | (1 << COM00) | (1 << CS00);
	OCR0 = 0;

	printf("\r\nStart\r\n");

	set_sleep_mode(SLEEP_MODE_IDLE);
	// Enable Timer 1 Overflow Interrupt
	TIMSK = (1 << TOIE1);
	sei();

	while(1) {
		switch(ireg) {
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
		

		if (run_pid >= 156) { // ~100Hz
			run_pid=0;
			update_pid();
		}

		update_motor();

		sleep_mode();
	}

	return 0;
}
