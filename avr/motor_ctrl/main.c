#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <math.h>
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
 * 0x28 Left speed wish (m/s) MSB
 * 0x29 Left speed wish (m/s)
 * 0x2A Left speed wish (m/s)
 * 0x2B Left speed wish (m/s) LSB
 * 0x2C Right speed wish (m/s) MSB
 * 0x2D Right speed wish (m/s)
 * 0x2E Right speed wish (m/s)
 * 0x2F Right speed wish (m/s) LSB
 * 0x30 Motor 1 speed MSB
 * 0x31 Motor 1 speed LSB
 * 0x32 Motor 2 speed MSB
 * 0x33 Motor 2 speed LSB
 * 0x34 Motor 3 speed MSB
 * 0x35 Motor 3 speed LSB
 * 0x36 Motor 4 speed MSB
 * 0x37 Motor 4 speed LSB
 * 0x38 Speed (m/s) MSB
 * 0x39 Speed (m/s)
 * 0x3A Speed (m/s)
 * 0x3B Speed (m/s) LSB
 * 0x3C Angle (rad/s) MSB
 * 0x3D Angle (rad/s)
 * 0x3E Angle (rad/s)
 * 0x3F Angle (rad/s) LSB
 * free
 * 0x40 Position x (m) MSB
 * 0x41 Position x (m)
 * 0x42 Position x (m)
 * 0x43 Position x (m) LSB
 * 0x44 Position y (m) MSB
 * 0x45 Position y (m)
 * 0x46 Position y (m)
 * 0x47 Position y (m) LSB
 * 0x48 Position angle MSB
 * 0x49 Position angle
 * 0x4A Position angle
 * 0x4B Position angle LSB
 * free
 * 0x50 speed wish (m/s) MSB
 * 0x51 speed wish (m/s)
 * 0x52 speed wish (m/s)
 * 0x53 speed wish (m/s) LSB
 * 0x54 angle wish (rad/s) MSB
 * 0x55 angle wish (rad/s)
 * 0x56 angle wish (rad/s)
 * 0x57 angle wish (rad/s) LSB
 * free
 * 0x90 Motor 1 switch
 * 0x91 Motor 2 switch
 * 0x92 Motor 3 switch
 * 0x93 Motor 4 switch
 * 0x94 Front Handicap
 * 0x95 Aft Handicap
 * free
 * 0xA0 Reset reason
 * 0xA1 TLE Error status
 * 0xA2 count test
 * free
 * 0xff Bootloader
 */


#define TWI_ACK		TWCR = (1<<TWEA) | (1<<TWINT) | (1<<TWEN) | (1<<TWIE)
#define TWI_RESET	TWCR &= ~((1 << TWSTO) | (1 << TWEN)); TWI_ACK
#define TWI_NAK		TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE)

#define KP 0.009
#define KI 0.051429
#define KD 0.000378
#define PID_T 0.01
#define STEP_PER_M 3376.1 // wheel diameter=12cm, encoder=48cpr, gear ratio=1:34
#define WHEEL_DIST 0.252

enum mode {
	MOTOR_MANUAL,
	MOTOR_PID
};

typedef union {
	float f;
	uint32_t i;
} ufloat_t;

static volatile struct {
	float speed;
	float angle;
	uint8_t bUpdate;
} cmd_vel = {0, 0, 0};

static volatile uint8_t ireg=0;
static volatile uint8_t bootloader=0;
static volatile int16_t motor1=0; // -255..+255
static volatile int16_t motor2=0;
static volatile int16_t motor3=0;
static volatile int16_t motor4=0;
static volatile int16_t pos1=0; // step
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
static volatile int16_t speed1_wish=0; // step/s
static volatile int16_t speed2_wish=0;
static volatile int16_t speed3_wish=0;
static volatile int16_t speed4_wish=0;
static volatile uint8_t run_update=0;
static volatile int16_t speed1=0; // step/s
static volatile int16_t speed2=0;
static volatile int16_t speed3=0;
static volatile int16_t speed4=0;
static volatile ufloat_t pos_x={0.0};
static volatile ufloat_t pos_y={0.0};
static volatile ufloat_t angle={0.0};
static volatile float cur_speed_lin=0;
static volatile float cur_speed_rot=0;
static volatile uint8_t count_test=0;
static volatile uint8_t front_handicap=0;
static volatile uint8_t aft_handicap=0;

ISR(TWI_vect)
{
	static uint8_t tmp=0;
	static int16_t tmp16=0;
	static ufloat_t tmp_speed;
	static ufloat_t tmp_angle;

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
				case 0x28: // Left speed wish MSB
					tmp_speed.i = TWDR;
					TWI_ACK;
					break;
				case 0x29: // Left speed wish
					tmp_speed.i = tmp_speed.i << 8 | TWDR;
					TWI_ACK;
					break;
				case 0x2A: // Left speed wish
					tmp_speed.i = tmp_speed.i << 8 | TWDR;
					TWI_ACK;
					break;
				case 0x2B: // Left speed wish LSB
					tmp_speed.i = tmp_speed.i << 8 | TWDR;
					speed1_wish = tmp_speed.f*STEP_PER_M;
					speed2_wish = tmp_speed.f*STEP_PER_M;
					motor1_mode = MOTOR_PID;
					motor2_mode = MOTOR_PID;
					TWI_ACK;
					break;
				case 0x2C: // Right speed wish MSB
					tmp_speed.i = TWDR;
					TWI_ACK;
					break;
				case 0x2D: // Right speed wish
					tmp_speed.i = tmp_speed.i << 8 | TWDR;
					TWI_ACK;
					break;
				case 0x2E: // Right speed wish
					tmp_speed.i = tmp_speed.i << 8 | TWDR;
					TWI_ACK;
					break;
				case 0x2F: // Right speed wish LSB
					tmp_speed.i = tmp_speed.i << 8 | TWDR;
					speed1_wish = tmp_speed.f*STEP_PER_M;
					speed2_wish = tmp_speed.f*STEP_PER_M;
					motor1_mode = MOTOR_PID;
					motor2_mode = MOTOR_PID;
					TWI_ACK;
					break;
				case 0x50: // speed wish MSB
					tmp_speed.i = TWDR;
					TWI_ACK;
					break;
				case 0x51: // speed wish
					tmp_speed.i = tmp_speed.i << 8 | TWDR;
					TWI_ACK;
					break;
				case 0x52: // speed wish
					tmp_speed.i = tmp_speed.i << 8 | TWDR;
					TWI_ACK;
					break;
				case 0x53: // speed wish LSB
					tmp_speed.i = tmp_speed.i << 8 | TWDR;
					cmd_vel.speed = tmp_speed.f;
					TWI_ACK;
					break;
				case 0x54: // angle wish MSB
					tmp_angle.i = TWDR;
					TWI_ACK;
					break;
				case 0x55: // angle wish
					tmp_angle.i = tmp_angle.i << 8 | TWDR;
					TWI_ACK;
					break;
				case 0x56: // angle wish
					tmp_angle.i = tmp_angle.i << 8 | TWDR;
					TWI_ACK;
					break;
				case 0x57: // angle wish LSB
					tmp_angle.i = tmp_angle.i << 8 | TWDR;
					cmd_vel.angle = tmp_angle.f;
					cmd_vel.bUpdate = 1;
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
				case 0x94: // Front Handicap
					front_handicap = TWDR;
					TWI_ACK;
					break;
				case 0x95: // Aft Handicap
					aft_handicap = TWDR;
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
				case 0x38: // speed MSB
					tmp_speed.f = cur_speed_lin;
					TWDR = tmp_speed.i>>24;
					TWI_ACK;
					break;
				case 0x39: // speed
					TWDR = tmp_speed.i>>16;
					TWI_ACK;
					break;
				case 0x3A: // speed
					TWDR = tmp_speed.i>>8;
					TWI_ACK;
					break;
				case 0x3B: // speed LSB
					TWDR = tmp_speed.i;
					TWI_ACK;
					break;
				case 0x3C: // angle MSB
					tmp_angle.f = cur_speed_rot;
					TWDR = tmp_angle.i>>24;
					TWI_ACK;
					break;
				case 0x3D: // angle
					TWDR = tmp_angle.i>>16;
					TWI_ACK;
					break;
				case 0x3E: // angle
					TWDR = tmp_angle.i>>8;
					TWI_ACK;
					break;
				case 0x3F: // angle LSB
					TWDR = angle.i;
					TWI_ACK;
					break;
				case 0x40: // Position x MSB
					TWDR = pos_x.i>>24;
					TWI_ACK;
					break;
				case 0x41: // Position x
					TWDR = pos_x.i>>16;
					TWI_ACK;
					break;
				case 0x42: // Position x
					TWDR = pos_x.i>>8;
					TWI_ACK;
					break;
				case 0x43: // Position x LSB
					TWDR = pos_x.i;
					TWI_ACK;
					break;
				case 0x44: // Position y MSB
					TWDR = pos_y.i>>24;
					TWI_ACK;
					break;
				case 0x45: // Position y
					TWDR = pos_y.i>>16;
					TWI_ACK;
					break;
				case 0x46: // Position y
					TWDR = pos_y.i>>8;
					TWI_ACK;
					break;
				case 0x47: // Position y LSB
					TWDR = pos_y.i;
					TWI_ACK;
					break;
				case 0x48: // Position angle MSB
					TWDR = pos_y.i>>24;
					TWI_ACK;
					break;
				case 0x49: // Position angle
					TWDR = pos_y.i>>16;
					TWI_ACK;
					break;
				case 0x4A: // Position angle
					TWDR = pos_y.i>>8;
					TWI_ACK;
					break;
				case 0x4B: // Position angle LSB
					TWDR = pos_y.i;
					TWI_ACK;
					break;
				case 0xA0: // Reset reason
					TWDR = MCUCSR & 0x0f;
					MCUCSR = 0x0;
					TWI_ACK;
					break;
				case 0xA1: // TLE Error status
					TWDR = ~((PIND & 0x40)>>3 | (PINB & 0x07)) & 0xf;
					TWI_ACK;
					break;
				case 0xA2: // count test
					TWDR = count_test;
					TWI_ACK;
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


static void update_pos(void) {
	static int16_t pos1_last=0;
	static int16_t pos2_last=0;
	static int16_t pos3_last=0;
	static int16_t pos4_last=0;
	int16_t pos1_diff; // steps
	int16_t pos2_diff;
	int16_t pos3_diff;
	int16_t pos4_diff;
	float diff_left_m, diff_right_m, angle_diff, translation;
	float pos_x_new, pos_y_new, angle_new;
	int16_t speed_l, speed_r;
	float tmp_speed_lin, tmp_speed_rot;
	int16_t cur_pos1, cur_pos2, cur_pos3, cur_pos4;
	int16_t new_speed1, new_speed2, new_speed3, new_speed4;

	// copy to tmp
	cli();
	cur_pos1 = pos1;
	cur_pos2 = pos2;
	cur_pos3 = pos3;
	cur_pos4 = pos4;
	sei();

	pos1_diff = cur_pos1 - pos1_last;
	pos2_diff = cur_pos2 - pos2_last;
	pos3_diff = cur_pos3 - pos3_last;
	pos4_diff = cur_pos4 - pos4_last;

	new_speed1 = pos1_diff/PID_T;
	new_speed2 = pos2_diff/PID_T;
	new_speed3 = pos3_diff/PID_T;
	new_speed4 = pos4_diff/PID_T;

	diff_left_m = (pos1_diff + pos2_diff)/(2*STEP_PER_M);
	diff_right_m = (pos3_diff + pos4_diff)/(2*STEP_PER_M);
	angle_diff = (diff_right_m - diff_left_m) / WHEEL_DIST;

	angle_new = angle.f + angle_diff;
	if (angle_new > 2*M_PI) angle_new-=2*M_PI;
	else if (angle_new < 2*M_PI) angle_new+=2*M_PI;

	translation = (diff_left_m + diff_right_m)/2.0;
	pos_x_new = pos_x.f + cos(angle_new)*translation;
	pos_y_new = pos_y.f + sin(angle_new)*translation;

	speed_l = (new_speed1+new_speed2)/2;
	speed_r = (new_speed3+new_speed4)/2;
	tmp_speed_lin = (speed_l + speed_r)/(2.0*STEP_PER_M);
	tmp_speed_rot = (speed_r - speed_l)/(M_PI*WHEEL_DIST*STEP_PER_M);

	// copy from tmp
	cli();
	angle.f = angle_new;
	pos_x.f = pos_x_new;
	pos_y.f = pos_y_new;
	speed1 = new_speed1;
	speed2 = new_speed2;
	speed3 = new_speed3;
	speed4 = new_speed4;
	cur_speed_lin = tmp_speed_lin;
	cur_speed_rot = tmp_speed_rot;
	sei();

	pos1_last = cur_pos1;
	pos2_last = cur_pos2;
	pos3_last = cur_pos3;
	pos4_last = cur_pos4;
}


static void update_pid(void) {
	static int16_t eold1=0;
	static int16_t eold2=0;
	static int16_t eold3=0;
	static int16_t eold4=0;
	static int32_t esum1=0;
	static int32_t esum2=0;
	static int32_t esum3=0;
	static int32_t esum4=0;

	if (motor1_mode == MOTOR_PID) {
		if (speed1_wish == 0) {
			motor1 = 0;
			eold1 = 0;
			esum1 = 0;
		} else {
			int16_t e = speed1_wish - speed1;
			esum1+=e;
			motor1 += KP*e + KI*PID_T*esum1 + KD/PID_T*(e - eold1);
			eold1 = e;

                        if (motor1 > 255) motor1 = 255;
			else if (motor1 < -255) motor1 = -255;
		}
	}
	if (motor2_mode == MOTOR_PID) {
		if (speed2_wish == 0) {
			motor2 = 0;
			eold2 = 0;
			esum2 = 0;
		} else {
			int16_t e = speed2_wish - speed2;
			esum2+=e;
			motor2 += KP*e + KI*PID_T*esum2 + KD/PID_T*(e - eold2);
			eold2 = e;

                        if (motor2 > 255) motor2 = 255;
			else if (motor2 < -255) motor2 = -255;
		}
	}
	if (motor3_mode == MOTOR_PID) {
		if (speed3_wish == 0) {
			motor3 = 0;
			eold3 = 0;
			esum3 = 0;
		} else {
			int16_t e = speed3_wish - speed3;
			esum3+=e;
			motor3 += KP*e + KI*PID_T*esum3 + KD/PID_T*(e - eold3);
			eold3 = e;

                        if (motor3 > 255) motor3 = 255;
			else if (motor3 < -255) motor3 = -255;
		}
	}
	if (motor4_mode == MOTOR_PID) {
		if (speed4_wish == 0) {
			motor4 = 0;
			eold4 = 0;
			esum4 = 0;
		} else {
			int16_t e = speed4_wish - speed4;
			esum4+=e;
			motor4 += KP*e + KI*PID_T*esum4 + KD/PID_T*(e - eold4);
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
	
	run_update++;
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
	// Timer 2: Fast PWM inverting mode, Top=256
	// Prescaler=1
	TCCR2 = (1 << WGM21) | (1 << WGM20) | (1 << COM21) | (1 << COM20) | (1 << CS20);
	OCR2 = 0;

	// Motor 4
	// Timer 0: Fast PWM inverting mode, Top=256
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

		if (cmd_vel.bUpdate) {
			float speed_wish_right, speed_wish_left;
			float speed, angle;

			cli();
			speed = cmd_vel.speed;
			angle = cmd_vel.angle;
			cmd_vel.bUpdate = 0;
			sei();

			speed_wish_right = angle*M_PI*WHEEL_DIST/2 + speed;
			speed_wish_left = speed*2-speed_wish_right;

			speed_wish_left*=STEP_PER_M;
			speed_wish_right*=STEP_PER_M;

			speed1_wish = speed_wish_left * (100-aft_handicap)/100.0;
			speed2_wish = speed_wish_left * (100-front_handicap)/100.0;
			speed3_wish = speed_wish_right * (100-front_handicap)/100.0;
			speed4_wish = speed_wish_right * (100-aft_handicap)/100.0;
			motor1_mode = MOTOR_PID;
			motor2_mode = MOTOR_PID;
			motor3_mode = MOTOR_PID;
			motor4_mode = MOTOR_PID;
		}

		if (run_update >= 156) { // ~100Hz
			run_update=0;

			update_pos();
			update_pid();
			update_motor();
			count_test++;
		}

		sleep_mode();
	}

	return 0;
}
