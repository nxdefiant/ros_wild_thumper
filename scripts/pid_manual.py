#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import sys
import struct
from time import sleep
from i2c import i2c_write_reg, i2c_read_reg

KP=0.06
KI=0.10
KD=0.0
PID_T=0.01
STEP_PER_M_AVG=5766.1
eold1 = 0.0
eold2 = 0.0
eold3 = 0.0
eold4 = 0.0
esum1 = 0.0
esum2 = 0.0
esum3 = 0.0
esum4 = 0.0
PWM_BREAK = -32768
speed1_wish=float(sys.argv[1])*STEP_PER_M_AVG
speed2_wish=float(sys.argv[1])*STEP_PER_M_AVG
speed3_wish=float(sys.argv[2])*STEP_PER_M_AVG
speed4_wish=float(sys.argv[2])*STEP_PER_M_AVG

def set_pwm(pwm1, pwm2, pwm3, pwm4):
	i2c_write_reg(0x50, 0x1, struct.pack(">h", pwm1))
	i2c_write_reg(0x50, 0x3, struct.pack(">h", pwm2))
	i2c_write_reg(0x50, 0x5, struct.pack(">h", pwm3))
	i2c_write_reg(0x50, 0x7, struct.pack(">h", pwm4))

if __name__ == "__main__":
	for i in range(200):
		speed1, speed2, speed3, speed4 = struct.unpack(">hhhh", i2c_read_reg(0x50, 0x30, 8))
		error,  = struct.unpack(">B", i2c_read_reg(0x50, 0xA1, 1))

		if speed1_wish == 0: 
			motor1 = 0.0
			eold1 = 0.0
			esum1 = 0.0
		else:
			e = speed1_wish - speed1
			esum1+=e
			motor1 = KP*e + KI*PID_T*esum1 + KD/PID_T*(e - eold1)
			eold1 = e

			if (motor1 < 0 and speed1_wish > 0): motor1 = PWM_BREAK	
			elif (motor1 > 0 and speed1_wish < 0): motor1 = PWM_BREAK
			elif (motor1 > 255): motor1 = 255
			elif (motor1 < -255): motor1 = -255

			print "Wish=", speed1_wish, "Speed=", speed1, "e=", e, "esum=", esum1, "pwm=", motor1, "error=", error

		if speed2_wish == 0: 
			motor2 = 0.0
			eold2 = 0.0
			esum2 = 0.0
		else:
			e = speed2_wish - speed2
			esum2+=e
			motor2 = KP*e + KI*PID_T*esum2 + KD/PID_T*(e - eold2)
			eold2 = e

			if (motor2 < 0 and speed2_wish > 0): motor2 = PWM_BREAK
			elif (motor2 > 0 and speed2_wish < 0): motor2 = PWM_BREAK
			elif (motor2 > 255): motor2 = 255
			elif (motor2 < -255): motor2 = -255

		if speed3_wish == 0: 
			motor3 = 0.0
			eold3 = 0.0
			esum3 = 0.0
		else:
			e = speed3_wish - speed3
			esum3+=e
			motor3 = KP*e + KI*PID_T*esum3 + KD/PID_T*(e - eold3)
			eold3 = e

			if (motor3 < 0 and speed3_wish > 0): motor3 = PWM_BREAK
			elif (motor3 > 0 and speed3_wish < 0): motor3 = PWM_BREAK
			elif (motor3 > 255): motor3 = 255
			elif (motor3 < -255): motor3 = -255

		if speed4_wish == 0: 
			motor4 = 0.0
			eold4 = 0.0
			esum4 = 0.0
		else:
			e = speed4_wish - speed4
			esum4+=e
			motor4 = KP*e + KI*PID_T*esum4 + KD/PID_T*(e - eold4)
			eold4 = e

			if (motor4 < 0 and speed4_wish > 0): motor4 = PWM_BREAK
			elif (motor4 > 0 and speed4_wish < 0): motor4 = PWM_BREAK
			elif (motor4 > 255): motor4 = 255
			elif (motor4 < -255): motor4 = -255

			#print "Wish=", speed4_wish, "Speed=", speed4, "e=", e, "esum=", esum4, "pwm=", motor4, "error=", error

		set_pwm(motor1, motor2, motor3, motor4)

		sleep(PID_T)
	set_pwm(0, 0, 0, 0)
