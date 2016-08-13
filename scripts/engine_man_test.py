#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import sys
import struct
from time import sleep
from i2c import i2c_write_reg, i2c_read_reg


def set_pwm(left, right):
	i2c_write_reg(0x50, 0x1, struct.pack(">h", left))
	i2c_write_reg(0x50, 0x3, struct.pack(">h", left))
	i2c_write_reg(0x50, 0x5, struct.pack(">h", right))
	i2c_write_reg(0x50, 0x7, struct.pack(">h", right))

if __name__ == "__main__":
	i2c_write_reg(0x50, 0x90, struct.pack("BBBB", 1, 1, 0, 0)) # switch direction
	set_pwm(int(sys.argv[1]), int(sys.argv[2]))

	while True:
		motor1, = struct.unpack(">B", i2c_read_reg(0x50, 0x2, 1))
		motor2, = struct.unpack(">B", i2c_read_reg(0x50, 0x4, 1))
		motor3, = struct.unpack(">B", i2c_read_reg(0x50, 0x6, 1))
		motor4, = struct.unpack(">B", i2c_read_reg(0x50, 0x8, 1))
		speed1, speed2, speed3, speed4 = struct.unpack(">hhhh", i2c_read_reg(0x50, 0x30, 8))
		error,  = struct.unpack(">B", i2c_read_reg(0x50, 0xA1, 1))
		print "PWM:", motor1, motor2, motor3, motor4, "Speed:", speed1, speed2, speed3, speed4, "Error:", error
		sleep(0.1)
