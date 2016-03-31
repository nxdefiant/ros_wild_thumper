#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import struct
from time import sleep
from i2c import i2c_write_reg, i2c_read_reg


def set_pwm(val):
	i2c_write_reg(0x50, 0x1, struct.pack(">H", val))
	i2c_write_reg(0x50, 0x3, struct.pack(">H", val))
	i2c_write_reg(0x50, 0x5, struct.pack(">H", val))
	i2c_write_reg(0x50, 0x7, struct.pack(">H", val))

if __name__ == "__main__":
	set_pwm(210)
	sleep(3)
	set_pwm(0)
