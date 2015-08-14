#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import struct
from i2c import *
from datetime import datetime
from time import sleep

def get_pos():
	s = i2c_read_reg(0x50, 0x10, 8)
	hall1, hall2, hall3, hall4 = struct.unpack(">hhhh", s)
	return hall1, hall2, hall3, hall4

if __name__ == "__main__":
	i2c_write_reg(0x50, 0x90, struct.pack("BB", 1, 1)) # switch direction
	speed = 255
	i2c_write_reg(0x50, 0x1, struct.pack(">hhhh", speed, speed, speed, speed))
	start = datetime.now()
	for i in range(100):
		diff = datetime.now() - start
		status = get_pos()
		print "%d.%03d: %d %d %d %d" % ((diff.seconds, diff.microseconds/1000) + status)
		sleep(0.01)
	speed = 0
	i2c_write_reg(0x50, 0x1, struct.pack(">hhhh", speed, speed, speed, speed))
