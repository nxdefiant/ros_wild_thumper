#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import sys
import struct
from time import sleep
from i2c import i2c_write_reg, i2c_read_reg


def set(trans, rot):
	i2c_write_reg(0x50, 0x50, struct.pack(">ff", trans, rot))

if __name__ == "__main__":
	set(float(sys.argv[1]), float(sys.argv[2]))
	while True:
		speed1, speed2, speed3, speed4 = struct.unpack(">hhhh", i2c_read_reg(0x50, 0x30, 8))
		print speed1, speed2, speed3, speed4
		sleep(0.1)
