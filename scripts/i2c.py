#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import threading
import inspect
import os
import logging
from ctypes import *
from time import sleep

DEBUG=0
logger = logging.getLogger(__name__)

class i2c:
	libc = CDLL("libc.so.6")
	I2C_SLAVE = 0x0703  # Use this slave address
	__single = None
	__lock = threading.Lock()
	__parent_owner = None

	def __init__(self, addr):
		with i2c.__lock:
			count = 0
			while(i2c.__single):
				parent = inspect.stack()[1][3]
				count += 1
				sleep(0.001)
			if DEBUG:
				if count > 10:
					parent_owner = "%s (%d), %s()" % (self.__parent_owner[1], self.__parent_owner[2], self.__parent_owner[3])
					logger.warning("Error: (%s) I2C blocked %fs by %s!", parent, count*0.001, parent_owner)
				i2c.__parent_owner = inspect.stack()[1]
			i2c.__single = True
		self.dev = i2c.libc.open("/dev/i2c-2", os.O_RDWR)
		if self.dev < 0:
			raise IOError("open")
		err = i2c.libc.ioctl(self.dev, i2c.I2C_SLAVE, addr>>1)
		if err < 0:
			raise IOError("ioctl")

	def write(self, s):
		num_write = i2c.libc.write(self.dev, s, len(s))
		if num_write != len(s):
			self.close()
			raise IOError("write: %d" % (num_write))
	
	def read(self, num):
		buf = create_string_buffer(num)
		num_read = i2c.libc.read(self.dev, buf, num)
		if num_read != num:
			self.close()
			raise IOError("read: %d" % (num_read))
		return buf.raw

	def close(self):
		if self.dev:
			i2c.libc.close(self.dev)
			self.dev = None
			#i2c.__parent_owner = None
			i2c.__single = None

	def __del__(self):
		self.close()

if __name__ == "__main__":
	import struct
	import sys

	dev = i2c(0x50)
	s = struct.pack(">Bh", int(sys.argv[1]), int(sys.argv[2]))
	dev.write(s)
	dev.close()
