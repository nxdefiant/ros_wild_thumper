#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

from pyshared.i2c import *

if __name__ == "__main__":
	import struct
	import sys

	dev = i2c(0x50)
	s = struct.pack(">Bh", int(sys.argv[1]), int(sys.argv[2]))
	dev.write(s)
	dev.close()
