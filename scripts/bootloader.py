#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import struct
import sys
import socket
from optparse import OptionParser
from time import sleep, time
from i2c import i2c

CMD_READ = 0x1
CMD_ERASE = 0x2
CMD_WRITE = 0x3
CMD_ERASE_ALL = 0x5
CMD_JUMP = 0x6
CMD_INFO = 0x99

PAGESIZE=64

class bootloader:
	def __init__(self, addr):
		self.i2c_addr = addr	
		self.boot_addr = 0x0

		if not self.identify():
			raise "Bootloader not running"
	
	def read_mem(self, addr, num):
		self.run_cmd(CMD_READ, addr, num)
		return self.read(num)

	def erase(self, addr):
		self.run_cmd(CMD_ERASE)
	
	def erase_all(self):
		self.run_cmd(CMD_ERASE_ALL)

	def __compare_memarea(self, addr, data):
			mem_cmp = self.read_mem(addr, 64)
			if mem_cmp != data:
				print "Expected:", data.encode("hex")
				print "Got:     ", mem_cmp.encode("hex")
				raise("Compare mismatch at 0x%x" % addr)
			return
	
	def __program_memarea(self, addr, data):
		self.run_cmd(CMD_WRITE, addr, 64, data)

	def compare(self, filename):
		return self.__process_hex(filename, self.__compare_memarea)
	
	def program(self, filename):
		return self.__process_hex(filename, self.__program_memarea)

	def __process_hex(self, filename, handle):
		next_addr = None
		buf = ""
		lFirstRow = None

		f = open(filename, "r")
		count=0
		for line in f:
			if line[0] != ':':
				raise("Bad line start character")
			hex = line[1:].replace("\r\n", "")
			data = hex.decode("hex")
			num = ord(data[0])
			chksum = 0
			for c in data:
				chksum+=ord(c)
			if chksum % 256 != 0:
				raise("Checksum error")
			addr, typ, data, chksum = struct.unpack(">HB%ssB" % num, data[1:])

			if typ == 0: # Data Record
				count+=len(data)
				if next_addr is not None:
					if next_addr != addr:
						raise "Gap in file"
				buf_addr = addr-len(buf)
				buf+=data
				if len(buf) >= PAGESIZE:
					if not lFirstRow:
						# do the first as last one
						lFirstRow = (buf_addr, buf[:PAGESIZE])
					else:
						print "Addr 0x%x" % buf_addr
						handle(buf_addr, buf[:PAGESIZE])
					buf = buf[PAGESIZE:]
			elif typ == 3: # Start Segment Address Record
				self.boot_addr = int(data.encode("hex"), 16)
			elif typ == 1: # End of File Record
				print "Addr (rest) 0x%x" % buf_addr
				buf_addr+=PAGESIZE
				diff = PAGESIZE-len(buf)
				buf+=chr(0xff)*diff # fill with 0xff
				handle(buf_addr, buf[:PAGESIZE])
				if lFirstRow: # was first
					buf_addr = lFirstRow[0]
					buf = lFirstRow[1]
					print "Addr (First) 0x%x" % buf_addr
					handle(buf_addr, buf)
			else:
				raise("Unknown type %d" % typ)
			
			next_addr = addr+num
		print "Byte count:", count
		f.close()

	def jump(self, addr):
		self.run_cmd(CMD_JUMP)

	def wait_ping(self):
		while(True):
			try:
				self.identify()
				break
			except:
				sleep(1)

	def load(self, filename):
		print "Erase..."
		self.erase_all()
		self.wait_ping()
		print "Erase Done."
		print "Program..."
		t1 = time()
		self.program(filename)
		print "Time: %.1fs" % (time() - t1)
		print "Compare..."
		t1 = time()
		self.compare(filename)
		print "Time: %.1fs" % (time() - t1)
		print "Jump:"
		self.jump(self.boot_addr)

	def write(self, s):
		dev = i2c(self.i2c_addr)
		dev.write(s)
		dev.close()

	def read(self, num):
		dev = i2c(self.i2c_addr)
		s = dev.read(num)
		dev.close()
		return s
	
	def run_cmd(self, cmd, addr=0x0, num=0, data=""):
		length = len(data)
		s1 = struct.pack("<BLB%ds" % (length), cmd, addr, num, data)
		self.write(s1)
		s2 = struct.pack("B", 0xff)
		self.write(s2)

	def identify(self):
		self.run_cmd(CMD_INFO)
		s = self.read(10)
		return s == "Bootloader"

def to_bootloader(addr):
	dev = i2c(addr)
	s = struct.pack("B", 0xff)
	dev.write(s)
	dev.close()

if __name__ == "__main__":
	usage = "usage: %prog [options] addr [ihex]"
	parser = OptionParser(usage=usage)
	parser.add_option("-b", "--start-bootloader", action="store_true", dest="bToBoot", default=False, help="Start Bootloader")

	(options, args) = parser.parse_args()
	if len(args) > 1:
		addr = int(args[0], 16)
		if options.bToBoot:
			to_bootloader(addr)
			sleep(1)
		if len(args) > 1:
			loader = bootloader(addr)
			loader.load(args[1])
