#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import rospy
import struct
import prctl
from datetime import datetime
from time import sleep
from pyshared.i2c import i2c
from pyshared.humidity import *
from wild_thumper.msg import Sensor

# Board warming offset
TEMP_ERROR = 0 # -5 # degree celsius
PRESSURE_ERROR = -2.5

"""
LDR:
val = [100 285 321 515 636 758 940 1023]
lux = [95  34  31  12  11   8    5    0]
"""

def get_i2c(addr):
	dev = i2c(addr)
	dev.write(struct.pack(">BB", 0x00, 0x0f))
	dev.close()

	dev = i2c(addr)
	dev.write(chr(0x00))
	msg = dev.read(11) # for unknown reason need to read one more byte
	dev.close()
	return msg[:10]


def get(addr=0x58):
	msg = get_i2c(addr)
	ldr, temp_mess, humidity_mess, pressure, co = struct.unpack(">hhhhh", msg)

	temp_mess/=10.0
	humidity_mess/=10.0
	lux = -3.9338e-07*ldr**3 +0.00083596*ldr**2 -0.58608*ldr +144.96
	pressure_v = pressure*2.56/1024
	pressure_kpa = P = (pressure_v/5 + 0.04) / 0.004 + PRESSURE_ERROR # datasheet

	# fix temperature/humidity
	if TEMP_ERROR:
		temp_real = temp_mess + TEMP_ERROR
		humidity_abs = calc_humidity_abs(temp_mess, humidity_mess)
		humidity_real = calc_humidity_rel(temp_real, humidity_abs)
	else:
		temp_real = temp_mess
		humidity_real = humidity_mess

	return lux, temp_real, humidity_real, pressure_kpa, co

class SensorBoard:
	def __init__(self):
		rospy.init_node('sensor_board')
		prctl.set_name("sensor_board")
		self.pub = rospy.Publisher("sensors", Sensor, queue_size=16)
		self.run()

	def run(self):
		rate = rospy.Rate(1.0)
		t_last_check = datetime.min
		ventilate = 0
		while not rospy.is_shutdown():
			if self.pub.get_num_connections() > 0:
				ldr, temp, humidity, pressure, co = get()
				if (datetime.now() - t_last_check).seconds > 900:
					ventilate = check_ventilate(temp, humidity)
					t_last_check = datetime.now()
				
				msg = Sensor()
				msg.header.stamp = rospy.Time.now()
				msg.light = ldr
				msg.temp = temp
				msg.humidity = humidity
				msg.pressure = pressure
				msg.co = co
				msg.ventilate = True if ventilate > 1.10 else False

				self.pub.publish(msg)

			rate.sleep()

if __name__ == "__main__":
	SensorBoard()
