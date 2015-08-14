#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import rospy
import struct
from i2c import *
from math import *
from geometry_msgs.msg import Twist

WHEEL_DIST = 0.248

class MoveBase:
	def __init__(self):
		rospy.init_node('wild_thumper_move_base')
		rospy.Subscriber("cmd_vel", Twist, self.cmdVelReceived)
		self.set_speed(0, 0)
		rospy.loginfo("Init done")
		i2c_write_reg(0x50, 0x90, struct.pack("BB", 1, 1)) # switch direction
		self.run()
	
	def run(self):
		rate = rospy.Rate(10.0)
		while not rospy.is_shutdown():
			#self.get_dist_forward()
			#self.get_dist_backward()
			#self.get_dist_left()
			#self.get_dist_right()
			rate.sleep()

	def set_speed(self, left, right):
		if left > 0: left+=80
		elif left < 0: left-=80
		if right > 0: right+=80
		elif right < 0: right-=80

		if left > 255: left=255
		elif left < -255: left=-255
		if right > 255: right=255
		elif right < -255: right=-255

		dev = i2c(0x50)
		s = struct.pack(">Bhhhh", 0x1, right, right, left, left)
		dev.write(s)
		dev.close()

	def cmdVelReceived(self, msg):
		trans = msg.linear.x
		rot = msg.angular.z # rad/s

		right = rot*pi*WHEEL_DIST + trans
		left = trans*2-right
		self.set_speed(left, right)

	# http://rn-wissen.de/wiki/index.php/Sensorarten#Sharp_GP2D12
	def get_dist_ir(self, num):
		dev = i2c(0x52)
		s = struct.pack("B", num)
		dev.write(s)
		dev.close()

		sleep(2e-6)

		dev = i2c(0x52)
		s = dev.read(2)
		dev.close()

		val = struct.unpack(">H", s)[0]
		return 15221/(val - -276.42)/100;
	
	def get_dist_srf(self, num):
		dev = i2c(0x52)
		s = struct.pack("B", num)
		dev.write(s)
		dev.close()

		sleep(50e-3)

		dev = i2c(0x52)
		s = dev.read(2)
		dev.close()

		return struct.unpack(">H", s)[0]/1000.0

	def get_dist_left(self):
		dist = self.get_dist_ir(0x1)

	def get_dist_right(self):
		 dist = self.get_dist_ir(0x3)

	def get_dist_forward(self):
		dist = self.get_dist_srf(0x5)

	def get_dist_backward(self):
		dist = self.get_dist_srf(0x7)


if __name__ == "__main__":
	MoveBase()
