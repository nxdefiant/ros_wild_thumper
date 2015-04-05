#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import rospy
import struct
from i2c import i2c
from math import *
from geometry_msgs.msg import Twist

WHEEL_DIST = 0.248

class MoveBase:
	def __init__(self):
		rospy.init_node('wild_thumper_move_base')
		rospy.Subscriber("cmd_vel", Twist, self.cmdVelReceived)
		self.set_speed(0, 0)
		rospy.loginfo("Init done")
		self.run()
	
	def run(self):
		rate = rospy.Rate(10.0)
		while not rospy.is_shutdown():
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

		dev = i2c(0x56)
		s = struct.pack(">Bhh", 0x1, left, right)
		dev.write(s)
		dev.close()

	def cmdVelReceived(self, msg):
		trans = msg.linear.x
		rot = msg.angular.z # rad/s

		right = rot*pi*WHEEL_DIST + trans
		left = trans*2-right
		self.set_speed(left, right)


if __name__ == "__main__":
	MoveBase()
