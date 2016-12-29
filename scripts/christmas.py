#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import rospy
from random import *
from datetime import datetime
from wild_thumper.msg import *

max_val = 10
light = 0
ldr_thres = 8

def sensorReceived(msg):
	global light
	light = msg.light

if __name__ == "__main__":
	rospy.init_node('christmas')
	pub = rospy.Publisher('led_stripe', LedStripe, queue_size=10)
	rospy.Subscriber("/sensors", Sensor, sensorReceived)
	rate = rospy.Rate(2)
	val = max_val
	while not rospy.is_shutdown():
		if light <= ldr_thres or val != 0:
			now = datetime.now()
			if light <= ldr_thres and now.hour >= 18 and now.hour <= 22:
				val = max_val
			else:
				val = 0
			msg = LedStripe()
			msg.leds = [
					Led(4, randint(0, val), randint(0, val), randint(0, val)),
					Led(5, randint(0, val), randint(0, val), randint(0, val)),
					Led(6, randint(0, val), randint(0, val), randint(0, val)),
					Led(7, randint(0, val), randint(0, val), randint(0, val))
					]
			pub.publish(msg)
		rate.sleep()
