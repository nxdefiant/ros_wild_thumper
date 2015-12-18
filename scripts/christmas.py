#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import rospy
from random import *
from wild_thumper.msg import LedStripe, Led

max_val = 10

if __name__ == "__main__":
	rospy.init_node('christmas')
	pub = rospy.Publisher('led_stripe', LedStripe, queue_size=10)
	rate = rospy.Rate(2)
	while not rospy.is_shutdown():
		msg = LedStripe()
		msg.leds = [
				Led(4, randint(0, max_val), randint(0, max_val), randint(0, max_val)),
				Led(5, randint(0, max_val), randint(0, max_val), randint(0, max_val)),
				Led(6, randint(0, max_val), randint(0, max_val), randint(0, max_val)),
				Led(7, randint(0, max_val), randint(0, max_val), randint(0, max_val))
				]
		pub.publish(msg)
		rate.sleep()
