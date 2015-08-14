#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import rospy
import tf
import struct
from i2c import *
from math import *
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

WHEEL_DIST = 0.248

class MoveBase:
	def __init__(self):
		rospy.init_node('wild_thumper_move_base')
		rospy.Subscriber("cmd_vel", Twist, self.cmdVelReceived)
		self.tf_broadcaster = tf.broadcaster.TransformBroadcaster()
		self.pub_odom = rospy.Publisher("odom", Odometry, queue_size=16)
		self.set_speed(0, 0)
		rospy.loginfo("Init done")
		i2c_write_reg(0x50, 0x90, struct.pack("BB", 1, 1)) # switch direction
		self.run()
	
	def run(self):
		rate = rospy.Rate(20.0)
		while not rospy.is_shutdown():
			#self.get_odom()
			#self.get_dist_forward()
			#self.get_dist_backward()
			#self.get_dist_left()
			#self.get_dist_right()
			rate.sleep()

	def get_odom(self):
		posx, posy, angle = struct.unpack(">fff", i2c_read_reg(0x50, 0x40, 12))
		speed_trans, speed_rot = struct.unpack(">ff", i2c_read_reg(0x50, 0x38, 8))
		current_time = rospy.Time.now()

		# since all odometry is 6DOF we'll need a quaternion created from yaw
		odom_quat = tf.transformations.quaternion_from_euler(0, 0, angle)

		# first, we'll publish the transform over tf
		self.tf_broadcaster.sendTransform((posx, posy, 0.0), odom_quat, current_time, "base_link", "odom")

		# next, we'll publish the odometry message over ROS
		odom = Odometry()
		odom.header.stamp = current_time
		odom.header.frame_id = "/odom"

		# set the position
		odom.pose.pose.position.x = posx
		odom.pose.pose.position.y = posy
		odom.pose.pose.position.z = 0.0
		odom.pose.pose.orientation.x = odom_quat[0]
		odom.pose.pose.orientation.y = odom_quat[1]
		odom.pose.pose.orientation.z = odom_quat[2]
		odom.pose.pose.orientation.w = odom_quat[3]

		# set the velocity
		odom.child_frame_id = "base_link"
		odom.twist.twist.linear.x = speed_trans
		odom.twist.twist.linear.y = 0.0
		odom.twist.twist.angular.z = speed_rot

		# publish the message
		self.pub_odom.publish(odom)

	
	def set_speed(self, trans, rot):
		i2c_write_reg(0x50, 0x50, struct.pack(">ff", trans, rot))

	def cmdVelReceived(self, msg):
		trans = msg.linear.x
		rot = msg.angular.z # rad/s
		self.set_speed(trans, rot)

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
