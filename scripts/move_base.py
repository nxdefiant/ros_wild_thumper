#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import rospy
import tf
import struct
from i2c import *
from math import *
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Imu

WHEEL_DIST = 0.248

class MoveBase:
	def __init__(self):
		rospy.init_node('wild_thumper_move_base')
		rospy.Subscriber("cmd_vel", Twist, self.cmdVelReceived)
		rospy.Subscriber("imu", Imu, self.imuReceived)
		self.tf_broadcaster = tf.broadcaster.TransformBroadcaster()
		self.pub_odom = rospy.Publisher("odom", Odometry, queue_size=16)
		self.pub_diag = rospy.Publisher("diagnostics", DiagnosticArray, queue_size=16)
		self.set_speed(0, 0)
		rospy.loginfo("Init done")
		i2c_write_reg(0x50, 0x90, struct.pack("BB", 1, 1)) # switch direction
		self.run()
	
	def run(self):
		rate = rospy.Rate(20.0)
		reset_val = self.get_reset()
		rospy.loginfo("Reset Status: 0x%x" % reset_val)
		while not rospy.is_shutdown():
			#print struct.unpack(">B", i2c_read_reg(0x50, 0xA2, 1))[0] # count test
			self.get_tle_err()
			self.get_odom()
			self.get_voltage()
			#self.get_dist_forward()
			#self.get_dist_backward()
			#self.get_dist_left()
			#self.get_dist_right()
			rate.sleep()

	def set_motor_handicap(self, front, aft): # percent
		i2c_write_reg(0x50, 0x94, struct.pack(">bb", front, aft))

	def imuReceived(self, msg):
		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(msg.orientation.__getstate__())
		if pitch > 30*pi/180:
			val = (100/90)*abs(pitch)*180/pi
			print "aft handicap", val
			self.set_motor_handicap(0, val)
		elif pitch < -30*pi/180:
			val = (100/90)*abs(pitch)*180/pi
			print "front handicap", val
			self.set_motor_handicap(val, 0)
		else:
			self.set_motor_handicap(0, 0)

	def get_reset(self):
		reset = struct.unpack(">B", i2c_read_reg(0x50, 0xA0, 1))[0]

		msg = DiagnosticArray()
		msg.header.stamp = rospy.Time.now()
		stat = DiagnosticStatus()
		stat.name = "Reset reason"
		stat.level = DiagnosticStatus.ERROR if reset & 0x0c else DiagnosticStatus.OK
		stat.message = "0x%02x" % reset

		stat.values.append(KeyValue("Watchdog Reset Flag", str(bool(reset & (1 << 3)))))
		stat.values.append(KeyValue("Brown-out Reset Flag", str(bool(reset & (1 << 2)))))
		stat.values.append(KeyValue("External Reset Flag", str(bool(reset & (1 << 1)))))
		stat.values.append(KeyValue("Power-on Reset Flag", str(bool(reset & (1 << 0)))))

		msg.status.append(stat)
		self.pub_diag.publish(msg)
		return reset


	def get_tle_err(self):
		err = struct.unpack(">B", i2c_read_reg(0x50, 0xA1, 1))[0]
		
		msg = DiagnosticArray()
		msg.header.stamp = rospy.Time.now()
		stat = DiagnosticStatus()
		stat.name = "Motor: Error Status"
		stat.level = DiagnosticStatus.ERROR if err else DiagnosticStatus.OK
		stat.message = "0x%02x" % err

		stat.values.append(KeyValue("aft left", str(bool(err & (1 << 0)))))
		stat.values.append(KeyValue("front left", str(bool(err & (1 << 1)))))
		stat.values.append(KeyValue("front right", str(bool(err & (1 << 2)))))
		stat.values.append(KeyValue("aft right", str(bool(err & (1 << 3)))))

		msg.status.append(stat)
		self.pub_diag.publish(msg)
	
	def get_voltage(self):
		volt = struct.unpack(">h", i2c_read_reg(0x52, 0x09, 2))[0]/100.0

		msg = DiagnosticArray()
		msg.header.stamp = rospy.Time.now()
		stat = DiagnosticStatus()
		stat.name = "Voltage"
		stat.level = DiagnosticStatus.ERROR if volt < 7 else DiagnosticStatus.OK
		stat.message = "%.2fV" % volt

		msg.status.append(stat)
		self.pub_diag.publish(msg)


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
