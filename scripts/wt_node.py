#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import rospy
import tf
import struct
import prctl
import spidev
from time import sleep
from i2c import i2c, i2c_write_reg, i2c_read_reg
from math import *
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Imu, Range
from wild_thumper.msg import LedStripe

WHEEL_DIST = 0.248

class LPD8806:
	def __init__(self, bus, device, num_leds):
		self.spi = spidev.SpiDev()
		self.spi.open(bus, device)
		self.spi.mode=0b00
		self.spi.max_speed_hz=int(2e6)
		self.num_leds = num_leds
		self.latch()
		self.l = [(0, 0, 0)] * num_leds
		self.update()
	
	def set(self, i, red=0, green=0, blue=0):
		if red > 127 or green > 127 or blue >> 127 or red < 0 or green < 0 or blue < 0:
			raise Exception("Bad RGB Value")
		self.l[i] = (red, green, blue)

	def latch(self):
		self.spi.writebytes([0x0 for i in range((self.num_leds+31)/32)])
	
	def update(self):
		l = []
		for i in range(self.num_leds):
			red, green, blue = self.l[i]
			l.append(0x80 | green)
			l.append(0x80 | red)
			l.append(0x80 | blue)
		self.spi.writebytes(l)
		self.latch()

class MoveBase:
	def __init__(self):
		rospy.init_node('wild_thumper')
		prctl.set_name("wild_thumper")
		enable_odom_tf = rospy.get_param("~enable_odom_tf", True)
		if enable_odom_tf:
			self.tf_broadcaster = tf.broadcaster.TransformBroadcaster()
		else:
			self.tf_broadcaster = None
		self.pub_odom = rospy.Publisher("odom", Odometry, queue_size=16)
		self.pub_diag = rospy.Publisher("diagnostics", DiagnosticArray, queue_size=16)
		self.pub_range_fwd = rospy.Publisher("range_forward", Range, queue_size=16)
		self.pub_range_bwd = rospy.Publisher("range_backward", Range, queue_size=16)
		self.pub_range_left = rospy.Publisher("range_left", Range, queue_size=16)
		self.pub_range_right = rospy.Publisher("range_right", Range, queue_size=16)
		self.cmd_vel = None
		self.set_speed(0, 0)
		rospy.loginfo("Init done")
		self.handicap_last = (-1, -1)
		self.pStripe = LPD8806(1, 0, 12)
		rospy.Subscriber("cmd_vel_out", Twist, self.cmdVelReceived)
		rospy.Subscriber("led_stripe", LedStripe, self.led_stripe_received)
		self.run()
	
	def run(self):
		rate = rospy.Rate(20.0)
		sleep(3) # wait 3s for ros to register and establish all subscriber connections before sending reset diag
		reset_val = self.get_reset()
		rospy.loginfo("Reset Status: 0x%x" % reset_val)
		i = 0
		while not rospy.is_shutdown():
			#print struct.unpack(">B", i2c_read_reg(0x50, 0xA2, 1))[0] # count test
			self.get_tle_err()
			self.get_odom()
			self.get_voltage()
			if i % 2:
				self.get_dist_forward()
				self.get_dist_left()
			else:
				self.get_dist_backward()
				self.get_dist_right()
			i+=1
			if self.cmd_vel != None:
				self.set_speed(self.cmd_vel[0], self.cmd_vel[1])
				self.cmd_vel = None
			rate.sleep()

	def get_reset(self):
		reset = struct.unpack(">B", i2c_read_reg(0x50, 0xA0, 1))[0]

		msg = DiagnosticArray()
		msg.header.stamp = rospy.Time.now()
		stat = DiagnosticStatus()
		stat.name = "Reset reason"
		stat.level = DiagnosticStatus.ERROR if reset & 0x0c else DiagnosticStatus.OK
		stat.message = "0x%02x" % reset

		wdrf = bool(reset & (1 << 3))
		if wdrf: rospy.loginfo("Watchdog Reset")
		borf = bool(reset & (1 << 2))
		if borf: rospy.loginfo("Brown-out Reset Flag")
		extrf = bool(reset & (1 << 1))
		if extrf: rospy.loginfo("External Reset Flag")
		porf = bool(reset & (1 << 0))
		if porf: rospy.loginfo("Power-on Reset Flag")
		stat.values.append(KeyValue("Watchdog Reset Flag", str(wdrf)))
		stat.values.append(KeyValue("Brown-out Reset Flag", str(borf)))
		stat.values.append(KeyValue("External Reset Flag", str(extrf)))
		stat.values.append(KeyValue("Power-on Reset Flag", str(porf)))

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
		speed_trans, speed_rot, posx, posy, angle = struct.unpack(">fffff", i2c_read_reg(0x50, 0x38, 20))
		current_time = rospy.Time.now()

		# since all odometry is 6DOF we'll need a quaternion created from yaw
		odom_quat = tf.transformations.quaternion_from_euler(0, 0, angle)

		# first, we'll publish the transform over tf
		if self.tf_broadcaster is not None:
			self.tf_broadcaster.sendTransform((posx, posy, 0.0), odom_quat, current_time, "base_footprint", "odom")

		# next, we'll publish the odometry message over ROS
		odom = Odometry()
		odom.header.stamp = current_time
		odom.header.frame_id = "odom"

		# set the position
		odom.pose.pose.position.x = posx
		odom.pose.pose.position.y = posy
		odom.pose.pose.position.z = 0.0
		odom.pose.pose.orientation.x = odom_quat[0]
		odom.pose.pose.orientation.y = odom_quat[1]
		odom.pose.pose.orientation.z = odom_quat[2]
		odom.pose.pose.orientation.w = odom_quat[3]
		odom.pose.covariance[0] = 1e-3 # x
		odom.pose.covariance[7] = 1e-3 # y
		odom.pose.covariance[14] = 1e6 # z
		odom.pose.covariance[21] = 1e6 # rotation about X axis
		odom.pose.covariance[28] = 1e6 # rotation about Y axis
		odom.pose.covariance[35] = 0.03 # rotation about Z axis

		# set the velocity
		odom.child_frame_id = "base_footprint"
		odom.twist.twist.linear.x = speed_trans
		odom.twist.twist.linear.y = 0.0
		odom.twist.twist.angular.z = speed_rot
		odom.twist.covariance[0] = 1e-3 # x
		odom.twist.covariance[7] = 1e-3 # y
		odom.twist.covariance[14] = 1e6 # z
		odom.twist.covariance[21] = 1e6 # rotation about X axis
		odom.twist.covariance[28] = 1e6 # rotation about Y axis
		odom.twist.covariance[35] = 0.03 # rotation about Z axis

		# publish the message
		self.pub_odom.publish(odom)

	
	def set_speed(self, trans, rot):
		i2c_write_reg(0x50, 0x50, struct.pack(">ff", trans, rot))

	def cmdVelReceived(self, msg):
		self.cmd_vel = (msg.linear.x, msg.angular.z) # commit speed on next update cycle

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
		return val
	
	def start_dist_srf(self, num):
		dev = i2c(0x52)
		s = struct.pack("B", num)
		dev.write(s)
		dev.close()

	def read_dist_srf(self, num):
		return struct.unpack(">H", i2c_read_reg(0x52, num, 2))[0]/1000.0

	def send_range(self, pub, frame_id, typ, dist, min_range, max_range, fov_deg):
		msg = Range()
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = frame_id
		msg.radiation_type = typ
		msg.field_of_view = fov_deg*pi/180
		msg.min_range = min_range
		msg.max_range = max_range
		msg.range = dist
		pub.publish(msg)

	def get_dist_left(self):
		if self.pub_range_left.get_num_connections() > 0:
			dist = self.get_dist_ir(0x1)
			if dist > -67:
				self.send_range(self.pub_range_left, "ir_left", Range.INFRARED, 30.553/(dist - -67.534), 0.04, 0.3, 1)

	def get_dist_right(self):
		if self.pub_range_right.get_num_connections() > 0:
			dist = self.get_dist_ir(0x3)
			if dist > 69:
				self.send_range(self.pub_range_right, "ir_right", Range.INFRARED, 17.4/(dist - 69), 0.04, 0.3, 1)

	def get_dist_forward(self):
		if self.pub_range_fwd.get_num_connections() > 0:
			dist = self.read_dist_srf(0x15)
			self.send_range(self.pub_range_fwd, "sonar_forward", Range.ULTRASOUND, dist, 0.04, 6, 40)
			self.start_dist_srf(0x5) # get next value

	def get_dist_backward(self):
		if self.pub_range_bwd.get_num_connections() > 0:
			dist = self.read_dist_srf(0x17)
			self.send_range(self.pub_range_bwd, "sonar_backward", Range.ULTRASOUND, dist, 0.04, 6, 40)
			self.start_dist_srf(0x7) # get next value
	
	def led_stripe_received(self, msg):
		for led in msg.leds:
			self.pStripe.set(led.num, red=led.red, green=led.green, blue=led.blue)
			self.pStripe.update()
		

if __name__ == "__main__":
	MoveBase()
