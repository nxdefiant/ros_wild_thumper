#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

HOST = "localhost"
PORT = 4223
UID = "6DdNSn"

import rospy
import prctl
from sensor_msgs.msg import Imu
from tinkerforge.ip_connection import IPConnection
from tinkerforge.brick_imu_v2 import BrickIMUV2

class ImuBrickv2:
	def __init__(self):
		rospy.init_node('imu_brick_v2')
		prctl.set_name("imu_brick_v2")
		self.pub_imu = rospy.Publisher("imu", Imu, queue_size=16)

		ipcon = IPConnection() # Create IP connection
		imu = BrickIMUV2(UID, ipcon) # Create device object
		ipcon.connect(HOST, PORT) # Connect to brickd

		imu.leds_off()
		#imu.disable_status_led()

		imu.register_callback(imu.CALLBACK_ALL_DATA, self.cb_all_data)
		imu.set_all_data_period(20) # ms

		rospy.spin()
		ipcon.disconnect()


	def cb_all_data(self, acceleration, magnetic_field, angular_velocity, euler_angle, quaternion, linear_acceleration, gravity_vector, temperature, calibration_status):
		msg = Imu()
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = "base_imu_link"

		msg.orientation.x = quaternion[1]/16383.0
		msg.orientation.y = quaternion[2]/16383.0
		msg.orientation.z = quaternion[3]/16383.0
		msg.orientation.w = quaternion[0]/16383.0
		# Observed orientation variance: 0.0 (10k samples)
		# Magnometer heading accuracy is +-2.5 deg => 0.088 rad
		# With heading accuracy as std dev, variance = 0.088^2 = 0.008 
		msg.orientation_covariance = [
			0.008, 0     , 0,
			0    , 0.008, 0,
			0    , 0    , 0.008
		]

		msg.angular_velocity.x = angular_velocity[0]/16.0
		msg.angular_velocity.y = angular_velocity[1]/16.0
		msg.angular_velocity.z = angular_velocity[2]/16.0
		# Observed angular velocity variance: 0.006223 (10k samples), => round up to 0.02
		msg.angular_velocity_covariance = [
			0.02, 0   , 0,
			0   , 0.02, 0,
			0   , 0   , 0.02
		]

		msg.linear_acceleration.x = linear_acceleration[0]/100.0
		msg.linear_acceleration.y = linear_acceleration[1]/100.0
		msg.linear_acceleration.z = linear_acceleration[2]/100.0
		# Observed linear acceleration variance: 0.001532 (10k samples)
		# Calculation for variance taken from razor imu:
		# nonliniarity spec: 1% of full scale (+-2G) => 0.2m/s^2
		# Choosing 0.2 as std dev, variance = 0.2^2 = 0.04
		msg.linear_acceleration_covariance = [
			0.04, 0    , 0,
			0    , 0.04, 0,
			0    , 0    , 0.04
		]

		self.pub_imu.publish(msg)


if __name__ == "__main__":
	ImuBrickv2()	
