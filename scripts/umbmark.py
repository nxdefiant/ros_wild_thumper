#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-
#
# Gazebo Position: rostopic echo -n 1 /gazebo/model_states
#
# x_err = x_real - x_odom
# y_err = y_real - y_odom
# phi_err = phi_real - phi_odom
#
# x_cg_cw/ccw: avg x_err
# y_cg_cw/ccw: avg y_err
#
# r_cg_cw = sqrt(x_cg_cw**2 + y_cg_cw**2)
# r_cg_ccw = sqrt(x_cg_ccw**2 + y_cg_ccw**2)
#
# L: length (2)
# D(l/r): Wheel diameter left/right
# b: wheelbase
#
# Wheel diameter correction:
# beta = (y_cg_cw + y_cg_ccw)/(-4*L)
# R = (L/2)/sin(beta/2)
# Ed = Dr/Dl*(R+b/2)/(R-b/2)
# Da = (Dr + Dl)/2
# Dl = 2/(Ed + 1) * Da
# Dr = 2/((1/Ed) + 1) * Da
# 
# Wheelbase correction:
# alpha = (y_cg_cw - y_cg_ccw)/(-4*L) * 180/pi
# Eb = (90)/(90-alpha)
# b_new = Eb*b

import sys
import rospy
import tf
import actionlib
import operator
from time import sleep
from math import *
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

class UMBMark:
	def __init__(self):
		rospy.init_node('umbmark')
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		rospy.Subscriber("odom", Odometry, self.odom_received)
		self.odom_pose = None
		while not self.move_base.wait_for_server(rospy.Duration(5)):
			rospy.loginfo("Waiting for the move_base action server to come up")

	def odom_received(self, msg):
		orientation = tf.transformations.euler_from_quaternion(msg.pose.pose.orientation.__getstate__())
		self.odom_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, orientation[2])

	def next_pos(self, x, y, angle):
		odom_quat = tf.transformations.quaternion_from_euler(0, 0, angle)

		rospy.loginfo("Moving to (%.2f, %2f), %d°.." % (x, y, angle*180/pi))

		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "base_footprint"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = x
		goal.target_pose.pose.position.y = y
		goal.target_pose.pose.orientation.x = odom_quat[0]
		goal.target_pose.pose.orientation.y = odom_quat[1]
		goal.target_pose.pose.orientation.z = odom_quat[2]
		goal.target_pose.pose.orientation.w = odom_quat[3]
		self.move_base.send_goal(goal)

		self.move_base.wait_for_result()

		if self.move_base.get_state() == GoalStatus.SUCCEEDED:
			rospy.loginfo("The base moved to (%.2f, %2f), %d°" % (x, y, angle*180/pi))
		else:
			rospy.logerr("The base failed to (%.2f, %2f), %d°" % (x, y, angle*180/pi))
			raise

	def run(self, direction=-1):
		while self.odom_pose is None:
			sleep(0.1)
		init_pose = self.odom_pose
		for i in range(4):
			self.next_pos(2, 0, 0)
			self.next_pos(0, 0, direction*90*pi/180)
		final_pose = map(operator.sub, self.odom_pose, init_pose)
		print "Odom Pose: x=%.3f, y=%.3f, angle=%.3f°" % (final_pose[0], final_pose[1], final_pose[2]*180/pi)

	def run_cw(self):
		self.run(-1)

	def run_ccw(self):
		self.run(1)

if __name__ == "__main__":
	p = UMBMark()
	if len(sys.argv) > 1 and sys.argv[1] == "ccw":
		p.run_ccw()
	else:
		p.run_cw()
