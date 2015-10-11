#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import rospy
import tf
import actionlib
from math import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from optparse import OptionParser


class Square:
	def __init__(self):
		rospy.init_node('umbmark')
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		while not self.move_base.wait_for_server(rospy.Duration(5)):
			rospy.loginfo("Waiting for the move_base action server to come up")

	def next_pos(self, x, y, angle):
		odom_quat = tf.transformations.quaternion_from_euler(0, 0, angle)

		rospy.loginfo("Moving to (%.2f, %2f), %d°.." % (x, y, angle*180/pi))

		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "odom"
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

	def run(self, size=1):
		self.next_pos(0, 0, 0)
		self.next_pos(size, 0, 0)
		self.next_pos(size, -size, 0)
		self.next_pos(0, size, 0)

if __name__ == "__main__":
	parser = OptionParser()
	parser.add_option("-l", "--length", dest="length", default=2, help="Square size")
	(options, args) = parser.parse_args()

	p = Square()
	p.run(float(options.length))
