#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import sys
import rospy
import tf
import actionlib
import tf2_ros
import dynamic_reconfigure.client
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geodesy import utm
from math import *

class GPSGotoCoords:
	def __init__(self):
		rospy.init_node('gps_goto_coords')
		rospy.on_shutdown(self.on_shutdown)

		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		self.tfBuffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tfBuffer)

		rospy.loginfo("Setting paramters")
		self.dynreconf = dynamic_reconfigure.client.Client("/move_base/TrajectoryPlannerROS")
		self.dynreconf.update_configuration({'max_vel_x': 1.0, 'max_vel_theta': 1.2, 'min_in_place_vel_theta': 1.0})

		rospy.loginfo("Waiting for the move_base action server to come up")
		self.move_base.wait_for_server()
		rospy.loginfo("Got move_base action server")

	def next_pos(self, lat, lon):
		rospy.loginfo("Moving to (%f, %f)" % (lat, lon))

		point = utm.fromLatLong(lat, lon)

		# Get the current position
		pos = self.tfBuffer.lookup_transform("utm", 'base_link', rospy.Time(0), rospy.Duration(2.0))
		# calculate angle between current position and goal
		angle_to_goal = atan2(point.northing - pos.transform.translation.y, point.easting - pos.transform.translation.x)
		odom_quat = tf.transformations.quaternion_from_euler(0, 0, angle_to_goal)

		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "utm"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = point.easting
		goal.target_pose.pose.position.y = point.northing
		goal.target_pose.pose.orientation.x = odom_quat[0]
		goal.target_pose.pose.orientation.y = odom_quat[1]
		goal.target_pose.pose.orientation.z = odom_quat[2]
		goal.target_pose.pose.orientation.w = odom_quat[3]
		self.move_base.send_goal(goal)

		while not self.move_base.wait_for_result(rospy.Duration(1.0)) and not rospy.is_shutdown():
			# Get the current position
			pos = self.tfBuffer.lookup_transform("utm", 'base_link', rospy.Time(0), rospy.Duration(2.0))
			# Cancel if we are close enough to goal
			if np.linalg.norm([point.northing - pos.transform.translation.y, point.easting - pos.transform.translation.x]) < 1:
				rospy.loginfo("Goal within 1m, canceling")
				self.move_base.cancel_goal()
				return

		if self.move_base.get_state() == GoalStatus.SUCCEEDED:
			rospy.loginfo("The base moved to (%f, %f)" % (lat, lon))
		else:
			rospy.logerr("The base failed to (%f, %f)" % (lat, lon))
			exit(1)

	def on_shutdown(self):
		rospy.loginfo("Canceling all goals")
		self.move_base.cancel_all_goals()

	def run(self, sCSVFile):
		f = open(sCSVFile)
		for line in f:
			pos = line.split(",")
			self.next_pos(float(pos[0]), float(pos[1]))
			self.move_base.cancel_all_goals()


if __name__ == "__main__":
	p = GPSGotoCoords()
	p.run(sys.argv[1])
