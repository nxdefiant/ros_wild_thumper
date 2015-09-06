#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

class Waypoint:
	def __init__(self):
		rospy.init_node('waypoint')
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		while not self.move_base.wait_for_server(rospy.Duration(5)):
			rospy.loginfo("Waiting for the move_base action server to come up")

	def send_goal(self):
		goal = MoveBaseGoal()

		# we'll send a goal to the robot to move 1 meter forward
		goal.target_pose.header.frame_id = "base_footprint"
		goal.target_pose.header.stamp = rospy.Time.now()

		goal.target_pose.pose.position.x = 1.0
		goal.target_pose.pose.orientation.w = 1.0

		self.move_base.send_goal(goal)

		self.move_base.wait_for_result()

		if self.move_base.get_state() == GoalStatus.SUCCEEDED:
			rospy.loginfo("The base moved 1 meter forward")
		else:
			rospy.logerr("The base failed to move forward 1 meter")

if __name__ == "__main__":
	p = Waypoint()
	p.send_goal()
