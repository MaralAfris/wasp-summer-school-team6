#! /usr/bin/env python

import roslib
import rospy

from controller import Controller, ActionStatus
from geometry_msgs.msg import PoseStamped

from bebop_controller.msg import *
from actionlib import SimpleActionClient
from BebopActionServer import BebopActionServer
from actionlib_msgs.msg import GoalStatus

if __name__ == '__main__':
	try:
		# Initialize the node
		rospy.init_node('BebopActionClient', anonymous=True, log_level=rospy.INFO)
		ac_takeoff = SimpleActionClient('BebopTakeOffAction', BebopTakeOffAction)
		ac_takeoff.wait_for_server()
		ac_movebase = SimpleActionClient('BebopMoveBaseAction', BebopMoveBaseAction)
		ac_movebase.wait_for_server()
		ac_land = SimpleActionClient('BebopLandAction', BebopLandAction)
		ac_land.wait_for_server()
		takeoff = BebopTakeOffGoal()
		ac_takeoff.send_goal(takeoff)
		ac_takeoff.wait_for_result()
		print "Successfully taken off!"
		success1 = (ac_takeoff.get_state() == GoalStatus.SUCCEEDED)
		print success1
		goal = BebopMoveBaseGoal()
		goal.target_pose = PoseStamped()
		goal.target_pose.header.frame_id = "odom"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = 1.0
		goal.target_pose.pose.position.z = 1.5
		ac_movebase.send_goal(goal)
		ac_movebase.wait_for_result()
		print "Got the result from moving to goal!"
		success2 = (ac_movebase.get_state() == GoalStatus.SUCCEEDED)
		print success2
		land = BebopLandAction()
		ac_land.send_goal(land)
		ac_land.wait_for_result()
		print "Successfully landed!"

		

	except rospy.ROSInterruptException:
		print "Program interrupted before completion"
