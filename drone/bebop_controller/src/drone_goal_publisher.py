#! /usr/bin/env python

import roslib
import rospy

from controller import Controller, ActionStatus
from geometry_msgs.msg import PoseStamped

from bebop_controller.msg import *
from actionlib import SimpleActionClient
from BebopActionServer import BebopActionServer
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion

prev_x = 0.0
prev_y = 0.0

def init_action_clients():
    global ac_takeoff, ac_movebase, ac_land, ac_load, ac_unload

    ac_takeoff = SimpleActionClient('BebopTakeOffAction', BebopTakeOffAction)
    ac_takeoff.wait_for_server()
    ac_movebase = SimpleActionClient('BebopMoveBaseAction', BebopMoveBaseAction)
    ac_movebase.wait_for_server()
    ac_land = SimpleActionClient('BebopLandAction', BebopLandAction)
    ac_land.wait_for_server()
    ac_load = SimpleActionClient('BebopLoadAction', BebopLoadAction)
    ac_load.wait_for_server()
    ac_unload = SimpleActionClient('BebopUnloadAction', BebopUnloadAction)
    ac_unload.wait_for_server()

def takeoff():
    global ac_takeoff
    takeoff = BebopTakeOffGoal()
    ac_takeoff.send_goal(takeoff)
    ac_takeoff.wait_for_result()
    success = (ac_takeoff.get_state() == GoalStatus.SUCCEEDED)
    return success

def drone_action(data):
    global ac_movebase, ac_load, ac_unload
    global prev_x, prev_y
    global pub_completed

    # Action types from planner
    move = 0
    pickup = 2
    handover = 3
    
    # Extract the goal of the action
    x_coord = data.poses[0].position.x
    y_coord = data.poses[0].position.y
    action_type = data.poses[0].position.z
    actionId = data.poses[0].orientation.z

    goal = BebopMoveBaseGoal()
    goal.target_pose = PoseStamped()
    goal.target_pose.header.frame_id = "odom"
    goal.target_pose.header.stamp = rospy.Time.now()

    if action_type == move:   
        goal.target_pose.pose.position.x = x_coord
        goal.target_pose.pose.position.y = y_coord
    else:
        goal.target_pose.pose.position.x = prev_x
        goal.target_pose.pose.position.y = prev_y

    ac_movebase.send_goal(goal)
    ac_movebase.wait_for_result()
    success = (ac_movebase.get_state() == GoalStatus.SUCCEEDED)

    prev_x = x_coord
    prev_y = y_coord
    
    # Construct result message
    newPoseArray = PoseArray()
    newPoseArray.poses.append(Pose())
    newPoseArray.poses[0].orientation.z = actionId;
        
    if success:
        print "Successfully completed an action"
        print actionId
        print action_type
        newPoseArray.poses[0].position.z = 0;
        pub_completed.publish(newPoseArray)
    else:
        newPoseArray.poses[0].position.z = -1;
        pub_completed.publish(newPoseArray)

def start():
    global pub_completed
    global ac_land
    try:
        rospy.init_node('drone_goal_publisher', anonymous=False)
        init_action_clients()
        taken_off = takeoff()
        if not taken_off:
            print "Warning: Take off failed!"
	    #Assigin publisher that publishes the index of the goal just accomplished
        pub_completed = rospy.Publisher('/drone_goal_completed', PoseArray, queue_size=1)
        rospy.Subscriber("/list_of_drone_goals", PoseArray, drone_action)
        rospy.spin()

    except rospy.ROSInterruptException:
        print "Execution interrupted, landing drone"
        land = BebopLandAction()
        ac_land.send_goal(land)
        ac_land.wait_for_result()
        print "Successfully landed!"

if __name__ == '__main__':
    start()