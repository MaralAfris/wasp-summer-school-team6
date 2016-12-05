import sys,re,rospy;
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion
from std_msgs.msg import Int16

previous_coords = [0.0, 0.0]

class GoToPose():
    def __init__(self):

        self.goal_sent = False

	# What to do if shut down (e.g. Ctrl-C or failure)
	rospy.on_shutdown(self.shutdown)

	# Tell the action client that we want to spin a thread by default
	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("Wait for the action server to come up")

	# Allow up to 5 seconds for the action server to come up
	self.move_base.wait_for_server(rospy.Duration(9))

    def goto(self, pos, quat):
        # Send a goal
        self.goal_sent = True

	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	# Start moving
        self.move_base.send_goal(goal)

	# Allow TurtleBot up to 60 seconds to complete task
	success = self.move_base.wait_for_result(rospy.Duration(60))

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

def drone_action(data):
    global previous_coords

    x_coord = data.poses[0].position.x
    y_coord = data.poses[0].position.y
    action_type = data.poses[0].position.z
    actionId = data.poses[0].orientation.z

    is_move = True
    moveToAcoordinate(x_coord, y_coord, is_move, actionId)

# a function that moves the turtlebot to a x,y location
def moveToAcoordinate(coorX, coorY, is_move, actionId):
    global pub_completed
    try:
        #navigator = GoToPose()
        #TODO implement the actual move method
        reached = True;
        # construct a return possArray
        newPoseArray = PoseArray()
        newPoseArray.poses.append(Pose())
        newPoseArray.poses[0].orientation.z = actionId;

        if reached:
            rospy.loginfo("Hooray, reached the desired pose")
            newPoseArray.poses[0].position.z = 0;
            pub_completed.publish(newPoseArray)

        else:
            rospy.loginfo("The drone failed to reach the desired pose")
            newPoseArray.poses[0].position.z = -1;
            pub_completed.publish(newPoseArray)

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")

def start():
    global pub_completed
    rospy.init_node('drone_goal_publisher', anonymous=False)
	#Assigin publisher that publishes the index of the goal just accomplished
    pub_completed = rospy.Publisher('/drone_goal_completed', PoseArray, queue_size=1)
    rospy.Subscriber("/list_of_drone_goals", PoseArray, drone_action)

    rospy.spin()

if __name__ == '__main__':
    start()
