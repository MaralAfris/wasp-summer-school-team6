import sys,re,rospy;
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

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
	self.move_base.wait_for_server(rospy.Duration(5))

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

def turtle_action(data):
    global previous_coords

    # Action types from planner
    move = 0
    deliver = 2

    is_move = True

    x_coord = data.poses[0].position.x
    y_coord = data.poses[0].position.y
    action_type = data.poses[0].position.z

    if action_type == deliver:
        is_move = False
        x_coord = previous_coords[0]
        y_coord = previous_coords[1]

    moveToAcoordinate(x_coord, y_coord, is_move)

    previous_coords[0] = x_coord
    previous_coords[1] = y_coord

# a function that moves the turtlebot to a x,y location
def moveToAcoordinate(coorX, coorY, is_move):
    global pub_completed
    try:
        navigator = GoToPose()

        # Customize the following values so they are appropriate for your location
        position = {'x': coorX, 'y' : coorY}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

        if is_move == False:
            # Spin to represent a deliver
            quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : -1.000}

        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        success = navigator.goto(position, quaternion)

        if success:
            rospy.loginfo("Hooray, reached the desired pose")
            pub_completed.publish(0)

        else:
            rospy.loginfo("The base failed to reach the desired pose")
            pub_completed.publish(-1)

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")

def start():
    global pub_completed
    rospy.init_node('turtle_goal_publisher', anonymous=False)
	#Assigin publisher that publishes the index of the goal just accomplished
	pub_completed = rospy.Publisher('/turtle_goal_completed', Int16, queue_size=1)
    rospy.Subscriber("/list_of_turtle_goals", PoseArray, turtle_action)
   rospy.spin()

if __name__ == '__main__':
    start()
