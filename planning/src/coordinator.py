import rospy
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Int16
import time


#Init node
def start(graph, actions):
    #global pub_turtle, pub_drone
    #Initialize curnt node with some name

    #Initialize publisher to publish PoseArray
    rospy.init_node('planner')
    turtlebot_publisher = rospy.Publisher("/list_of_turtle_goals", PoseArray, queue_size = 1)
    drone_publisher = rospy.Publisher("/list_of_drone_goals", PoseArray, queue_size = 1)
    time.sleep(4)
    print "I am ready to move!"

    #pub_drone = rospy.Publisher("/list_of_drone_goals", PoseArray, queue_size = 1)
    obj = Coordinator(actions, turtlebot_publisher, drone_publisher)
    #Subscribe to message published from goal_publisher about the goal accomplished
    rospy.Subscriber("/turtle_goal_completed", PoseArray, obj.setGoalStatus)
    rospy.Subscriber("/drone_goal_completed", PoseArray, obj.setGoalStatus)
    #Subscribe to message published from goal_publisher about the goal accomplished
    #rospy.Subscriber("/drone_goal_completed", Int16, drone_completed)
    #Sleep for a while to let all nodes Initialize

    """
    # call to move the turtlebot
    x = -0.67; y = 3.57; actionType = 0; actionId = 1


    # We check if the TurtleBot reaches the Objective
    moveTurtleBot(x,y, actionType,actionId, publisher)


    x = 0.211; y = 2.002; actionType = 0; actionId = 2
    moveTurtleBot(x,y, actionType,actionId, publisher)
    """

    for action in graph.succ:
        action.execute(turtlebot_publisher, drone_publisher)

    #This keeps the  active till it is killed
    rospy.spin()


# A method that calls the publishing API and moves bot to location x,y,z
# x,y are coordinates, z is suppose to be the type of the goal
def moveTurtleBot(x,y,actionType, actionId, publisher):
    print "move"
    newPoseArray = PoseArray()
    newPoseArray.header.frame_id = "map"
    newPoseArray.poses.append(Pose())
    newPoseArray.poses[0].position.x = x
    newPoseArray.poses[0].position.y = y
    newPoseArray.poses[0].position.z = actionType;
    newPoseArray.poses[0].orientation.z = actionId;



    # Publish the new list to the tb_path_publisher
    # to instruct the robot to move
    publisher.publish(newPoseArray)

class Coordinator(object):

    def __init__(self, actions, turtlebot_publisher, drone_publisher):
        self.actions = actions
        self.turtlebot_publisher = turtlebot_publisher
        self.drone_publisher = drone_publisher
        self.action_index = {}
        for action in actions:
            self.action_index[action.index] = action

    # This is a function that is calledback by the topic
    def setGoalStatus(self, data):
        print "info: completed ", data
        action_type = data.poses[0].position.z
        actionId = data.poses[0].orientation.z

        action = self.action_index[actionId]
        if action_type < 0:
            raise Exception("Failed action: " + action.format())

        action.complete_action()
        for succ_action in action.succ:
            all_done = True
            for pre_action in succ_action.pre:
                if not pre_action.completed:
                    alldone = False
                    break
            if all_done:
                succ_action.execute(self.turtlebot_publisher, self.drone_publisher)

        print "action type: ", action_type
        print "actionId: ", actionId

# end moveTurtleBot(x,y,z,publisher)
