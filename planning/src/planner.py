import rospy
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Int16
import time
"""
# Arrays that keep information about goals
turtle_goals = []
drone_goals = []

# Action types from planner
move = 0
rendezvous = 1
deliver = 2
pickup = 3
putdown = 4


def publish_to_turtle_if_allowed():
    global turtle_goals, pub_turtle

    publish_if_allowed(turtle_goals, pub_turtle)

def publish_to_drone_if_allowed():
    global drone_goals, pub_drone

    publish_if_allowed(drone_goals, pub_drone)


#This function will update the list of goals that should be published to goal_publisher
def publish_if_allowed(goals, publisher):
    #Cate an object to new pose array
    newPoseArray = PoseArray()
    #assign frame of these pose objects as map
    newPoseArray.header.frame_id = "map"
    #Boundary Checks
    if(len(goals)!=0):
        # Always try to publish the top element
        current_goal = goals[0]
        if is_at_rendezvous(current_goal) == False:
            newPoseArray.poses.append(Pose())
            newPoseArray.poses[0].position.x = goal[0]
            newPoseArray.poses[0].position.y = goal[1]
            newPoseArray.poses[0].position.z = goal[2]; # the type of the goal
            #Publish the new list to the tb_path_publisher to instruct the robot
            publisher.publish(newPoseArray)

def is_at_rendezvous(goal):
    global turtle_goals, drone_goals

    action_type = goal[2]

    if action_type != rendezvous:
        return False

    current_turtle_goal_type = turtle_goals[0][2]
    current_drone_goal_type = drone_goals[0][2]

    if current_drone_goal_type == rendezvous && current_turtle_goal_type == rendezvous:
        # Let both continue moving past the rendezvous point
        turtle_goals.pop(0)
        drone_goals.pop(0)
        publish_to_turtle_if_allowed()
        publish_to_drone_if_allowed()

    return True

# This Callback function will be called everytime a turtle goal is ached.
# goal_publisher will publish the index of point in goal list
def turtle_completed(data):
    #global turtle_goals
    #index = data.data
    #update_goal_list(turtle_goals, index)
    #publish_to_turtle_if_allowed()

# This Callback function will be called everytime a drone goal is ached.
# goal_publisher will publish the index of point in goal list
def drone_completed(data):
    global drone_goals
    index = data.data
    update_goal_list(drone_goals, index)
    publish_to_drone_if_allowed()

def update_goal_list(goal_list, index):
    failure = -1
    if index == failure:
        print "FAILURE, ABORT MISSION!"
    else if index < len(goal_list):
        goal_list.pop(index)
    else:
        print "ERROR: GOAL LIST OUT OF RANGE"

def init_plan():
    global turtle_goals, drone_goals
    global move, rendezvous, deliver, pickup, putdown

    Plan = sys.stdin; # TODO: Fix this! (Or parse from file?)
    actions = [];

    for line in Plan:
        actions.append(line);

    waypoints = {'a': [-1.39,0.424], 'b': [1.63, -1.34], 'c': [-2.51, 1.2], 'd': [-2.51, 1.2],
                 'w': [-1.39,0.424], 'x': [1.63, -1.34], 'y': [-2.51, 1.2], 'z': [-2.51, 1.2],
                 'meeting-place': [0,0]};

    actions = actions[3:len(actions)-1];

    plan = re.compile('[0-9]+: \(([a-z0-9- ]+)\) \[[0-9]+\]');
    for a in actions:
        m = plan.match(a);
        action = m.group(1);
        parts = action.split(" ");
        type = parts[0];
        if type == 'move':
           bot = parts[1];
           #action_from = parts[2];
           action_to = parts[3];

           #map and send new locations
           print bot,waypoints[action_to]
           #Update the list of goals by appending the Curntly ceived point.
           if bot == "turtlebot1":
               print "turtlebot action!"
               print waypoints[action_to][0]
               turtle_goals.append([waypoints[action_to][0],waypoints[action_to][1], move)
           if bot == "drone1":
               print "drone action!"
               print waypoints[action_to][0]
               drone_goals.append([waypoints[action_to][0],waypoints[action_to][1], move)

       else if type == 'deliver':
           turtle_goals.append(0.0, 0.0, deliver) # action only for turtlebot

       else if type == 'pick-up':
           drone_goals.append(0.0, 0.0, pickup) # action only for drone

       else if type == 'hand-over':
           turtle_goals.append(0.0, 0.0, rendezvous)
           drone_goals.append(0.0, 0.0, rendezvous)
           drone_goals.append(0.0, 0.0, putdown) # action only for drone
"""
#Init node
def start():
    #global pub_turtle, pub_drone
    #Initialize curnt node with some name
    rospy.init_node('planner')
    #Initialize publisher to publish PoseArray
    publisher = rospy.Publisher("/list_of_turtle_goals", PoseArray, queue_size = 1)
    #pub_drone = rospy.Publisher("/list_of_drone_goals", PoseArray, queue_size = 1)

    #Subscribe to message published from goal_publisher about the goal accomplished
    #rospy.Subscriber("/turtle_goal_completed", Int16, turtle_completed)
    #Subscribe to message published from goal_publisher about the goal accomplished
    #rospy.Subscriber("/drone_goal_completed", Int16, drone_completed)
    #Sleep for a while to let all nodes Initialize
    print "Starting to sleep..."
    time.sleep(5)
    print "Woke up"
    #Cate an object to new pose array
    newPoseArray = PoseArray()
    #assign frame of these pose objects as map
    newPoseArray.header.frame_id = "map"
    newPoseArray.poses.append(Pose())
    newPoseArray.poses[0].position.x = -1.0
    newPoseArray.poses[0].position.y = 1.0
    newPoseArray.poses[0].position.z = 0; # the type of the goal
    #Publish the new list to the tb_path_publisher to instruct the robot
    publisher.publish(newPoseArray)

    #Init the global plan
    #init_plan()
    
    #Publish initial goals
    #publish_to_drone_if_allowed()
    #publish_to_turtle_if_allowed()

    #This keeps the  active till it is killed
    rospy.spin()

if __name__ == '__main__':
    start()
