#!/usr/bin/python2.7
import math, rospy
from geometry_msgs.msg import PoseStamped, Twist
import tf

# Initialize ROS::node
rospy.init_node('move', anonymous=True)

# Initialize global variable
_trans = tf.TransformListener()
_goal = PoseStamped()

# Initialize node parrameters (parrameter name, default value)
def node_parameter(name, default):
    value= default
    try:
        value= rospy.get_param('~' + name)
    except KeyError:
        value= default
    return value

_goal_topic= node_parameter('goal_topic', '/goal')
_goal_frame_id= node_parameter('goal_frame_id', 'odom')
_cmd_topic= node_parameter('cmd_topic', '/cmd_vel')
_cmd_frame_id= node_parameter('cmd_frame_id', 'base_link')

# Initialize command publisher:
_cmb_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

# Subscribe to topic to get a goal position with goal_subscriber function
def log_goal(data):
    rospy.loginfo( 'Goal ' + data.header.frame_id +": (" + str(data.pose.position.x) +", "+str(data.pose.position.y) +", "+str(data.pose.position.z) +")" )

def goal_subscriber(data):
    global _goal
    log_goal(data)
    _goal= _trans.transformPose(_goal_frame_id, data)
    log_goal(_goal)

rospy.Subscriber(_goal_topic, PoseStamped, goal_subscriber)

# Publish continously velocity commandes
def move_command(data):
    log_goal(_goal)
    if _goal.header.frame_id != '' :
        local_goal= _trans.transformPose(_cmd_frame_id, _goal)
    else :
        local_goal= _goal
    log_goal(local_goal)
    # Compute cmd_vel here and publish... (do not forget to reduce timer duration)

rospy.Timer( rospy.Duration(1), move_command, oneshot=False )

# spin() simply keeps python from exiting until this node is stopped
print("Start move.py")
rospy.spin()
