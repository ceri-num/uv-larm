# 02 - Move a robot

This tutorial aims to take control over turtlebut2 kobuki platform helped by laser range to navigate in a cluttered environment.

This tutorial supose that you already process the [Challenge - Kick-Off](../challenge/intro.md).

## Installation:

First, on a new machine, you have to create a user for your group.

Log as bot, and in a terminal:

```bash
sudo adduser grp-`color`
sudo usermod -a -G sudo grp-`color`
sudo usermod -a -G dialout grp-`color`
```

You can now logout and login in your own id. 

Then, the authors provide ready to use catkin package here: 

- On [bitbucket.org/imt-mobisyst](https://bitbucket.org/imt-mobisyst/mb6-tbot)

Go there and follow the instructions...

## Connect the turtlebot:

You can connect and switch the robots on.
*minimal.launch* in *turtlebot_bringup* packages permits starting minimal *ROS* nodes to communicate with the robot.

In a first terminal: 

```bash
roslaunch turtlebot_bringup minimal.launch
```

Without a `turtlebot`, you can play the same exercise with `turtlesim`.

Listing the topics and a generating the graph of *ROS* nodes provide an idea of robot processes.

```bash
rostopic list
rqt_graph
```

Then, by publishing velocity msg on the appropriated command topic (*input/navi*).
The message structure is a [geometry_msgs](https://wiki.ros.org/geometry_msgs) - [twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)

Try to move the robot in a straight line at a speed of $0.1$ meters per second.  The message requires to be sent several times until the robot reach the appropriate position, for instance, at a rate of 10 messages per second.

So:

```bash
rostopic pub -r 10 /cmd_vel_mux/input/navi geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
```

`ctr-c` to stop a running program in a terminal.

With turtlesim, the topic must be adapted to something like: `/turtle1/cmd_vel`

## Move the turtlebot:

We want to repeat this exercise but with a ROS-node.
We want to create a program that moves the robot, until it reached a position one meter in front of it.

1. We will require a new `catkin_pkg` dependent on geometry messages.

From your catkin repository:

```bash
cd src/grp-`color`
catkin_create_pkg my_awesome_pkg std_msgs geometry_msgs rospy roscpp
git add my_awesome_pkg
git commit -am "new my_awesome_pkg"
git pull
git push
```

2. In this package we include, make executable and edit a python scripts `moveStraight.py`.

```bash
cd my_awesome_pkg
mkdir scripts
touch scripts/move-1-meter.py
chmod +x scripts/move-1-meter.py
```

1. Write the code  with a publisher that moves the robot:

Something like this:

```python
#!/usr/bin/python3
import math, rospy
from geometry_msgs.msg import Twist

# Initialize ROS::node
rospy.init_node('move', anonymous=True)

commandPublisher = rospy.Publisher(
    '/cmd_vel_mux/input/navi',
    Twist, queue_size=10
)

# Publish velocity commandes:
def move_command(data):
    # Compute cmd_vel here and publish... (do not forget to reduce timer duration)
    cmd= Twist()
    cmd.linear.x= 0.1
    commandPublisher.publish(cmd)

# call the move_command at a regular frequency:
rospy.Timer( rospy.Duration(0.1), move_command, oneshot=False )

# spin() enter the program in a infinite loop
print("Start move.py")
rospy.spin()
```

You already can try your code (catkin_make, sources your devel/setup file then run `moveStraight.py` in `my_awesome_pkg` ). Your robot must move until you stop the program.


## Avoid obstacles :

We get the basics.

The next move is to detect obstacles with a laser-range and to avoid it.

Hokuyo laser range is a sensor compliant with ROS by using the *urg_node* package. Connect the sensor, run *urg_node* node in *urg_node* ros package. 

A new topic appears `/scan` streaming the LaserRange data. It is possible to visualize it with *rviz* (fixed frame initially on `map` has to be set on `laser` frame, i.e. at this point it is impossible for the system to know where is the laser in the map).

The game is to integrate incoming information from `\scan` to `move.py` script to avoid obstacles in front of the robot.

1. Add dependence to scan messages. 
2. Subscribe to `\scan` topics and associate a call back function.
3. Transform the `\scan` distances into obstacle positions
4. Make the robot turning right or left to avoid close obstable on the left  or on the right (positive or negative value on the `Twist angular z` attribut).

Without a turtlebot, you can play with reccorded sensor stream (a bagfile).

```bash
rosbag play scan.bag
```

### From laser scan to point cloud

[laser scan](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html) in the sensor messages package, provide both the recorded bean distances and the meta information permiting to convert the distances on points in a regular carthesian frame (.i.e the angle between beans).

In a python script, the conversion to put in a call-back function attached to `\scan` topic, would look-like this:

```python
obstacles= []
angle= data.angle_min
for aDistance in data.ranges :
    aPoint= [ 
        cos(angle) * aDistance, 
        sin( angle ) * aDistance
    ]
    obstacles.push( aPoint )
    angle+= scan.angle_increment
print( obstacles )
```

Ideally it is possible to publish this result in a pointCloud mesage and to visualize it on rviz...


### Avoid obstacle

If the laser position is coherent on the robot, it is possible to consider that the obstacle point list is provided in the same frame than the robot movement.

To avoid obstacle:

1. Determine if obstacle points are in front of th robot and in a given raduis.
2. Apply a rotation to the robot until the obstacle leave the danger zone.

Astuce: to share variable from a callback funtion to another in python, it is possible to use global variable (i.e. the obstacle list). [Exemple on w3schools](https://www.w3schools.com/python/gloss_python_global_variables.asp).
