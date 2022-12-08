# 02 - Move a robot

This tutorial aims to take control of a __tbot__  robot.
A tbot is a turtlebot2 (itself based on a kobuki platform) equipped with a laser range to navigate in a cluttered environment and a camera to recognize objects.

<!-- OU PAS This tutorial supose that you already processed the [Challenge - Kick-Off](../challenge/intro.md).-->

## Connect the tbot:

If it is not yet the case,
the machine connecting the robot and its sensors have to be conferred accordingly to the [IMT MobiSyst tbot](https://bitbucket.org/imt-mobisyst/mb6-tbot) 

You will have then a ROS WorkSpace including tbot meta-package itself including several ROS packages.

- Verrify it: 

```sh
cd rosworkspace
ls src
ls src/tbot
```

- Build the packages: 

```
colcon build
```

- Update your shell environment variables: 

```sh
source install/setup.sh
```

- Connect the tbot base and start the control nodes : 

```sh
ros2 run tbot_start start # feed the bot password is asked
```

Into another terminal start a bridge between ros1 and ros2: 

```sh
source /opt/ros/noetic/setup.sh
ros2 run ros1_bridge dynamic_bridge
```

Finally, try to take control in a third terminal:

``sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/mobile_base/commands/velocity
``

Close everything with `ctrl-c`.



## Somme explanations:


The control of tbot (turtlebot3 kobuki base) is still dependent on old version of ros1 so it is packaged into a docker environment.
This solution implies: 1) super user access to run docker (potentially a sudo password) and 2) to launch a bridge between ros1 and ros2.

The 'dynamic_bridge' needs some ROS1 variables that why the user has to source `ROS/noetic` in the terminal before to start the bridge.
At this point, the ROS1 topics are invisible in ROS2. It is a dynamic bridge, It connects things only on demand.
So by starting the teleop node, you can see the activation of the bridge in its terminal.

It works well if and only if you address the appropriate topic. Here it is exactly `/mobile_base/commands/velocity`.
If you would like to see all robot topics, you have to switch your terminal in ROS1 and then list the topics with ROS1 command:

```
source /opt/ros/noetic/setup.sh
rostopic list
```

The teleop publishes a [geometry_msgs](https://docs.ros2.org/foxy/api/geometry_msgs/index-msg.html)/[twist](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html) mesage.
It is composed of two vectors $(x, y, z)$, one for linear speed $(m/s)$, and the second for angular speed $(rad/s)$. 
However a [https://en.wikipedia.org/wiki/Nonholonomic_system](nonholonomic) ground robot as the **tbot** would move only on `x` and turn only on `z`. 
It is not as free as a drone, you can echo the mesages into a 4th terminal.

- Try to control the robot with `ros2 topic pub` command instead of teleop.


## move node














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


Try to move the robot in a straight line at a speed of $0.1$ meters per second.  The message requires to be sent several times until the robot reach the appropriate position, for instance, at a rate of 10 messages per second.

So:

```bash
rostopic pub -r 10 /cmd_vel_mux/input/navi geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
```

`ctr-c` to stop a running program in a terminal.

With turtlesim, the topic must be adapted to something like: `/turtle1/cmd_vel`








**OLD VERSION**


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

1. Launch `rviz`
2. Add topic `laserScan`
3. Change reference frame to `laser`

The game is to integrate incoming information from `/scan` to `move.py` script to avoid obstacles in front of the robot.

1. Add dependence to scan messages. 
2. Subscribe to `/scan` topics and associate a call back function.
3. Transform the `/scan` distances into obstacle positions
4. Make the robot turning right or left to avoid close obstable on the left  or on the right (positive or negative value on the `Twist angular z` attribut).

Without a turtlebot, you can play with reccorded sensor stream (a bagfile).
cf. tutorials: [record bag files](http://wiki.ros.org/ROS/Tutorials/Recording%20and%20playing%20back%20data), [play bag files](http://wiki.ros.org/ROS/Tutorials/reading%20msgs%20from%20a%20bag%20file).

You can get a bag file in `mb6-data` repository:

```bash
wget https://bitbucket.org/imt-mobisyst/mb6-data/raw/master/tbot_bags/bags/tbot_bag_first_loop.bag -O move.bag
```

then run it: 

```bash
rosbag play move.bag
```

### From laser scan to point cloud

[laser scan](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html) in the sensor messages package, provide both the recorded bean distances and the meta information permiting to convert the distances on points in a regular carthesian frame (i.e. the angle between beams).

In a python script, the conversion to put in a call-back function attached to `/scan` topic, would look-like this:

```python
obstacles= []
angle= data.angle_min
for aDistance in data.ranges :
    if 0.1 < aDistance and aDistance < 5.0 :
        aPoint= [ 
            math.cos(angle) * aDistance, 
            math.sin( angle ) * aDistance
        ]
        obstacles.append( aPoint )
    angle+= data.angle_increment
rospy.loginfo( str(
    [ [ round(p[0], 2), round(p[1], 2) ] for p in  obstacles[0:10] ] 
) + " ..." )
```

Ideally, it is possible to publish this result in a pointCloud mesage and to visualize it on rviz...


### Avoid obstacle

If the laser position is coherent on the robot, it is possible to consider that the obstacle point list is provided in the same frame than the robot movement.

To avoid obstacle:

1. Determine if obstacle points are in front of th robot and in a given raduis.
2. Apply a rotation to the robot until the obstacle leave the danger zone.

Astuce: to share variable from a callback funtion to another in python, it is possible to use global variable (i.e. the obstacle list). [Exemple on w3schools](https://www.w3schools.com/python/gloss_python_global_variables.asp).
