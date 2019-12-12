# 02 - Move a robot

This tutorial aims to take control over turtlebut2 kobuki platform helped by laserange to nagivate in a cluterred environement.

## Installation:

Targeted robot is a TurttleBot2 (Kobuki version)

First you have to install turtlebut2 packages that allow you to interact with the robot through ROS topics.

Prepare the install

```bash
sudo apt install ros-melodic-joy
```

Instructions for ROS melodic version are available on github: [github.com/gaunthan/Turtlebot2-On-Melodic](https://github.com/gaunthan/Turtlebot2-On-Melodic)

From your catkin worksapce:

```bash
catkin_make
curl -sLf https://raw.githubusercontent.com/gaunthan/Turtlebot2-On-Melodic/master/install_basic.sh | bash
catkin_make
```

## Move the turtlebot:

You can connect and switch the robots on.
*minimal.launch* in *turtlebot_bringup* packages permits starting minimal *ROS* nodes to communicate with the robot.

Listing the topics and a generating the graph of *ROS* nodes provide an idea of robot processes.
Then, by publishing velocity msg on on the appropriated command topic (*input/navi*).
The mesage structure is a [geometri_msgs](https://wiki.ros.org/geometry_msgs) - [twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)

### To do:

- Identify *cmd_vel_mux*. This node prioritizes commands provides by different systems through different topics ([wiki.ros.org/cmd_vel_mux](https://wiki.ros.org/cmd_vel_mux))
- Try to teleoperate the robot
- Create your own turtlebot package with a new node "dance" that move the robot with a sequence of predetermined movements.

## First estimation of robot position

*Open Loop Control* consider that there is a gap between the applied commands and the effective movement.
Control process continuously observe the pose estimation correct commands.
We aim to move the robot until it reaches a target position.

The topic *odom* provide a first estimation of the robot movements by observing wheels' movements.
By cumulating *odom* information, it is possible to maintain an estimation of the pose (position, orientation) of the robot on its environment.

Lucky for us, turtlebot already provides such pose estimation by maintaining transformation data that permit connecting *\odom* frame (matching the start pose of the robot) with *\base_footprint* frame (matching the actual pose of the robot).

### To do:

- Echo *odom* topic and manipulate turtlebot
- Echo *tf* topic and manipulate turtlebot
- Understand how to use *transformation* in ROS
  * [wiki.ros.org/tf](http://wiki.ros.org/tf)
  * [Tutorials](http://wiki.ros.org/tf/Tutorials)
- Propose a *node* (move) permitting the robot to reach a (x, y) position with an *epsilon* error by listening *tf* information to estimate the robot position toward its goal position and to computing the appropriated velocity commands.

## Detect obstacles

Ideally, the robot reaches its targeted position by avoiding obstacles...

### To do:
- Connect a USB LaserRange with a *urg_node* and interpret sensor information to adapt robot movements.
