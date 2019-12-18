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

- Identify *cmd_vel_mux*. This node prioritizes commands provided by different systems through different topics ([wiki.ros.org/cmd_vel_mux](https://wiki.ros.org/cmd_vel_mux))
- Try to teleoperate the robot
- Create your own turtlebot package with a new node "dance" that move the robot with a sequence of predetermined movements.

## Detect obstacles

Ideally, the robot reaches a targeted position (move forward for 1 meter) by avoiding obstacles...

### To do:
- Connect a USB LaserRange with a *urg_node* and interpret sensor information to adapt robot movements.
