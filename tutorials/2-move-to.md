# 02 - Move a robot

This tutorial aims to take control over turtlebut2 kobuki platform helped by laserange to nagivate in a cluterred environement.

## Installation:

The authors provide ready to use catkin package here: 

- On [bitbucket.org/imt-mobisyst](https://bitbucket.org/imt-mobisyst/mb6-tbot)

Go there and follow the instructions...

## Connect the turtlebot:

You can connect and switch the robots on.
*minimal.launch* in *turtlebot_bringup* packages permits starting minimal *ROS* nodes to communicate with the robot.

In a first terminal: 

```bash
roslaunch turtlebot_bringup minimal.launch
```

Listing the topics and a generating the graph of *ROS* nodes provide an idea of robot processes.
Then, by publishing velocity msg on on the appropriated command topic (*input/navi*).
The mesage structure is a [geometri_msgs](https://wiki.ros.org/geometry_msgs) - [twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)

## Move the turtlebot:


### To do:

- Identify *cmd_vel_mux*. This node prioritizes commands provided by different systems through different topics ([wiki.ros.org/cmd_vel_mux](https://wiki.ros.org/cmd_vel_mux))
- Try to teleoperate the robot
- Create your own turtlebot package with a new node "dance" that move the robot with a sequence of predetermined movements.

## Detect obstacles

Ideally, the robot reaches a targeted position (move forward for 1 meter) by avoiding obstacles...

### To do:

- Connect a USB LaserRange with a *urg_node* and interpret sensor information to adapt robot movements.
