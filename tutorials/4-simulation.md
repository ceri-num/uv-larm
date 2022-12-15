# Simulation in ROS

## Gazebo Simulator

[Gazebo](http://gazebosim.org/) is a 3d simulator.
It makes it possible to rapidly test algorithms, design robots, perform regression testing, and train AI system using realistic scenarios. Gazebo is integrated with ROS (cf. [Gazebo ROS](http://wiki.ros.org/gazebo_ros_pkgs)) and supports various robots out of the box.

Gazebo is heavily used by the DARPA challenges (cf. [Wikipedia](https://en.wikipedia.org/wiki/Gazebo_simulator)).
You can see videos online ([example](https://www.youtube.com/watch?v=v6-heLIg85o)) and even load the maps and robot model that are available.

## Gazebo Installation

Vérifiez que Gazebo est installé.

```console
$ dpkg -l | grep gazebo
ii  gazebo11                                        11.12.0-1~focal                      amd64        Open Source Robotics Simulator
ii  gazebo11-common                                 11.12.0-1~focal                      all          Open Source Robotics Simulator - Shared files
ii  gazebo11-plugin-base                            11.12.0-1~focal                      amd64        Open Source Robotics Simulator - base plug-ins
ii  libgazebo11:amd64                               11.12.0-1~focal                      amd64        Open Source Robotics Simulator - shared library
ii  libgazebo11-dev:amd64                           11.12.0-1~focal                      amd64        Open Source Robotics Simulator - Development Files
ii  ros-foxy-gazebo-dev                             3.5.3-1focal.20220829.174620         amd64        Provides a cmake config for the default version of Gazebo for the ROS distribution.
ii  ros-foxy-gazebo-msgs                            3.5.3-1focal.20221012.224922         amd64        Message and service data structures for interacting with Gazebo from ROS2.
ii  ros-foxy-gazebo-plugins                         3.5.3-1focal.20221021.150213         amd64        Robot-independent Gazebo plugins for sensors, motors and dynamic reconfigurable components.
ii  ros-foxy-gazebo-ros                             3.5.3-1focal.20221013.010602         amd64        Utilities to interface with Gazebo through ROS.
```

Installez éventuellement les paquets manquants avec `sudo apt install <nom_paquet>`

## Testez Gazebo

```console
$ rosify2
(ros2) $ ros2 launch gazebo_ros gazebo.launch.py world:=$HOME/ros2_ws/src/tbot/tbot_sim/challenge-1.world
```

### Simulating a Robot

<!-- https://turtlebot.github.io/turtlebot4-user-manual/software/slam.html -->

```console
(ros2) $ xxxx
```

Test the main ROS2 tools:

```console
# show the ROS graph (nodes + topics)
(ros2) $ rqt_graph

# list the ROS topics and available data
(ros2) $ ros2 topic list
```

>Question: In which topic are laser scans published?

[rviz](http://wiki.ros.org/rviz) is a very useful and versatile tool to visualize data that goes through topics.
Visualize laser scans using `rviz`:

```console
(ros2) $ rviz2
```

Configure rviz to visualize the laser scans.
Be carreful, ensure that `Global Option` / `Fixed frame` is correctly configured to `base_link`.

>Question: why is this important? (hint: check your `tf` using `ros2 run tf_tools view_frames.py`)

![rviz2](../files/SLAM/rviz_laserscan.png)

Use `rqt_graph` to see the graph of ROS nodes and the topics they use to communicate.

## Controlling the Simulated Robot

Launch a simple node to control a robot using keyboard:

```console
(ros2) $ ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

> Why you cannot control the robot?

```
(ros2) $ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/demo/cmd_demo
```


## A program to control the simulated robot

TODO

