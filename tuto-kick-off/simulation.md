# Simulation in ROS

## Gazebo Simulator

[Gazebo](http://gazebosim.org/) is a 3D simulator.
It makes it possible to rapidly test algorithms, design robots, perform regression testing, and train AI systems using realistic scenarios.
Gazebo is integrated with ROS and supports various robots out of the box.

Gazebo is heavily used by the DARPA challenges (cf. [Wikipedia](https://en.wikipedia.org/wiki/Gazebo_simulator)).
You can see videos online ([example](https://www.youtube.com/watch?v=v6-heLIg85o)) and even load the maps and robot model that are available.

## Gazebo Installation

Verify that _Gazebo_ is installed using:

```console
dpkg -l | grep gazebo
```

You should have at least the following packages:

```
ii  gazebo                                          11.10.2+dfsg-1                          amd64        Open Source Robotics Simulator - Binaries
ii  gazebo-common                                   11.10.2+dfsg-1                          all          Open Source Robotics Simulator - Shared files
ii  gazebo-plugin-base                              11.10.2+dfsg-1                          amd64        Open Source Robotics Simulator - base plug-ins
ii  libgazebo-dev                                   11.10.2+dfsg-1                          amd64        Open Source Robotics Simulator - Development Files
ii  libgazebo11:amd64                               11.10.2+dfsg-1                          amd64        Open Source Robotics Simulator - shared library
ii  ros-iron-gazebo-dev                             3.7.0-3jammy.20230622.191804            amd64        Provides a cmake config for the default version of Gazebo for the ROS distribution.
ii  ros-iron-gazebo-msgs                            3.7.0-3jammy.20231117.090251            amd64        Message and service data structures for interacting with Gazebo from ROS2.
ii  ros-iron-gazebo-plugins                         3.7.0-3jammy.20231117.111548            amd64        Robot-independent Gazebo plugins for sensors, motors and dynamic reconfigurable components.
ii  ros-iron-gazebo-ros                             3.7.0-3jammy.20231117.104944            amd64        Utilities to interface with Gazebo through ROS.
ii  ros-iron-gazebo-ros-pkgs                        3.7.0-3jammy.20231117.114324            amd64        Interface for using ROS with the Gazebo simulator.
ii  ros-iron-turtlebot3-gazebo                      2.2.5-4jammy.20231117.114359            amd64        Gazebo simulation package for the TurtleBot3
```

Install missing packages using:

```console
sudo apt install <pakage_name>
```

## Launch your first Gazebo Simulation

If `$ROS_WORKSPACE/pkg-tsim` is *not* installed:

```console
cd $ROS_WORKSPACE
git clone https://bitbucket.org/imt-mobisyst/pkg-tsim
colcon build
source ~/.bashrc
```

Then, you can launch a preconfigured simulation:

```console
ros2 launch tbot_sim challenge-1.launch.py
```

Look at the content of this launch file [here](https://bitbucket.org/imt-mobisyst/pkg-tsim/src/master/tbot_sim/launch/challenge-1.launch.py).
We can see that Gazebo/ROS supports loading a world file describing the simulation environment and spawn elements such as robots.
This simulation spawns a robot configured like a `tbot` i.e. it is equipped with a laser range finder and a camera (kinect).
The interaction with the simulation will operate through ROS topics as it would be with a real robot with real equipments.

## Quiz on challenge 1

While the challenge 1 simulation is running:

> Question: which topics are available i.e published by Gazebo?

Hint: use `rqt_graph` to see the graph of all ROS nodes and topics ([doc](https://docs.ros.org/en/iron/Concepts/About-RQt.html)).

> Question: In which topic are laser scans published?

## Visualizing data

`rviz2` is a versatile and powerful tool to display data published in topics.
Launch it:

```console
rviz2
```

> Question: How to configure `rviz2` to visualize the laser scans?

Be carreful, ensure that `Global Option` / `Fixed frame` is correctly set to `base_link`.

> Question: why is this important? (hint: check your `tf` using `ros2 run tf2_tools view_frames.py`)

You can also display the `tf` in `rviz2` directly.

> Question: How to visualize camera images using `rqt`?

![rviz2](../files/SLAM/rviz_laserscan.png)

## Controlling the Simulated Robot

Launch a simple node to control the simulated robot using keyboard:

```console
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## tuto_sim

Create a new package (python or cmake as you want) `tuto_sim` in your _ROS2_ workspace and create a launch file that starts the apropriate configration: the `challenge-1`, a configured `rviz2` displaying laser scans and the `teleop`.

All the information you need are in the tutorials on [docs.ros.org](https://docs.ros.org/en/iron/Tutorials/Intermediate/Launch/Launch-Main.html).

## Move the simulated robot using your code

In the previous `tuto_move`, you already work on different nodes (`move_1m.py`, `turn_left_45`, `turn_right_45`, `rear_0.5m`) that makes the real robot moving. Without modify the code of these nodes, use them to move the simulated robot.
