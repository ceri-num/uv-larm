# Simultaneous Localization and Mapping (SLAM) in a nutshell

Mobile robots rely heavily on accurate representations of the environment (i.e **maps**) to fulfill their tasks (autonomous navigation, exploration, ...). Inside buildings, GPS signals are too weak to be used to localize robots. Hence we face a so-called Chicken-and-Egg-Problem, as *localization* requires a map, and *map building* requires the current location.

One solution consists in doing Simultaneous Localization and Mapping using a SLAM algorithm that typically reaches centimetric precision. There are many different flavors of SLAM especially regarding the map format.

The dominating 2D map format is the occupancy grid, also called grid map.
A grid map is a matrix whose cells represents a defined region of the real world; this is the *resolution* of the grid map (typically a square of 5cm).
A cell holds the estimated probability that the space it represents is traversable (free space) or not (obstacle).
The simplest format is the 3-state occupancy grid in which a cell has 3 different possible values: 0 (free space), 0.5 (unknown) and 1 (obstacle).


@Todo: fig. Example
<!-- Figure 2.5 -->

<!-- Localization
- Dead Reckoning
- Particle Filters
- Kalman Filters
- Pose Graph Optimization
- Scan matching -->

# Goal of this Tutorial

- Discover and use ROS tools: `roscore`, `rosrun`, `roslaunch` (launch files), `rqt_graph`, catkin, `rviz`, `rosbag`
- Use robotics simulators: **stage** (2d), **gazebo** (3d)
- Use *GMapping* SLAM to build a map of an indoor environment both in simulation and real world  

# ROS Prerequisites


- ROS correctly installed (cf. [Setup](tutorials/setup.md))
- **Catkin workspace** is a directory (usually `~/catkin_ws`) in which you deploy catkin packages 

# 2d Simulation with Stage 

Let's start with a first 2d ROS simulation with the [stage](http://wiki.ros.org/stage) simulator.

>Stage provides several sensor and actuator models, including sonar or infrared rangers, scanning laser rangefinder, color-blob tracking, fiducial tracking, bumpers, grippers and mobile robot bases with odometric or global localization. 

##Install Stage

Verify that stage is installed:

	```shell
	$ dpkg -l | grep stage
	ii  ros-melodic-stage                                  4.3.0-0bionic.20191008.154542                       amd64        Mobile robot simulator http://rtv.github.com/Stage
	ii  ros-melodic-stage-ros                              1.8.0-0bionic.20191008.204912                       amd64        This package provides ROS specific hooks for stage 
	```

Install it otherwise:
	
	```shell
	$ sudo apt update
	$ sudo apt install ros-melodic-stage ros-melodic-stage-ros
	```

You can explore the files installed by these two packages:

	```shell
	$ roscd stage
	```

##Simulating one robot with stage

[Source](http://wiki.ros.org/stage/Tutorials/SimulatingOneRobot)

	```shell
	$ roscore
	...
	# another shell
	$ rosrun stage_ros stageros $(rospack find stage_ros)/world/willow-erratic.world
	```

<!-- 
rostopic list	
rqt_image
rviz
move robot around
.world format
-->

## Rviz



# 3d Simulation with Gazebo 


# Save and Replay Data using `rosbag`


# Map building using GMapping

both in simul and real world


# Bibliography 

[1] Phd Johann