# Simultaneous Localization and Mapping (SLAM)


# Theory in a nutshell

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

- Discover and use ROS tools: `rosrun`, `roslaunch` (launch files), `rqt_graph`, catkin, `rviz`, `rosbag`
- Use robotics simulators: stage (2d), gazebo (3d)
- Use GMapping SLAM to build a map of an indoor environment both in simulation and real world  

# ROS Prerequisites


* [catkin workspace]

# 2d Simulation with Stage 


# 3d Simulation with Gazebo 
 

# Rviz and Laser


# Save and Replay Data using `rosbag`


# Map building using GMapping




# Bibliography 

[1] Phd Johann