# Mapping an Medium Environment

The goal of the challenge is to demonstrate the capability the robot has to map an environment and to retrieve spécific objects on it.

## Preparation

Install the [LARM simulation workspace]() in place of `simulation_ws`.

## Expected

A launch file `mapping.launch` in the student package in `catkin_ws/src/students_package` that configure the robot control architecture in a way that:

* The robot moves toward provided positions all over the environment.
* The robot build a map while navigating in it environment.
* The robot communicate in a topic `/bottle` the position of bottles, each time the robot recognizes the appropriate object.

## Demonstration protocol

1. Launch the simulation configuration: `roslaunch larm large_world.launch`
2. Launch the control architecture: `roslaunch studient_pkg mapping.launch`
3. Visualize the map and the pose position in the `/bottle` topic on rviz
4. The robot can be controlled manually.

The map configuration and the robot position would be provided long time before the evaluation.
Only the bottle poses (with baits) would change randomly at the last minute, but in a radius of 10 meters arround the start possition of the robot.

## In the video

1. A presentation of the challenge
2. A presentation of the launch-file and the proposed architecture
3. A demonstration by controlling the robot directly from the keyboard, or by goal points in rviz.
