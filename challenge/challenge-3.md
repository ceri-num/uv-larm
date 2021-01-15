# Autonomous Exploration

The goal of the challenge is to demonstrate the capability the robot has to move in a cluttered environment.

## Preparation

Install the [LARM simulation workspace]() in place of `simulation_ws`.

## Expected

A launch file `exploration.launch` in the student package in `catkin_ws/src/student_package` that configure the robot control architecture in a way that:

* The robot moves autonomously in the environement.
* The robot build a map while navigating.
* The robot communicate in a topic `/bottle` the position of bottles, each time the robot recognizes the appropriate object.

The initial position of the robots can be random in the environment.

## Demonstration protocol

1. Launch the simulation configuration: `roslaunch larm challenge-3.launch`
2. Launch the control architecture: `roslaunch student_pkg exploration.launch`
3. Launch rviz

## In the video

1. A presentation of the challenge
2. A presentation of the launch-file and the proposed architecture
3. A demonstration
