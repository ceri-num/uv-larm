# Navigation in a Small Environment

The goal of the challenge is to demonstrate the capability the robot has to move in a cluttered environment.

## Preparation

Install the [LARM simulation workspace](https://github.com/ceri-num/LARM-RDS-Simulation-WS) in place of `simulation_ws`.

## Expected

A launch file `navigation.launch` in the student package in `catkin_ws/src/student_package` that configure the robot control architecture in a way that:

* The robot wait target position to reach in a `/goal` topic.
* The robot moves toward the last provided position and stop when reached.
* The robot movement is smooth and avoid the present obstacles.

## Demonstration protocol

1. Launch the simulation configuration: `roslaunch larm challenge-1.launch`
2. Launch the control architecture: `roslaunch student_pkg navigation.launch`
2. Launch rviz
3. Provide a succession of goal destinations in `/goal` topic throught rviz as pose messages. The poses are provided in `odom` frame, successively after the robot stop to the previous goal location.

The environnement configuration can be random, but the destination goal are provided manually.

## In the video

1. A presentation of the challenge
2. A presentation of the launch-file and the proposed architecture
3. A demonstration
