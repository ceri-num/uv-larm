# Efficient Exploration

The goal of the challenge is to demonstrate the capability a robot has to navigate in a cluttered environment and locate specific objects.
The localization requires to build a map of the environment.

## Expected

The robot is positioned somewhere in a closed area (i.e. an area bounded with obstacles).

The robot moves continuously in this area by avoiding obstacles.

The robot knowledge is extended with new incoming data.
In other words, the robot build a map and localizes itself in it.

The robot detects _NukaCola_ bottle in the vision flux.
Messages are sent in topics
one to state the detection
and another one to mark the position in the map (cf. [marker_msgs](http://wiki.ros.org/marker_msgs))

Experiments can be performed with 2 computers, one on the robot (Control PC) and a second for visualization and human control (Operator PC).

## consigns

Each group commit the minimal required files in a specific `grp_'machine'` ros2 package inside their git repository.

Release: **Monday of week-4, Monday 22 of January**

### The required files:

* At the root repository, a `README.md` file in markdown syntax introducing the project.
* A directory `grp_'machine'` matching the ROS2 package where the elements will be found (`machine` matches the name of the machine embedded in the robot).
* Inside the `grp_'machine'` package, a launch file `simulation_launch` starting the appropriate nodes for demonstrating in the simulation.
* Then, a launch file `tbot_launch` starting the appropriate nodes for demonstrating with a Turtlebot.
* Finally, a launch file `operator_launch` start a visualization + control solution.

In simulations, we will work with the configuration set in `challenge-2.launch.py`.

## Criteria

Minimal:

1. The group follows the consigns (i.e. the repository is presented as expected)
2. The robot behavior is safe (no collision with any obstacles)
3. rviz2 is started and well configured in a second PC and display the built map.
4. It is possible to visualize a marker for detected bottles at the position of the bottle in the environment. The bottles are identified with a number and the robot is capable of recognizing a bottle on a second passage (it should be easy to count the bottle in the map or by reading the topic).
5. The bottle detection detects all the bottles, in any position but with robustness to false positives.

Optional (the order does not matter):

- The robot movement is oriented toward the unknown areas to speed up the exploration.
- Processes are clearly established (start and stop the robot, save the map, get the lists of bottles, set the xp in pause, ...)
- Developed nodes are based on ROS2 Parameters (for speed, obstacle detections, ...) 
- The Kobuki features are integrated to the scenario (robot button, contact to the ground, void detection, bips,...)
- The list is not exhaustive, be inventive !

The `challenge2 tbot.launch.py` launch file may take an initial map.


## Evaluation protocol

Here the evaluation protocol to apply.
It is highly recommended to process it yourself before the submission...

1. Clone/update the group’s repository on both machines (Control and Operator)
2. Take a look to what is inside the repository and read the `README.md` file (normally it states that the project depends on `mb6-tbot`, make sure that `mb6-tbot` project is already installed aside).
3. Build it: `colcon build` and `source` from the workspace directory.
3. Set the appropriate ROS configuration (domain ID, etc.).
4. Launch the simulation demonstration: `ros2 launch challenge1 simulation.launch.py` and appreciate the solution.
5. Stop everything.
6. Configure the computers for working with the Tbot.
7. Launch the Tbot demonstration, and appreciate the solution.
8. Take a look to the code, by starting from the launch files.
