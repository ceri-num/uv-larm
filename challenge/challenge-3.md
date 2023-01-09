# Efficient Exploration

The goal of the challenge is to demonstrate the capability of a robot to navigate in a cluttered environment and locate specific objects.

## Expected

The robot is positioned somewhere in a closed area (i.e. an area bounded with obstacles).

The robot moves continuously toward an unknown area by avoiding the obstacles to extend its knowledge.

The robot build a map and localizes itself in it. The map is saved at a regular frequency.

The robot detects _NukaCola_ bottle in the vision flux. A message is sent in a topic to state the detection and marker are positioned in the map (cf. [marker_msgs](http://wiki.ros.org/marker_msgs))

## consigns

Each group commit the minimal required files in a specific `challenge3` ros2 package inside their git repository.

Release: **Thursday morning of week-4**

### The required files:

* At the root repository, a `README.md` file in markdown syntax introducing the project.
* A directory `grp_'machine'` matching the ROS2 package where the elements will be found (`machine` mathes the name of the machine embedded on the robot).
* Inside the `grp_'machine'` package, a launch file `simulation.launch.py` starting the appropriate nodes for demonstrating in the simulation.
* Then, a launch file `tbot.launch.py` starting the appropriate nodes for demonstrating with a Turtlebot.

In simulations, we will work with the configuration set in `challenge-2.launch.py`.

## Criteria

Minimal:

1. The group follows the consigns (i.e. the repository is presented as expected)
2. The robot behavior is safe (no collision with any obstacles)
3. rviz2 is started and well configured in a second PC and display the built map.
4. It is possible to visualize a marker for detected bottles at the position of the bottle in the environment.

Optional:

5. The move strategy of the robot is efficient. The goal destination is wisely chosen and the trajectory to it is minimal.
6. The bottles are identified and the robot is capable of recognizing a bottle on a second passage.


## Evaluation protocol

cf. protocol proposed in challenge1, adapted to new expectations.