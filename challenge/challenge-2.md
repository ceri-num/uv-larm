# Simultaneous Localization And Mapping

The goal of the challenge is to demonstrate the capability of a robot to localize and detect specific objects.

## Expected

The robot is positioned somewhere in a closed area (i.e. an area bounded with obstacles).

The robot moves continuously in this area by avoiding the obstacles (i.e. area limits and some obstacles randomly set in the area).

The robot build a map and localize itself in it. The map is saved at a regular frequency.

The robot detect _NukaCola_ bottle in the vision flux. A mesage is sent in a topic to state the detection.

## consigns

Each group commit the minimal required files in a specific `grp_'machine'` ros2 package inside their git repository.

Release: **Thursday morning of week-4**

### The required files:

* At the root repository, a `README.md` file in markdown syntax introducing the project.
* A directory `grp_'machine'` matching the ROS2 package where the elements will be found (`machine` mathes the name of the machine embedded on the robot).
* Inside the `grp_'machine'` package, a launch file `simulation.launch.py` starting the appropriate nodes for demonstrating in the simulation.
* Then, a launch file `tbot.launch.py` starting the appropriate nodes for demonstrating with a Turtlebot.

In simulations, we will work with the configuration set in `challenge-1.launch.py`.

## Criteria

Minimal:

1. The group follows the consigns (i.e. the repository is presented as expected)
2. The robot behavior is safe (no collision with any obstacles)
3. rviz2 is started and well configured in a second PC and displays the built map.
4. A `String` message is sent in a `detection` topic each time a bottle is front of the robot.

Optional:

5. a new launch file permits to start the robot into a navigation mode (localisation without mapping).
6. The `challenge2 tbot.launch.py` launch file may take an initial map.

## Evaluation protocol

cf. protocol proposed in challenge1, adapted to new expectations.
