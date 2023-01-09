# Random exploration of a Small Environment

The goal of the challenge is to demonstrate the capability of a robot to move in a cluttered environment with a possibility to see what the robot see.

## Expected

The robot is positioned somewhere in a closed area (i.e. an area bounded with obstacles).

The robot moves continuously in this area by avoiding the obstacles (i.e. area limits and some obstacles randomly set in the area).

The sensor data (scan, vision) and potentially other information (considered obstacles, frames,...) is visible on rviz in a remote a station PC.

The robot trajectory would permit the robot to explore the overall area. In other words, the robot will go everywhere (i.e. the probability that the robot will reach a specific reachable position in the area is equal to 1 at infinity times).

One first approach can be to develop a *ricochet robot* that changes its direction randomly each time an obstacle prevent the robot to move forward.

## consigns

Each group commit the minimal required files in a specific `challenge1` ros2 package inside their git repository.

Release: **Monday afternoon of week-3**

### The required files:

* At the root repository, a `README.md` file in markdown syntax introducing the project.
* A directory `grp_'machine'` matching the ROS2 package where the elements will be found (`machine` mathes the name of the machine embedded on the robot).
* Inside the `grp_'machine'` package, a launch file `challenge1_simulation.launch` starting the appropriate nodes for demonstrating in the simulation.
* Then, a launch file `challenge1_tbot.launch` starting the appropriate nodes for demonstrating with a Turtlebot.
* Finally, a launch file `challenge1_visualize.launch` starting the appropriate nodes including rviz2 to visualize in the PC-station side (i.e., a computer not connect to the robot).

In simulations, we will work with the configuration set in `challenge-1.launch.py`.

## Criteria

Minimal:

1. The group follows the consigns (i.e. the repository is presented as expected)
2. The robot behavior is safe (no collision with any obstacles)
3. rviz2 is started and well configured in a second PC connected with wifi (to notice that simulation will not provide for vision at this step).
4. The robot moves everywhere in it environment.

Optional:

5. An operator can take control at any time.
6. The robot movement is fluid (no stop), fast and the robot unique in every little passage.

## Evaluation protocol

Here the evaluation protocol applied.
It is highly recommended to process it yourself before the submission...

1. clone the group’s repository
2. Take a look to what is inside the repository and read the `README.md` file (normally it states that the project depends on `mb6-tbot`, make sure that `mb6-tbot` project is already installed aside).
3. make it: `colcon build` and `source` from the workspace directory.
4. Launch the simulation demonstration: `ros2 launch grp-'machine' challenge1_simulation.launch.py` and appreciate the solution.
4. Launch the visualization : `ros2 launch grp-'machine' visualize.launch.py` and appreciate the solution.
5. Stop everything.
6. Connect the computer to the robot, the hokuyo and the camera.
7. Launch the Turtlebot demonstration, and appreciate the solution.
8. Take a look to the code, by starting from the launchfiles.
