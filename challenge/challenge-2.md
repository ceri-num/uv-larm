# Mapping and Gathering

The goal of the challenge is to demonstrate the capability the robot has to map an environment and to retrieve specific objects on it.


## Expected

Considering that the appropriate data are streamed in ROS topics (odm, scam, camera/image, transformations...) the robot is capable of generating a map of its environment and the position in the map of objects of interest.

Technically, the data would be provided by a `rosbag` and the object to recognize and localize are bottles of nuka-cola, in any position possible.
The data flow is built with a moving robot (teleoperate for instance).

The map would be published in a `/map` topic all over the SLAM process and the position of the bottle in `/bottle`.
The `/bottle` caring visualization_msgs/Marker messages.
A message is published each time a bottle is detected by the robot.
The marker takes the form of a green Cube at the position of the bottle in the map frame.


* [visualization_msgs package](http://wiki.ros.org/visualization_msgs)
* [regarding rviz](https://wiki.ros.org/rviz/DisplayTypes/Marker)


## consigns

Each group commit the minimal required files in a specific `challenge2` git branch.

### The required files:

* a `README.md` file in markdown syntax introducing the project.
* a directory `grp-'color'` matching a ros package **and only this package**
* Inside the `grp-'color'` package, the code (python scripts or cpp sources) for the relevant nodes to the challenge.
* The launch file `challenge2.launch` starting the appropriate nodes for demonstrating the capability of the robot to map and to gather bottles when appropriate data are streamed in appropriate topics (`challenge2.launch` does not start the rosbag player).


## Criteria

Minimal:

1. The group follows the consigns (i.e. the repository is presented as expected)
3. The robot build a map in `/map` topic
3. The robot detect bottle and publish markers `/bottle` topic

Optional:

4. Information is returned to rviz (started automaticaly, with appropriate configuration).
5. The map is good shapped even in large environement.
6. The position of bottle in the map is precise.
7. The position of the bottle is streamed one and only one time in the `/bottle` topic.
8. All the bottle are detected (wathever the bottle position and the background).
9. Only the bottle are detected (even if similar object are in the environment).
10. A service permit to get all bottle positions


## Evaluation protocol

Here the evaluation protocol applied.
It is highly recommended to process it yourself before the submission...

1. clone the group’s repository
1. check out the appropriate branch `git checkout challenge2`
2. Take a look to what is inside the repository and read the `README.md` file (normally it states that the project depends on `mb6-tbot`, make sure that `mb6-tbot` project **is not included in the studient project** but already installed aside).
3. make it: `catkin_make` and `source` from the catkin directory.
4. Launch the demonstration: `roslaunch grp-color challenge2.launch`, echo the `bottle topic`, start rviz to visualize the map (if it is not automaticaly started)
5. Appreciate the solution with differents `rosbag`.
6. Stop everything.
7. Take a look to the code, by starting from the launchfiles.

<!--
* Example of [rosbag](https://partage.imt.fr/index.php/s/EH8o7dL5Jt7Nc4w) (you can use `unzip` command to... unzip the file before to use it)
-->

## Make your own rosbag:

Turn-on the robot in a first terminal:

```
roslaunch tbot_bringup start_teleop.launch
```

Start recording the minimal needed topics (Example without 3D data):

```
rosbag record -o larm /tf /tf_static /odom /scan /camera/color/image_raw /camera/aligned_depth_to_color/image_raw /mobile_base/commands/velocity
```

Return to your first terminal and control the robot with `ijkl` keys.

**To notice that rosbag are heavy files. It is not recommended to add those file to git versioning.**

To play the *rosbag* and map the environment, we will use the clock of the rosbag. First you have to specify the use of simulated clock on your launch file, then you will play the rosbag with clock option.

In the launchfile:

```xml
<param name="/use_sim_time" value="true" />
```

To ply a ROSBag:

```bash
rosbag play --clock yourRecordedFile.bag
```