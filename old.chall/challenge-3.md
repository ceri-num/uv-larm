# Autonomous Exploration

The goal of the challenge is to demonstrate the capability the robot has to explore autonomously an unknown environment and to retrieve specific objects on it.


## Expected

The goal is to start a robot in an unknown environment corner, to let him explore the area and return with a map and an estimation of the bottle location. 
The behavior of the robot is expected as autonomous as possible.

The map would be published in a `/map` topic all over the SLAM process and an operator can publish goals to reach for the robot. The robot moves by avoiding the obstacle toward the goal position.
Finally, a `/bottle` topic caring visualization_msgs/Marker messages of the recognized bottle.

## consigns

Each group commit the minimal required files in a specific `challenge3` git branch.

### The required files:

* a `README.md` file in markdown syntax introducing the project.
* a directory `grp_'color'` matching a ros package **and only this package**
* Inside the `grp_'color'` package, the code (python scripts or cpp sources) for the relevant nodes to the challenge.
* The launch file `challenge3_simulation.launch` starting the appropriate nodes for demonstrating the capability of the robot to map an unknown environment autonomously based on `larm_challenge challenge1.launch` (no bottle recognition).
* The launch file `challenge3_tbot.launch` starting the appropriate nodes for demonstrating the capability of the robot to explore and retrieve the bottle on an unknown environment (on the top of the `tbot_bringup start.launch` launch file).
* the `challenge3_tbot.launch` also starts rviz, and an image viewer to appreciate the map construction and the bottle detection.
* In the `README.md` file, a reference to a video.

### In the video

1. A presentation of the challenge, it context and the authors.
2. A presentation of the launch-file and the proposed architecture
2. A presentation of the specificity of the solution
3. A demonstration

## Criteria

Minimal:

1. The group follows the consigns (i.e. the repository is presented as expected)
2. The robot navigate to goal position provided with `rviz`
3. The robot build a map in `/map` topic
4. The robot detects 1st-version-bottle (orange one) and publish markers in `/bottle` topic at the position of detected bottles.

Optional:

5. The robot detect 2d-version-bottle (black one)
6. There is no need to publish goal positions. The robot is autonomous to achieve its mission.
7. Any suggestions provided by the group are welcome.

## Evaluation protocol

Here the evaluation protocol applied.
It is highly recommended to process it yourself before the submission...

1. clone the group’s repository
2. check out the appropriate branch `git checkout challenge3`
3. `README.md` also mention if the group targeted (and how) some of the optional features.
4. make it: `catkin_make` and `source` from the catkin directory.
5. Launch the demonstration: `roslaunch grp-color challenge3_simulation.launch`, provide successively some goals (if required) and appreciate the resulting map.
6. Launch the demonstration: `roslaunch grp-color challenge3_tbot.launch`, echo `\bottle` topic, provide successively some goals (if required) and appreciate the resulting map. 
7. Take a look to the code, by starting from the launch files.

* Only the `tbot` challenge matching the `challenge3_tbot.launch` launch file would be evaluated at the demonstration day.
