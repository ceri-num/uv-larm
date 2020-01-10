# Challenge 1: Move to a known pose

**Setup**: The robot starts in the known `positionA` in the arena. 

**Objective**: The robot must move autonomously to the known `positionB` in the arena.

# Challenge 2: Move to an unknown pose

**Setup**: The robot starts in the known `positionA` in the arena. 

**Objective**: The robot must move autonomously to the unknown `positionC` in the arena.

# Challenge 3: Explore a known environment

**Setup**: The robot starts in the known arena. 

**Objective**: The robot should produce a map of the arena. You may teleoperate the robot.

# Challenge 4: Teleoperated exploration of an unknown environment 

**Setup**: The robot starts in an unknown arena. 

**Objective**: The robot should produce a map of the arena. You may teleoperate the robot.

# Challenge 5: Autonomous exploration of an unknown environment 

**Setup**: The robot starts in an unknown arena. 

**Objective**: The robot should **autonomously** produce a map of the arena autonomously.

# Challenge 6: Autonomous exploration of an unknown environment 

**Setup**: The robot starts in an unknown arena. 

**Objective**: The robot should **autonomously** produce a map of the arena autonomously.

# Challenge 7: Detect bottles in a known environment

**Setup**: The robot starts in a known arena. 

**Objective**: The robot should move in the arena autonomously or not and detect nuka cola bottles. 
Each time a bottle is detected, the position of the bottle in the map should be published in a specific ROS topic named `/bottle` with the data type `visualization_msgs/marker`. `rviz` should display this `/bottle` topic.

# Challenge 8: Detect bottles in an unknown environment

same as challenge 7 but in an unknown environment.

# Deliverables

- URL of your git repository with **all** your **cleaned** code 
- A short README that:
-- provides at least one `roslaunch` command to launch a stage simulation with your code
-- gives the URL of your video(s) shared on a (partage.imt-lille-douai.fr) or dropbox
