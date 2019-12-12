# Goal of this Tutorial

- Discover and use ROS tools: `roscore`, `rosrun`, `roslaunch` (launch files), `rqt_graph`, catkin, `rviz`
- Discover robotics simulators: **stage** (2d), **gazebo** (3d)

# ROS Prerequisites

- ROS correctly installed (cf. [Setup](setup.md))
- A *catkin workspace* directory (usually `~/catkin_ws`) in which you will create your catkin packages or install third party ones

# 2d Simulation with Stage 

Let's start with a first 2d ROS simulation with the [stage](http://wiki.ros.org/stage) simulator.

>Stage provides several sensor and actuator models, including sonar or infrared rangers, scanning laser rangefinder, color-blob tracking, fiducial tracking, bumpers, grippers and mobile robot bases with odometric or global localization. 

##Stage Installation

Check that stage is installed:

	```shell
	$ dpkg -l | grep stage
	ii  ros-melodic-stage        4.3.0-0bionic.20191008.154542    amd64    Mobile robot simulator http://rtv.github.com/Stage
	ii  ros-melodic-stage-ros    1.8.0-0bionic.20191008.204912    amd64    This package provides ROS specific hooks for stage 
	```

Install it otherwise:
	
	```shell
	$ sudo apt update
	$ sudo apt install ros-melodic-stage ros-melodic-stage-ros
	```

You can explore the files installed by these two packages:

	```shell
	$ roscd stage_ros
	```

##Stage Simulation with one Robot

[Source](http://wiki.ros.org/stage/Tutorials/SimulatingOneRobot)

	```shell
	$ roscore
	...
	# Launch stage in another shell
	$ rosrun stage_ros stageros $(rospack find stage_ros)/world/willow-erratic.world
	```

![stage simulation with one robot (blue square) equipped with a laser scanner](../files/SLAM/stage.png)

By pressing `r`, the stage simulation is rendered in 3d:
 
![3d-view on the same stage simulation](../files/SLAM/stage-3d.png)

You can use various ROS tools to analyse what is involved in this simulation.

	```shell
	# show the ROS graph (nodes + topics)
	$ rqt_graph

	# list the ROS topics and available data
	$ rostopic list

	# read the willow-erratic.world file
	$ gedit $(rospack find stage_ros)/world/willow-erratic.world
	
	# launch a pre-configured rviz
	$ rviz -d $(rospack find stage_ros)/rviz/stage.rviz
	```

[rviz](http://wiki.ros.org/rviz) is a very useful and versatile tool to visualize data that goes through topics. 

![rviz dispaying laser scans data published into the /base_scan topic by stage)](../files/SLAM/rviz_laserscan.png)

##Controlling the Simulated Robot

Install needed packets if needed:

	```shell
	$ sudo apt install ros-melodic-teleop-twist-keyboard
	```

Launch a simple node to control a robot using keyboard:

	```shell
	$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
	```
![teleop to control the robot into stage](../files/SLAM/teleop.png)

Use `rqt_graph` to see the graph of ROS nodes and the topics they use to communicate.

![rqt_graph shows ROS nodes and topics](../files/SLAM/rqt_graph.png)

Using `rostopic echo`, you see the data exchanged.

![Echo data published into the /cmdvel topic](../files/SLAM/cmdvel_echo.png)

Try to issue a command (`rostopic pub`) that mimic `teleop_twist_keyboard` by publishing data directly into the topic `/cmd_vel`, it should make the robot move into stage.

Find how to control a robot with a joypad (xbox, ps3/4 controllers).


##Re-Use your `dance` node

In the [move-to](move-to.md) tutorial, you wrote a `dance` node in Python that makes the robot move by publishing data into the topic `cmd_vel_mux`.

Launch this node (adapt it if necessary) to make moving the simulated robot in stage

#Create and Version control your first catkin package


1. Connect to your account on [gvipers](https://gvipers.imt-lille-douai.fr/) (IMT Lille Douai gitlab)

2. Create a repository named `larm1_slam` 

3. Clone your git repository into `~/catkin_ws/src` 

	```shell
	$ cd ~/catkin_ws/src
	$ git clone https://gvipers.imt-lille-douai.fr/XXXX/larm1_slam
	``` 
4. Create two files in this catkin package:

	```shell
	# create an empty CMakeLists.txt
	$ touch CMakeLists.txt
	
	# mandatory metadata of this catkin package
	$ cat > package.xml <<END
		<package>
		  <name>larm1_slam</name>
		  <version>0.0.1</version>
		  <description>
		      SLAM experiements
		  </description>
		  <maintainer email="luc@luc.sw">Luc</maintainer>
		  <license>BSD</license>
		</package>
		END
	```
	
	Note that the `catkin_create_pkg` command tool can generate these two files, however it then requires to move files around to version them on git.

5. Commit / push 
	
	```shell
	$ cd ~/catkin_ws/src/larm1_slam
	$ git status # to view modified files
	$ git add -A # to add all files into the staging area
	$ git commit -m "my slam package"
	$ git push
	```

	You can now check on the web interface of your git repository to see these committed files.
	
	There are tons of ressources on the Web to learn more about git. 
	You can find your own or have a look at this one: [learngitbranching](https://learngitbranching.js.org/).

<!-- The floorplan map is given (*dia.pgm*). -->
<!-- The directory `~/catkin_ws/src/larm1_mapping` should contains: -->
<!-- - Create a *launch file* named (`imt.launch`) -->

#**Your** first launch file

Into your `larm1_slam` catkin package, create a *launch file* named `robot_stage.world` that launches a full-fledge simulation using stage with one robot equipped with 2d laser ranger.
The [launch file documentation](http://wiki.ros.org/roslaunch).
Once this file finished, you should be able to launch everything with this single command line:

	```shell
	roslaunch larm1_slam robot_stage.launch
	```

`rviz` might be launched also using an optionnal argument to the launch file.
 
	```shell
	roslaunch larm1_slam robot_stage.launch rviz:=true
	```

When a launch file uses a simulator instead of a real robot, it is mandatory that ensure that ROS uses the simulator clock instead of the real clock of your machine (cf. [ROS clock documentation](http://wiki.ros.org/Clock)).
To achieve this, add this line into your launch file:

	```xml
	 <param name="/use_sim_time" value="true">
	```

##Advanced Stage 

You can customize your simulation by writing your own `.world` file and for example:
- change the map
- change the robot model (sensors, body shape, ...)
- add multiple robots into the scene
- ...

You can find `world` file examples into the `stage_ros` and `stage` catkin packages as well as read the [documentation](https://player-stage-manual.readthedocs.io/en/stable/WORLDFILES/).

# Gazebo Simulator

[Gazebo](http://gazebosim.org/) simulator makes it possible to rapidly test algorithms, design robots, perform regression testing, and train AI system using realistic scenarios. Gazebo is integrated with ROS (cf. [Gezebo ROS](http://wiki.ros.org/gazebo_ros_pkgs)) and supports various robots out of the box.

Gazebo is heavily used by the DARPA challenges (cf. [Wikipedia](https://en.wikipedia.org/wiki/Gazebo_simulator)).
You can see videos online ([example](https://www.youtube.com/watch?v=v6-heLIg85o)) and even load the maps and robot model that are available.

Follow [this tutorial](http://docs.fetchrobotics.com/gazebo.html) to simulate the *freight robot* (available at IMT Lille Douai). 

![Example of Gazebo simulation with a fetch robot](../files/SLAM/gazebo.png)

	```shell
	$ roslaunch fetch_gazebo playground.launch robot:=freight
	
	$ roslaunch teleop_twist_joy teleop.launch joy_config:=xbox
	```

At IMT Lille Douai, we also have turtlebot robots that you can simulate in gazebo:


	```shell
	roslaunch turtlebot_gazebo turtlebot_world.launch
	```

Write and commit a new launch file into your `larm1_slam` package that launches everything:

	```shell
	roslaunch larm1_slam freight_gazebo.launch rviz:=true xbox:=true
	```

<!-- gazebo models
	cd ~/.gazebo/
	rm -fr models
	wget https://bitbucket.org/osrf/gazebo_models/get/e6d645674e8a.zip
	unzip osrf*.zip 
	rm *.zip
	mv osrf* models
	-->
