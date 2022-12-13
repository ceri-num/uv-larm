# Simulation in ROS 

## Gazebo Simulator

[Gazebo](http://gazebosim.org/) is a 3d simulator.
It makes it possible to rapidly test algorithms, design robots, perform regression testing, and train AI system using realistic scenarios. Gazebo is integrated with ROS (cf. [Gazebo ROS](http://wiki.ros.org/gazebo_ros_pkgs)) and supports various robots out of the box.

Gazebo is heavily used by the DARPA challenges (cf. [Wikipedia](https://en.wikipedia.org/wiki/Gazebo_simulator)).
You can see videos online ([example](https://www.youtube.com/watch?v=v6-heLIg85o)) and even load the maps and robot model that are available.

## Gazebo Installation

Vérifiez que Gazebo est installé.

```console
$ dpkg -l | grep gazebo
ii  gazebo11                                        11.12.0-1~focal                      amd64        Open Source Robotics Simulator
ii  gazebo11-common                                 11.12.0-1~focal                      all          Open Source Robotics Simulator - Shared files
ii  gazebo11-plugin-base                            11.12.0-1~focal                      amd64        Open Source Robotics Simulator - base plug-ins
ii  libgazebo11:amd64                               11.12.0-1~focal                      amd64        Open Source Robotics Simulator - shared library
ii  libgazebo11-dev:amd64                           11.12.0-1~focal                      amd64        Open Source Robotics Simulator - Development Files
ii  ros-foxy-gazebo-dev                             3.5.3-1focal.20220829.174620         amd64        Provides a cmake config for the default version of Gazebo for the ROS distribution.
ii  ros-foxy-gazebo-msgs                            3.5.3-1focal.20221012.224922         amd64        Message and service data structures for interacting with Gazebo from ROS2.
ii  ros-foxy-gazebo-plugins                         3.5.3-1focal.20221021.150213         amd64        Robot-independent Gazebo plugins for sensors, motors and dynamic reconfigurable components.
ii  ros-foxy-gazebo-ros                             3.5.3-1focal.20221013.010602         amd64        Utilities to interface with Gazebo through ROS.
ii  ros-noetic-gazebo-dev                           2.9.2-1focal.20210423.224909         amd64        Provides a cmake config for the default version of Gazebo for the ROS distribution.
ii  ros-noetic-gazebo-msgs                          2.9.2-1focal.20221124.042608         amd64        Message and service data structures for interacting with Gazebo from ROS.
ii  ros-noetic-gazebo-plugins                       2.9.2-1focal.20221124.043536         amd64        Robot-independent Gazebo plugins for sensors, motors and dynamic reconfigurable components.
ii  ros-noetic-gazebo-ros                           2.9.2-1focal.20221124.042851         amd64        Provides ROS plugins that offer message and service publishers for interfacing with Gazebo through ROS.
ii  ros-noetic-gazebo-ros-control                   2.9.2-1focal.20221124.043459         amd64        gazebo_ros_control
ii  ros-noetic-gazebo-ros-pkgs                      2.9.2-1focal.20221124.061712         amd64        Interface for using ROS with the Gazebo simulator.
ii  ros-noetic-hector-gazebo-plugins                0.5.4-1focal.20221124.043534         amd64        hector_gazebo_plugins provides gazebo plugins from Team Hector.
ii  ros-noetic-velodyne-gazebo-plugins              1.0.12-2focal.20221124.043606        amd64        Gazebo plugin to provide simulated data from Velodyne laser scanners.
```

Installez éventuellement les paquets manquants avec `sudo apt install <nom_paquet>`

## Testez Gazebo

```console
$ rosify2
(ros2) $ ros2 launch gazebo_ros gazebo.launch.py world:=
```
### Simulating a Robot


```console
(ros2) $ 
```

![Gazebo simution with a tbot](../files/SLAM/stage.png)

Test the main ROS2 tools:

```console
# show the ROS graph (nodes + topics)
(ros2) $ rqt_graph

# list the ROS topics and available data
(ros2) $ ros2 topic list
```

>Question: In which topic laser scans are published?

[rviz](http://wiki.ros.org/rviz) is a very useful and versatile tool to visualize data that goes through topics. 
Visualize laser scans using `rviz`:

```console
# launch a pre-configured rviz
(ros1) $ rviz -d $(rospack find stage_ros)/rviz/stage.rviz
```

To visualise the laser scans in `rviz` ensure that `Global Option` / `Fixed frame` is correctly setup to: `base_link`.

>Question: why is this important? (hint: check your `tf` using `rosrun tf view_frames`)

![rviz)](../files/SLAM/rviz_laserscan.png)

## Controlling the Simulated Robot

Launch a simple node to control a robot using keyboard:

```console
(ros2) $ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/demo/cmd_demo
```

Use `rqt_graph` to see the graph of ROS nodes and the topics they use to communicate.
Why you cannot control the robot?

Create a new catkin package named `my_teleop` using the `catkin_create_pkg` command line tool (cf. [catkin package creation](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)).

```console
(ros1) $ cd ~/catkin_ws/src
(ros1) $ catkin_create_pkg my_teleop std_msgs rospy
```

Then, write your __own__ launch file to control the robot by copying and modifying `keyboard_teleop.launch` from the package __turtlebot_teleop__.
You should have the nodes and topics as depicted below.

![teleop to control the robot into stage](../files/SLAM/teleop.png)

Use `rqt_graph` to see the graph of ROS nodes and the topics they use to communicate.

![rqt_graph shows ROS nodes and topics](../files/SLAM/rqt_graph.png)

Using `rostopic echo`, you see the data exchanged.

![Echo data published into the /cmdvel topic](../files/SLAM/cmdvel_echo.png)

Try to issue a command (`rostopic pub`) that mimic keyboard teleoperation by publishing data directly into the topic `/cmd_vel`, it should make the robot move into stage.

Find how to control a robot with a joypad (xbox, ps3/4 controllers).

## A program to control the simulated robot

Write a publisher node in Python to make moving the simulated robot in stage by publishing data into the topic `cmd_vel`.

(cf. Tutorial: Move The Robot - with some adaptations...)

# Versioning

Document your package by adding a Readme inside (mandatory).
Commit and push your new catkin package into your git repository.

```console
(ros1) $ cd ~/catkin_ws/src/my_teleop
(ros1) $ git status # to view modified files
(ros1) $ git add -A # to add all files into the staging area
(ros1) $ git commit -m "my slam package"
(ros1) $ git push
```

You can now check on the web interface of your git repository to see these committed files.

There are tons of ressources on the Web to learn more about git.
You can find your own or have a look at this one: [learngitbranching](https://learngitbranching.js.org/).


# **Your** first SLAM launch file

Create a `larm1_slam` catkin package, create a *launch file* named `robot_stage.launch` that launches a full-fledge simulation using stage with one robot equipped with a 2d laser ranger.
The [launch file documentation](http://wiki.ros.org/roslaunch).
Once this file finished, you should be able to launch everything with this single command line:

```console
(ros1) $ roslaunch larm1_slam robot_stage.launch
```

`rviz` might be launched also using an optionnal argument to the launch file.

```console
(ros1) $ roslaunch larm1_slam robot_stage.launch rviz:=true
```

When a launch file uses a simulator instead of a real robot, it is mandatory to ensure that ROS uses the simulator clock instead of the real clock of your machine (cf. [ROS clock documentation](http://wiki.ros.org/Clock)).
To achieve this, add this line into your launch file:

```xml
	<param name="/use_sim_time" value="true" />
```


cd ~/ros2_ws/src
ros2 pkg create my_package --build-type ament_cmake --dependencies rclcpp
ros2 pkg create larm_corrections --build-type ament_python 
mkdir -p ~/ros2_ws/src/my_package/launch
touch ~/ros2_ws/src/my_package/launch/dolly.launch.py

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_dolly_gazebo = get_package_share_directory('dolly_gazebo')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(pkg_dolly_gazebo, 'worlds', 'dolly_empty.world'), ''],
            description='SDF world file',
        ),
        gazebo
    ])
```

## Final Exercice

1. create a bag file that only record `/cmd_vel` while you are teleoperating a robot in a simulated map
2. then replay this bag file on a fresh simuation

You can now reproduce the *same* experimentation multiple times and possible make some variations.

<!-- gazebo models
	cd ~/.gazebo/
	rm -fr models
	wget https://bitbucket.org/osrf/gazebo_models/get/e6d645674e8a.zip
	unzip osrf*.zip
	rm *.zip
	mv osrf* models
	-->
