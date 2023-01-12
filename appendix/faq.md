# Frequent Asked Question...

## ROS2 tbot driver

> How to get rid of ROS1 ;-)

__Install ROS2 tbot driver__:

```
cd ~/ros2_ws/pkg-tbot
git pull
./script/install-kobuki_ros.sh

cd ~/ros2_ws
colcon build
```

__Launch ROS2 tbot driver__:

```
# base only
ros2 launch tobt_start base.launch.py

# base + laser
ros2 launch tobt_start minimal.launch.py

# base + with laser + camera
ros2 launch tobt_start full.launch.py

ros2 topic list
```

## Fix malformed packets

Info there: https://github.com/kobuki-base/kobuki_core/commit/2bc11a1bf0ff364b37eb812a404f124dae9c0699

```
sudo cp kobuki_core/60-kobuki.rules /lib/udev/rules.d/
```

Then unplug / replug the robot.
To ensure it worked, the followwing command should display 1:

```
cat /sys/bus/usb-serial/devices/ttyUSB0/tty/ttyUSB0/device/latency_timer
```

## How to install Ubuntu ?

Ubuntu is a fork of the Debian project, a Linux-based desktop operating system.

  - Official website : <https://ubuntu.com/>
  - French community : <https://www.ubuntu-fr.org/>

Notice that, ubuntu can be installed in double boot mode in parallel to a Microsoft OS on your personal computer.
It is not recommended to use Ubuntu+ROS in a virtual machine (the performances would be  poor).

### To-do:
  - Install Ubuntu **20.04 LTS** (Long Term Supported version) from live USB-Key
  - Ideally, use all the hard disk (you can split the disk in advance for [double-boot install](https://help.ubuntu.com/community/WindowsDualBoot))
  - Configure "bot" username and "bot" password.
  - Configure network
  - Login and upgrade your installation

```bash
sudo apt update
sudo apt upgrade
```

## There is no Wifi on my dell-xps13 ?

1. Connect with cable
2. Get the appropriate drivers: [killer driver](https://support.killernetworking.com/knowledge-base/killer-ax1650-in-debian-ubuntu-16-04/)

```bash
sudo add-apt-repository ppa:canonical-hwe-team/ backport-iwlwifi
sudo apt-get update
sudo apt-get install backport-iwlwifi-dkms
```

3. reboot

## Remove password asking for docker commands

```
sudo echo "\n%sudo   ALL=(ALL) NOPASSWD: /usr/bin/docker\n" >> /etc/sudoers
```

## Catkin_create_pkg - invalid email ?

you can use the `-m` option to force an author name.

```bash
catin_create_pkg -m AuthorName package_name [dependencies...]
```

## `opencv_createsamples`

The latest OpenCV does not include  `opencv_createsamples`.
Let's compile an older version (~5 min on labs PC).

```bash
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout 3.4.17
mkdir build
cd build
cmake -D'CMAKE_BUILD_TYPE=RELEASE' ..
make -j8

ls -l bin/opencv_createsamples
```

## How to get an aligned depth image to the color image ?

 you can use the `align_depth:=true` ROS parameter. The aligned image is streamed in a specific topic. (tks Orange group)

```bash
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```

## ROS1 vs ROS2 commands cheatsheet

|ROS1   | ROS2   |
|---|---|
| `rostopic list`  | `ros2 topic list`  |
| `rqt_graph`  | `rqt_graph`  |
| `rviz`  | `rviz2`  |
| `rosrun tf view_frames` | `ros2 run tf2_tools view_frames.py` |
| | `colcon build --packages-select my_package` |
| | `colcon build --symlink-install` |

## `.bashrc` ROS additions

```consoleell
# ROS
export ROS_LOCALHOST_ONLY=1
export PS1="\${ROS_VERSION:+(ros\$ROS_VERSION) }$PS1"
alias rosify1="source /opt/ros/noetic/setup.bash && source $HOME/ros1_ws/devel/setup.bash"
alias rosify2="source /opt/ros/foxy/setup.bash && source $HOME/ros2_ws/install/setup.bash"
```

## Flash kobuki 

https://kobuki.readthedocs.io/en/devel/firmware.html#linux

