# ROS setup

The goal of this exercise is to set up a computer and a software environment to control robots and to develop robotics programs.
We choose to work with ROS (the most used open robotic middleware) on a Linux-Ubuntu computer (which are the best supported configuration).

The required configuration at the end is based on:
  - Ubuntu 18.04
  - ROS Melodic
  - Your favorite development environment for Python and C++.

# Installing Ubuntu

Ubuntu is a fork of the Debian project, a Linux-based desktop operating system.

  - Official website : <https://ubuntu.com/>
  - French community : <https://www.ubuntu-fr.org/>

Notice that, ubuntu can be installed in double boot mode in parallel to a Microsoft OS on your personal computer.
It is not recommended to use Ubuntu+ROS in a virtual machine (the performances would be  poor).

### To-do:
  - Install ubuntu 18.04 LTS (Long Term Supported version) from live USB-Key
  - use all the hard disk
  - Configure "bot" username and "bot" password.
  - configure network ("robot des mines")
  - login and upgrade your installation
```bash
sudo apt update
sudo apt upgrade
```

# Installing ROS

ROS could be installing directly why 'apt' tool while configuring the appropriate repositories.

<http://wiki.ros.org/melodic/Installation/Ubuntu>

### To-do:
  - Install ROS
  - Play Beginner tutorials from **1** to **6**
    * <http://wiki.ros.org/ROS/Tutorials>

To notice that ROS support two systems to manage a ROS project. It is recommended to used Catkin rather than ROSBuilt.


## Configure your development environment

ROSprovide tools to compile and include a project in the ROS ecosystem. The most recent (**Catkin**) is based on **CMake**.
However it is not a complete IDE and you will require your favorite tools:
  - editor (geany, atom, visual studio code,...)
  - versioning (git,...)
  - multi-terminal (terminator,...)
  - system (ssh, curl,... )

### To-Do
  - Install and configure your favorite Dev. tools for Python and C++
  - Play tutorials:
    * [Installing and Configuring Your ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
    * [Navigating the ROS Filesystem](http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem)
  - Install and configure your favorite project management tool.
  - Configure a shared and versioned workspace for your team.
