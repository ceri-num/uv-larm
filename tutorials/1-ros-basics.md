# 01 - ROS setup

The goal of this exercise is to set up a computer and a software environment to control robots and to develop robotics programs.
We choose to work with ROS (the most used open middleware for robotics) on a Linux-Ubuntu computer (which are the best supported configuration).

The required configuration at the end is based on:
  - Ubuntu 20.04 (Or the geek and gamer vertion [POP!_os](https://pop.system76.com/))
  - ROS Neotic
  - Your favorite development environment for Python and C++.

# Installing Ubuntu

Ubuntu is a fork of the Debian project, a Linux-based desktop operating system.

  - Official website : <https://ubuntu.com/>
  - French community : <https://www.ubuntu-fr.org/>

Notice that, ubuntu can be installed in double boot mode in parallel to a Microsoft OS on your personal computer.
It is not recommended to use Ubuntu+ROS in a virtual machine (the performances would be  poor).

### To-do:
  - Install Ubuntu 20.04 LTS (Long Term Supported version) from live USB-Key
  - Ideally, use all the hard disk (you can split the disk in advance for [double-boot install](https://help.ubuntu.com/community/WindowsDualBoot))
  - Configure "bot" username and "bot" password.
  - Configure network
  - Login and upgrade your installation

```bash
sudo apt update
sudo apt upgrade
```

# Play with Linux.

Ubuntu is a distribution Linux (OS + Window Managment + tools) derived from Debian distribution.

It is a classical graphical operating system. You can explore the system and launch different program to test-it.

In the variety of program, we are mostly interested in the terminal emulator, permitting to manipulate our system directly from command (moving in the directory tree, read and organize files, execute script or programs, administer...).

- **gnome-terminal**: https://help.gnome.org/users/gnome-terminal/stable

In a terminal, explore the following command (i.e. what for, how to use it):

- **man**
- **ls**
- **cp**
- **mv**
- **cat**
- **rm**
- **apt**
- **sudo**

# Installing ROS

ROS could be installing directly with `apt` tool while configuring the appropriate repositories.

<http://wiki.ros.org/ROS/Installation>


# Play with ROS

ROS documentation is developed by the community on a wiki: http://wiki.ros.org.
This wiki includes the documentation on core ROS as on a large variety of ROS drivers and modules shared by the community.

## First contact

As a first step, the beginner tutorials permit to visit the main element of ROS.
To notice that ROS support two systems to manage a ROS project.
This lecture is focused on `Catkin`, so you can avoid the parts reserved to `rosBuild`.

Execute beginner tutorials from **1** to **6**.

## Develop a first ROS program

ROS provides tools to compile and include a project in the ROS ecosystem.
The most recent (**Catkin**) is based on **CMake**.
However it is not a complete IDE and you will require your favorite tools:

- editor (geany, atom, visual studio code,...)
- versioning (git,...)
- multi-terminal (terminator,...)
- system tools (ssh, curl,... )

- Install and configure your favorite Dev. tools for *Python* and *C++*
  *  `visual studio code` is a gode option: `sudo apt install code`
- Play tutorials from **7** to **20**
  * <http://wiki.ros.org/ROS/Tutorials>
