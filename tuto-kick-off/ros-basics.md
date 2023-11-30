# Linux and ROS Basics

The goal of this tutorial is to set up a workspace to control robots and to develop robotics programs.
We choose to work with ROS (the most used open middleware for robotics) on a Linux-Ubuntu computer (which are the best supported configuration).

This tutorial supposes that you have an Ubuntu-Like 22.04 computers with configured Python and C++ APIs.
The tutorial relies on [ROS2 Inron](https://docs.ros.org/en/iron).


## Play with Linux.

Ubuntu is a distribution Linux (Operating System + Window Managment + Tools) derived from Debian distribution.

It is a classical graphical operating system.
You can explore the system and launch different program to test-it.

In the variety of program, we are mostly interested in the terminal emulator, permitting to manipulate our system directly from command (moving in the directory tree, read and organize files, execute script or programs, administer...).

- **gnome-terminal**: on [help.gnome.org](https://help.gnome.org/users/gnome-terminal/stable)

Explore the following command (i.e. what for, how to use it). A good way to do this is to first Google the command ([example](https://duckduckgo.com/?q=command+man&t=newext&atb=v351-6&ia=web)), then play with the command in your terminal.

- **man**, **ls**, **cp**, **mv**, **cat**, **rm**
- **source**, **apt**, **sudo**

To notice that `tabulation` allows for autocompletion.

More commands: **egrep**, **find**, **ps**, ... ([Wikipedia is your friend](https://en.wikibooks.org/wiki/Linux_Guide/Linux_commands)).


## Play with ROS

The remainder of the tutorial relies directly on ROS documentation [docs.ros.org](https://docs.ros.org/)
ROS is mainly composed by **Tools** and **Libraries**.

The **Tools** permit to configure and start a control architecture, to explore this architecture and to visualize data.
The **Libraries** offer an API (Application Programming Interface) to developers who want to propose a new program compliant with ROS.

### Tools

Start with [Beginner CLI-Tools tutorials](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools.html) to get familiar with ROS tools.

Some responses to the questions you could have : 

- Yes we are in a classroom so we will work with : `ROS_LOCALHOST_ONLY=1`. You can check your bash configuration by editing the hidden file at the user home directory: `get ~/.bashrc &`
- `bot` user is sudoer, and the password is `bot`.

### Libraries

Then follow the [Beginner Client-Libraries tutorials](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries.html) by focusing on **Python** language.

In a first time, we encourage using **Python** language. The language is less efficient at execution time, but more easily to handle and to debug in pedagogical purpose.
If you have a strong knowledge of **C++** you can use this language.
Otherwise, you have to always select and process for **Python** commands.

Also, you can work with classical text editor (gedit on Ubuntu), but we encourage you to use `code` (VisualStudio Code) as development environment.
You can open code by specifiing a workspace from your terminal (and then open as many terminal you need in `code` directly): 

```console
cd
code ros2_ws
```
