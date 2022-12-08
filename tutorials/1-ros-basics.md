# Linux and ROS Basics

The goal of this tutorials is to set up a workspace to control robots and to develop robotics programs.
We choose to work with ROS (the most used open middleware for robotics) on a Linux-Ubuntu computer (which are the best supported configuration).

This tutorials supose that you have an Ubuntu-Like 20.04 computer with configured Python and C++ APIs.
The tutorials relies on [https://docs.ros.org/en/foxy](ROS2 Foxy).


## Play with Linux.

Ubuntu is a distribution Linux (Operating System + Window Managment + Tools) derived from Debian distribution.

It is a classical graphical operating system.
You can explore the system and launch different program to test-it.

In the variety of program, we are mostly interested in the terminal emulator, permitting to manipulate our system directly from command (moving in the directory tree, read and organize files, execute script or programs, administer...).

- **gnome-terminal**: [https://help.gnome.org/users/gnome-terminal/stable](On help.gnome.org)

Explore the following command (i.e. what for, how to use it). A good way to do this is to first Google the command [https://duckduckgo.com/?q=command+man&t=newext&atb=v351-6&ia=web](example), then play with the command in our terminal.

- **man**, **ls**, **cp**, **mv**, **cat**, **rm**
- **source**, **apt**, **sudo**

To notice that `tabulation` permits autocompletion.

More commands: **egrep**, **find**, **ps**, ... [https://en.wikibooks.org/wiki/Linux_Guide/Linux_commands](Wikipedia is your friend).


## Play with ROS

The remander of the tutorial realies directly on ROS documentation [https://docs.ros.org/](docs.ros.org)
ROS is mainly composed by **Tools** and **Libraries**.

The **Tools** permit to configure and start a control architectecture, to explore this architecture and to visualise Data.
The **Libraries** offer an API (Application Programming Interface) to developpers who want to propose new program compliant with ROS.

### Tools

Start with [https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools.html](Beginner CLI-Tools tutorials) to get familliar with ROS tools.

Some responces to the questions you could have: 

- Yes we are in a classroom so we will work with : `ROS_LOCALHOST_ONLY=1`. You can check your bash configuration by editing the hidden fille at the user home dirctory: `gedit ~/.bashrc &`
- `bot` user is sudoer, and the password is `bot`.

### Libraries

Then follow the [https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries.html](Beginner Client-Libraries tutorials) by focusing on **Python** language.

In a first time we encurage to use **Python** language. The langage is less efficient at execution-time, but more easelly to handle in pedagogical purpose.
If you have a strong knowledge of **C++** you can use this language.
Otherwise, you have to always select and process for **Python** commands.

Also, you can work with classical editor (gedit on Ubuntu), but we encurage you to use `code` (VisualStudio Code) as devellopement environment.
