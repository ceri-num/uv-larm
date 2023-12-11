# 02 - Move a robot

This tutorial aims to take control of a __tbot__  robot.
A tbot is a turtlebot2 robot (based on a kobuki platform)
equipped with a laser range to navigate in a cluttered environment
and a camera to recognize objects.


## Connect the tbot:

If it is not yet the case,
the machine connecting the robot and its sensors have to be configured accordingly to the [IMT MobiSyst tbot](https://bitbucket.org/imt-mobisyst/pkg-tbot)

You will have then a ROS-2 WorkSpace including __tbot__ meta-package (`pkg-tbot`) itself including several ROS packages.

- Verrify it:

```console
cd ~/mb6_space
ls 
ls pkg-tbot
```

- Build the packages:

```console
cd pkg-tbot   # enter le project directory
git pull      # download the last version
cd ..         # return on your ros workspace (mb6_space)
colcon build  # build...
```

- Update your shell environment variables:

```console
source bin/run-command.bash
```

- Connect the tbot base, the laser and launch the control nodes (a ros launch file permits to start a collection of nodes with a desired configuration):

```console
ros2 launch tbot_node minimal_launch.yaml
```

Into another, you can explore the existing node (`rqt_graph`) or topics (`ros2 topic list`)

Finally, you can try to take control in a third terminal:

```console
ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/multi/cmd_teleop
```

Close everything with `ctrl-c`.

The teleop publishes a [geometry_msgs](https://index.ros.org/r/common_interfaces/github-ros2-common_interfaces/) `twist` message.
It is composed of two vectors $(x, y, z)$, one for linear speed $(m/s)$, and the second for angular speed $(rad/s)$.
However a [nonholonomic](https://en.wikipedia.org/wiki/Nonholonomic_system) ground robot as the **tbot** would move only on `x` and turn only on `z`.
It is not as free as a drone, you can echo the messages into a 4th terminal.

- Try to control the robot with `ros2 topic pub` command publishing in the navigation topic (`/multi/cmd_nav`).

Tbot integrate a [subsumption](https://en.wikipedia.org/wiki/Subsumption_architecture) multiplexer.
The node listens different topics with different priorities (by default: `/multi/cmd_nav` and `/multi/cmd_telop`) and filter the appropriate commands to send to the robot.
The topics `cmd_nav` and `cmd_telop` stend for autonomous navigation and operator teleoperate.
The human operator has a higher priority and make the multiplexer to trash the `cmd_nav` commands.

## Our first node

The goal is to create a process connecting a topic and publishing velocities as a twist message:

- First we have to create a file for our script.

```
touch tuto_move.py
code tuto_move.py
```

- The script depends from several ROS2 ressources.

```python
import rclpy                  # core ROS2 client python librairie
from rclpy.node import Node   # To manipulate ROS Nodes

print("tuto_move :: START...")
```

- You can try that everything is well imported:

In a shell:

```sh
python3 tuto_move.py
```

- Next move consists in making a node and an infinite loop 

In `tuto_move.py` add: 

```python
def main():
    rclpy.init()     # Initialize ROS2 client
    myNode= Node('move_node') # Create a Node, with a name         

    # Start the ros infinit loop with myNode.
    while True :
        rclpy.spin_once( myNode, timeout_sec=0.1 )
        print("Running...")

    # At the end, destroy the node explicitly.
    move.destroy_node()

    # and shut the light down.
    rclpy.shutdown()

    print("tuto_move :: STOP.")

# activate main() function,
# if the file is executed as a script (ie. not imported).
if __name__ == '__main__':
    # call main() function
    main()
```

- You can try that a node is created :

```sh
# In a 1st shell:
python3 tuto_move.py

# In a 2d shell:
ros2 node list
```

## Move Script

We want our node to publish velocities continuously.
To do that we have to import the type of message we have to send, to instantiate a publisher, and to publish messages.

You have to add the next pieces of codes at the appropriate location in the `tuto_move.py` script:

```python
# Message to publish:
from geometry_msgs.msg import Twist

# Initialize a publisher:
velocity_publisher = node.create_publisher(Twist, '/multi/cmd_nav', 10)

# publish a msg
velo = Twist()
velocity_publisher.publish(velo)
```

- To verify that everythong goes right:

```sh
# In a 1st shell:
python3 tuto_move.py

# In a 2d shell:
ros2 node list
ros2 topic list
ros2 topic echo /multi/cmd_nav
```

Basicly it echoes 0 speed vectors. 
To investigate what is a `Twist` you can ask to `ros2 interface` or search at the package location (`/opt/ros/iron/share`) (or search the documentations).

```sh
ros2 interface list | Twist
ros2 interface show geometry_msgs/msg/Twist
```

```sh
ls /opt/ros/iron/share
cat /opt/ros/iron/share/geometry_msgs/msg/Twist.msg
cat /opt/ros/iron/share/geometry_msgs/msg/Vector3.msg
```

In case of [nonhonolome mobile](https://en.wikipedia.org/wiki/Nonholonomic_system) ground robot,
the control uses two speed values, one lienar (x) and one angular (z).

SO, for a circling behavior:

```python
# publish a msg
velo = Twist()
velo.linear.x= 0.2   # meter per second
velo.angular.z= 0.14 # radian per second
velocity_publisher.publish(velo)
```

At this point, your robot should move...
To notice that you can also test your code on `turtlesim` by changing the name of the velocity topic.
