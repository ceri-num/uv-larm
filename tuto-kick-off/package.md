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

- Connect the tbot base and launch the control nodes (a ros launch file permits to start a collection of nodes with a desired configuration):

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
import rclpy                         # core ROS2 client python librairie
from rclpy.node import Node          # To manipulate ROS Nodes

print("tuto_move :: START...")
```

- You can try that everything is well imported:

In a shell:

```sh
python3 tuto_move.py
```

- Next move consists in making an infinite loop 

In `tuto_move.py` add: 

```python
def main():
    rclpy.init()     # Initialize ROS2 client
    myNode= Node('move_node') # Create a Node, with a name         

    # Start the ros infinit loop with myNode.
    while True :
        rclpy.spin( myNode, timeout_sec=0.1 )
        print("Running...")

    # At the end, destroy the node explicitly.
    move.destroy_node()

    # and shut the light down.
    rclpy.shutdown()

    print("tuto_move :: STOP.")

# If the file is executed as a script (ie. not imported).
if __name__ == '__main__':
    # call main() function
    main()
```

- You can try that a node is created :

In a shell:

```sh
# In a 1st shell:
python3 tuto_move.py

# In a 2d shell:
ros2 node list
```

## Move Script

We want that our node publishes velocities at regular rate.

To do that we have to import the type of mesage we wana to send.

You have to add the next pieces of codes at the appropriate location in the `tuto_move.py` scrip:

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

# In a 2d shell:
ros2 topic list

# In a 2d shell:
ros2 topic echo /multi/cmd_nav
```

Basicly it is 0 speed vectors. 
To investigate what is a `Twist` you can ask to `ros2 interface` or search at the package location (`/opt/ros/iron/share`).

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

For a circling behavior:

```python
# publish a msg
velo = Twist()
velo.linear.x= 0.2   # meter per second
velo.angular.z= 0.14 # radian per second
velocity_publisher.publish(velo)
```

Well your robot should move...

To notice that you can also test your code on `turtlesim` by changing the name of the velocity topic.






## Move node in a tutos pkg

The idea now is to create a node that will control the robot accordingly to our expectation.
For that we will create a python ros package and a new node in this package to send velocity in the appropriate topic.

This tutorial is adapted from [official ros2 tutorial](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries.html).

First create a new package `tuto_move` in the workspace directory of your workspace (ie. in `mb6-space`, aside of `pkg-tbot`).

```console
cd ros2_ws
ros2 pkg create --build-type ament_python tuto_move
```

Inside your new package create a node `move_1m` at the appropriate location that will integrate the code for moving the tbot 1 meter forward.

```console
touch tuto_move/tuto_move/move_1m.py
```

Edit this new file with a very simple code in order to test the packages:

```python
def main():
    print('Move move move !')

if __name__ == '_main__' :
    main()
```

For more detail on those manipulation, you can return to the [ros tutorial](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html).

If you have troubles in understanding this python code: [functions](https://www.w3schools.com/python/python_functions.asp), [Top-level code environment](https://docs.python.org/3/library/__main__.html).

Then, you have to inform ROS for the existence of your new node.
In your package `setup.py` file, add your node in the `console_scripts` [list](https://www.w3schools.com/python/python_lists.asp) of the `entry_points` [dictionnary](https://www.w3schools.com/python/python_dictionaries.asp).
The new list item would look like `'move_1m = tuto_move.move_1m:main'`.

Actually, you have only one entry point:

```python
entry_points={
    'console_scripts': [
        'move_1m = tuto_move.move_1m:main'
    ]
}
```

Finally, you can test your node:










## 

class MoveNode(Node):

    def __init__(self):
        super().__init__('move')
        self.velocity_publisher = self.create_publisher(Twist, '/multi/cmd_nav', 10)
        self.timer = self.create_timer(0.1, self.activate) # 0.1 seconds to target a frequency of 10 hertz

    def activate(self):
        velo = Twist()
        velo.linear.x = 0.2 # target a 0.2 meter per second velocity
        self.velocity_publisher.publish(velo)

def main(args=None):
    rclpy.init(args=args)
    move = MoveNode()

    # Start the ros infinit loop with the move node.
    rclpy.spin(move)

    # At the end, destroy the node explicitly.
    move.destroy_node()

    # and shut the light down.
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

If you have troubles in understanding this python code: [classes](https://www.w3schools.com/python/python_classes.asp).

Then as we said earlier, expected message is a geometry_msgs twist, so composed by two attributes $(\mathit{linear},\ \mathit{angular})$ themselves composed by tree attributes $(x,\ y,\ z)$. But only `linear.x` and `angular.z` would have an effect.

Finally, you also have to fill some information into your package configuration.
Inform ROS that the package depends on `rclpy` (the **R**os **Cl**ient in **Py**thon) and `geometry_msgs`,
by adding in the `package.xml` file (inside the `<package>` markup):

```xml
    <depend>rclpy</depend>
    <depend>geometry_msgs</depend>
```

Ok, you just have to build and test your node (the tbot and the__ROS1__dynamic bridge activated).
To notice that you can also test your code on `turtlesim` by changing the targeted topic name.







```console
cd ..
colcon build
source ./install/setup.sh
ros2 run tuto_move move_1m
```

## The code

Next The goal is to create a process connecting a topic and publishing velocity as a twist message:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveNode(Node):

    def __init__(self):
        super().__init__('move')
        self.velocity_publisher = self.create_publisher(Twist, '/multi/cmd_nav', 10)
        self.timer = self.create_timer(0.1, self.activate) # 0.1 seconds to target a frequency of 10 hertz

    def activate(self):
        velo = Twist()
        velo.linear.x = 0.2 # target a 0.2 meter per second velocity
        self.velocity_publisher.publish(velo)

def main(args=None):
    rclpy.init(args=args)
    move = MoveNode()

    # Start the ros infinit loop with the move node.
    rclpy.spin(move)

    # At the end, destroy the node explicitly.
    move.destroy_node()

    # and shut the light down.
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

<!-- If you have troubles in understanding this python code: [classes](https://www.w3schools.com/python/python_classes.asp). -->

Then as we said earlier, expected message is a geometry_msgs twist, so composed by two attributes $(\mathit{linear},\ \mathit{angular})$ themselves composed by tree attributes $(x,\ y,\ z)$. But only `linear.x` and `angular.z` would have an effect.

Finally, you also have to fill some information into your package configuration.
Inform ROS that the package depends on `rclpy` (the **R**os **Cl**ient in **Py**thon) and `geometry_msgs`,
by adding in the `package.xml` file (inside the `<package>` markup):

```xml
    <depend>rclpy</depend>
    <depend>geometry_msgs</depend>
```

Ok, you just have to build and test your node (the tbot and the__ROS1__dynamic bridge activated).
To notice that you can also test your code on `turtlesim` by changing the targeted topic name.


## Terminate the exercise

We want the `move_1m` to move the robot for one meter then stop automatically.
To do that your node requires a new timer at the approximate time required to perform the movement with a new callback function to stop the robot.

To notice that the robot will stop but not necessarily the node.
To stop the control program, one of the solutions is to take control on the infinite loop.
# This would be performed by replacing the call to `spin` by a call to `spin_until_future_complete` (cf. [init/shutdown doc](https://docs.ros2.org/latest/api/rclpy/api/init_shutdown.html)).
A simple way to take control is to use a sequential call to `spin_once()` rather than a blocking call to `spin()`, typically into a `while rclpy.ok() :` loop.
Then you can define your own stop condition, the fact that the node terminate its job or not.

```python
# rclpy.spin(move)
while rclpy.ok() and move.isMoving() :
    rclpy.spin_once(move)
```

Do not forget to add the `isMoving` method to `MoveNode`.

## Going further

The _node/topic_ pattern is dedicated to a continuous process. 
ROS propose an action tool to define one shot behavior.
Use this tool to define the `move_1m` [action](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html).
Use the action parameters to set the distance (1 meter per default).

We want 3 new actions: `turn_left`, `turn_right` and `rear`.