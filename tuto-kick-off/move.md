# 02 - Move a robot

This tutorial aims to take control of a __tbot__  robot.
A tbot is a turtlebot2 (itself based on a kobuki platform) equipped with a laser range to navigate in a cluttered environment and a camera to recognize objects.

<!-- This tutorial suppose that you already processed the [Challenge - Kick-Off](../challenge/intro.md). OU PAS -->

## Connect the tbot:

If it is not yet the case,
the machine connecting the robot and its sensors have to be configured accordingly to the [IMT MobiSyst tbot](https://bitbucket.org/imt-mobisyst/mb6-tbot)

You will have then a ROS-2 WorkSpace including __tbot__ meta-package (`pkg-tbot`) itself including several ROS packages.

- Verrify it:

```console
cd ~/ros2_ws
ls
```

- Build the packages:

```console
colcon build
```

- Update your shell environment variables:

```console
source install/setup.sh
```

- Connect the tbot base and start the control nodes :

```console
ros2 run tbot_start start_base # feed the bot password is asked
```

Into another terminal start a bridge between__ROS1__and ros2:

```console
ros2 run ros1_bridge dynamic_bridge
```

Finally, try to take control in a third terminal:

```console
ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/commands/velocity
```

Close everything with `ctrl-c`.


## Some explanations:

The control of __tbot__ (turtlebot3 kobuki base) is still dependent on old version of ROS (__ROS1__) so it is packaged into a docker environment.
This solution implies: 1) super user access to run docker (potentially a sudo password at `tbot_start start_base` ) and 2) to launch a bridge between__ROS1__and ros2.

<!--[RESOLVED] The 'dynamic_bridge' needs some__ROS1__variables that why the user has to source `ROS/noetic` in the terminal before to start the bridge. -->
At this point, the__ROS1__topics are invisible in ROS2.
It is a dynamic bridge, It connects things only on demand.
So by starting the teleop node, you can see the activation of the bridge in its terminal.

It works well if and only if you address the appropriate topic.
Here it is exactly `/commands/velocity`.

However, tbot integrate a multiplexer.
The node listens different topics with different priorities (by default: `cmd-nav` and `cmd-telop`) and filter the appropriate commands and send them into `/cmd_vel`.

```console
ros2 run tbot_pytools multiplexer
```

As a first result, the dynamic bridge is now activated and all the commands topic are visible from __ROS2__ (`ros2 topic list`).
As a second result, the teleop commands has to target the appropriate topics.
This way, if an operator teleop the robot, the multiplexer will shunt the autonomous navigation and will force the robot to stop if no commands are published from a certain time.

From now, you always operate on the robot with the multiplexer on.
However, you can use `ros2 launch tbot-start move_launch.py` to start the bridge and the multiplexer (this launch do not include `tbot_start start_base`).

If you would like to see all robot topics, you have to switch your terminal in __ROS1__ and then list the topics with __ROS1__ command:

```console
source /opt/ros/noetic/setup.sh
rostopic list
```

The teleop publishes a [geometry_msgs](https://docs.ros2.org/foxy/api/geometry_msgs/index-msg.html) [twist](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html) message.
It is composed of two vectors $(x, y, z)$, one for linear speed $(m/s)$, and the second for angular speed $(rad/s)$.
However a [nonholonomic](https://en.wikipedia.org/wiki/Nonholonomic_system) ground robot as the **tbot** would move only on `x` and turn only on `z`.
It is not as free as a drone, you can echo the messages into a 4th terminal.

- Try to control the robot with `ros2 topic pub` command publishing in the navigation topic (`/multi/cmd_nav`).


## move node

The idea now is to create a node that will control the robot accordingly to our expectation.
For that we will create a python ros package and a new node in this package to send velocity in the appropriate topic.

This tutorial is adapted from [official ros2 tutorial](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries.html).

First create a new package `tuto_move` in the workspace directory of your workspace (ie. aside of `pkg-tbot`).

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

```console
cd ..
colcon build
source ./install/setup.sh
ros2 run tuto_move move_1m
```

## The code

Next the goal is to create a process connecting a topic and publishing velocity as a twist message:

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