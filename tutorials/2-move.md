# 02 - Move a robot


This tutorial aims to take control of a __tbot__  robot.
A tbot is a turtlebot2 (itself based on a kobuki platform) equipped with a laser range to navigate in a cluttered environment and a camera to recognize objects.

<!-- This tutorial supose that you already processed the [Challenge - Kick-Off](../challenge/intro.md). OU PAS -->

## Connect the tbot:

If it is not yet the case,
the machine connecting the robot and its sensors have to be conferred accordingly to the [IMT MobiSyst tbot](https://bitbucket.org/imt-mobisyst/mb6-tbot) 

You will have then a ROS WorkSpace including tbot meta-package itself including several ROS packages.

- Verrify it: 

```sh
cd rosworkspace
ls src
ls src/tbot
```

- Build the packages: 

```
colcon build
```

- Update your shell environment variables: 

```sh
source install/setup.sh
```

- Connect the tbot base and start the control nodes : 

```sh
ros2 run tbot_start start # feed the bot password is asked
```

Into another terminal start a bridge between ros1 and ros2: 

```sh
source /opt/ros/noetic/setup.sh
ros2 run ros1_bridge dynamic_bridge
```

Finally, try to take control in a third terminal:

``sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/mobile_base/commands/velocity
``

Close everything with `ctrl-c`.


## Somme explanations:

The control of tbot (turtlebot3 kobuki base) is still dependent on old version of ros1 so it is packaged into a docker environment.
This solution implies: 1) super user access to run docker (potentially a sudo password) and 2) to launch a bridge between ros1 and ros2.

The 'dynamic_bridge' needs some ROS1 variables that why the user has to source `ROS/noetic` in the terminal before to start the bridge.
At this point, the ROS1 topics are invisible in ROS2. It is a dynamic bridge, It connects things only on demand.
So by starting the teleop node, you can see the activation of the bridge in its terminal.

It works well if and only if you address the appropriate topic. Here it is exactly `/mobile_base/commands/velocity`.
If you would like to see all robot topics, you have to switch your terminal in ROS1 and then list the topics with ROS1 command:

```
source /opt/ros/noetic/setup.sh
rostopic list
```

The teleop publishes a [geometry_msgs](https://docs.ros2.org/foxy/api/geometry_msgs/index-msg.html) [twist](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html) mesage.
It is composed of two vectors $(x, y, z)$, one for linear speed $(m/s)$, and the second for angular speed $(rad/s)$. 
However a [nonholonomic](https://en.wikipedia.org/wiki/Nonholonomic_system) ground robot as the **tbot** would move only on `x` and turn only on `z`. 
It is not as free as a drone, you can echo the mesages into a 4th terminal.

- Try to control the robot with `ros2 topic pub` command instead of teleop.


## move node

The idea now is to create a node that will control the robot accordingly to our expectation.
For that we will create a python ros package and a new node in this packatge to sent velocity in the appropriate topic.

First create a new package `tuto_move` in your `src` directory of your workspace (ie. aside of `tbot`).

```
cd rosworkspace/src
ros2 pkg create --build-type ament_python tuto_move
```

Inside your new package create a node `move_1m` at the appropriate location that will integrate the code for moving the tbot 1 meter forward.

```
touch tuto_move/tuto_move/move_1m.py
```

Edit this new file with a verry simple code in order to test the packages: 

```python
def main():
    print('Move move move !')

if __name__ == '_main__' :
    main()
```

For more detail on those manipulation, you can return to the [ros tutorial](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html).

If you have troubles in understanding this python code: [functions](https://www.w3schools.com/python/python_functions.asp), [environnement d'exécution principal](https://docs.python.org/fr/3/library/__main__.html).

To test our node: 

```
cd ~/rosspace
colcon build
source ./install/setup.sh
ros2 run tuto_move move_1m
```

## The code

Next the goal is to create a process connecting a topic and publing velocity as a twist mesage: 

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveNode(Node):

    def __init__(self):
        super().__init__('move')
        self.velocity_publisher = self.create_publisher(String, '/mobile_base/commands/velocity', 10)
        self.timer = self.create_timer(0.1, self.activate) # 0.1 seconds to target a frequency of 10 hertz 

    def activate(self):
        velo = Twist()
        velo.linear.x = 0.5 # target a 0.5 meter per second velocity
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

Then as we said erlier, expected mesage is a geometry_msgs twist, so composed by two attributes $(\mathit{linear},\ \mathit{angular})$ thenself composed by tree attributs $(x,\ y,\ z)$.


Ok, you just has to test your node the tbot and the ros1 dynamic bridge activated. To notice that you can also test your code on turtlesim by changing the targeted topic name.


## Terminate the exercice

We want that the `move_1m` move the robot for one meter then stop automaticaly.
To do that your node require a new timer at the approximate time required to perform the movement with a new callback function to stop the robot.

To notice that the robot will stop but not necessarly the node.
To stop the control probram, one of the solution is to take control on the infinite loop.
This would be performed by replace the call to `spin` by a call to `spin_until_future_complete` (cf. [init/shutdown doc](https://docs.ros2.org/latest/api/rclpy/api/init_shutdown.html)).


## Going futher

We want 3 new nodes: `turn_left_45`, `turn_right_45`, `rear_0.5m`.
