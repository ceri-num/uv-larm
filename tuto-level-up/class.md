# Create Node with Class

The official ROS tutorial invites to use inheritance of ROS node.
We propose here, to explore another way.

## Motivation

ROS-node philosophy relies on events that trigger call-back function.
Typically, the subscription to a topic trigger a function to process (a call-back) each time a new message is available.
It is coded as :

```python
# Define a callback function
def aCallback( aMsg ):
    ...

# Connect a callback to events.
aRosNode= Node('node_name')
aRosNode.create_subscription( MsgType, 'scan', aCallback, 10)
```

However, we generally need a context of execution for a given callback.
We wan to access certain information in the callback definition (parameters, recorded previous data, et.).
Or, in a more technical way, we need something like:

```python
# Define a callback function
def aCallback( aContext, aMsg ):
    ...
```

And [Object-Oriented Programming](https://en.wikipedia.org/wiki/Object-oriented_programming) allows such a thing.
The trick consists of defining a class to model the context, and use a method of the class as call-back function.

In python: 

```python
# Define a callback function
class Context :
    def aCallback( self, aMsg ):
        ...

# Connect a callback to events.
aRosNode= Node('node_name')
myContext= Context()
aRosNode.create_subscription( MsgType, 'scan', myContext.aCallback, 10)
```

## ROS solution

The [ROS tutorial](https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html) propose a solution based on [inheritence](https://en.wikipedia.org/wiki/Inheritance_(object-oriented_programming)).
The `Context` class inherits from ros `Node` class.

Example with the subscriber:

```python
#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription= self.create_subscription(
            String, 'topic',
            self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

As a result, Node method can be performed on `self` as for `create_subscription`,
but the inheritance needs to be set correctly (call to `super().__init__`)
and the node instance need to be created after the initialization to `rclpy`.


## Alternative Solution

An alternative solution aims to minimize the effort developers should do when they use our solution.

Typically, the number of lines in the main function,
with a main for instance as :

```python
def main():
    minimal_subscriber= MSContext()
    minimal_subscriber.process()
```

The management of ros client (`rclpy`) and ros node is hidden to the user.
For this purpose,
one of the solutions, is to define initialize ros client only in the process method and to use ROS `node` as an attribute of the class.

The solution could be, for instance:

```python
class MSContext():

    def listener_callback(self, msg):
        self._node.get_logger().info('I heard: "%s"' % msg.data)
    
    def process(self)
        rclpy.init(args=args)
        self._node = Node()
        self._node.create_subscription(
            String, 'topic',
            self.listener_callback, 10)
        # Infinite loop:
        rclpy.spin(minimal_subscriber)
        # Clean stop:
        minimal_subscriber.destroy_node()
        rclpy.shutdown()
```

## Event-driven programming

To notice that [Event-Driven Programming](https://en.wikipedia.org/wiki/Event-driven_programming) is not something unique for ROS.
Typically, most of the IHM API or web technology are based on EDP.

Technically in ROS, it is managed by the ros client, with the `spin_once` or `spin` functions.
These 2 functions connect events (the reception of a message for instance) with the appropriate callback.
It is also those functions that effectively push messages into the topics after a `publishe` action.

`spin_once` activates once the ros client and should be call in an infinite loop.
`spin` handle the infinite loop directly (and should be preferred).

To notice that it is possible to generate events with timers:

```python
aNode.create_timer( timer_period, timer_callback )
```

For instance, with a simple publisher: 

```python
#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MPContext:

    def __init__(self):
        self.i = 0
    
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self._node.publisher_.publish(msg)
        self._node.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def process(self):
        rclpy.init()
        self._node= Node()
        # Create a publisher
        self._publisher= self._node.create_publisher(String, 'topic', 10)
        # Create a timer at 0.5 hertz, with a callback
        self._timer = self._node.create_timer(0.5, self.timer_callback)
        # Go
        rclpy.spin(self._node)
        # Clean stop
        self._node.minimal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    rosContext= MPContext()
    rosContext.process()
```

<!-- ## Python package -->

## Infinit Safe Move

Create a new node `reactive_move` in your package that will command the robot velocities in a way that the robot will avoid the obstacles.
The node subscribes to scan data and publish velocities.

1. Determine a rectangle in front of the robot and get the point cloud obstacles in this rectangle.
2. If an obstacle is present in the right part of the rectangle, turn left.
3. If an obstacle in present in the left part of the rectangle, turn right.
4. Otherwise move in a straight line.
5. Calibrate the rectangle configuration and the speeds to get a coherent and safe control.
6. Add rules to better control the robot in a dead end scenario

When it is working in the simulation, test your solution on a real robot.

IMPORTANT - For a better security, implement the control function independently from the scan callback and activate your control at a desired frequency by using a timer.

OPTIONAL - Stop the robot if no scan was arrived the last second and test it by stopping the urg_node.
