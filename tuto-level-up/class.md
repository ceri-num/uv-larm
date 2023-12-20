# Create Node with Class

The official ROS tutorial invites to use inheritence of ROS node.
We propose here, to explore another way.

## Motivation

ROS-node philosiphy relies on events that triger call back function.
Typically, the subscrition to a topic trigger a function to process (a call back) each time a new mesages is available.
It is coded as :

```python
# define a callback function
def aCallback( aMsg ):
    ...

# Connect a callback to events.
aRosNode= Node('node_name')
aRosNode.create_subscription( MsgType, 'scan', aCallback, 10)
```

However, we generally need a context of execution for a given callback.
We wan to access certain information in the callback definition (parameters, recorded previous data, et.).
Or, in a more technical way, we need something like : 

```python
# define a callback function
def aCallback( aContext, aMsg ):
    ...
```

And [Object Oriented Programming](https://en.wikipedia.org/wiki/Object-oriented_programming) allows such a thing.
The astuce consists in defining a class to model the context, and use a method of the class as call-back function.

In python: 

```python
# define a callback function
class Context :
    def aCallback( self, aMsg ):
        ...

# Connect a callback to events.
aRosNode= Node('node_name')
myContext= Context()
aRosNode.create_subscription( MsgType, 'scan', myContext.aCallback, 10)
```

# Connect a callback to events.
aRosNode= Node('node_name')
aRosNode.create_subscription( MsgType, 'scan', aCallback, 10)
```

## ROS solution


## Alternative Solution



## Event-driven programming

To notice that [Event-Driven Programming](https://en.wikipedia.org/wiki/Event-driven_programming) is not something unique for ROS.

IMPORTANT - For a better security, implement the control function independently from the scan callback and activate your control at a desired frequency by using a timer.
Stop the robot if no scan was arrived the last second and test it by disconnecting the laser.


<!--
Like this: 

```python 

```
-->





## Infinit Safe Move

Create a new node `reactive_move` that will command the robot velocities in a way that the robot will avoid the obstacles.
The node subscribe to scan data and publish velocities.

1. Determine a rectangle in front of the robot and get the point cloup obstacles in this rectangle.
2. If an obstacle is present in the right part of the rectangle, turn left.
3. If an obstacle in present in the left part of the rectangle, turn right.
4. Otherwise move in straight line.
5. Calibrate the rectangle configuration and the speeds to get a coherent and safe control.
6. Add rules to better control the robot in a dead end scenario

When it is working in simulation, test your solution on a real robot.
