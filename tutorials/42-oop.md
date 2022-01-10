# RosPY and OOP

`rospy` librairie support developement with class (Object-oriented programming).
The ideas is that, rather than declare global variables, the developpers will use class attributs.

- [Python class on w3schools](https://www.w3schools.com/python/python_classes.asp)

## Use instances with rospy

ROS libraries (rospy for python) is a event engine, working with callback function.
However, callback function could be replaced by callback method attached to an instance of a class.

For instance, with Timer callbacks, a classical function: 

```python
# call the move_command at a regular frequency:
rospy.Timer( rospy.Duration(0.1), move_command, oneshot=False )
```

to a class method callback: 

```python
# call the move_command at a regular frequency:
rospy.Timer( rospy.Duration(0.1), myClassInstance.move_command, oneshot=False )
```

This way, `myClassInstance` will carry all the attributs required to compute and publish the command (robot state, gloal, obstacles, ...).


## Example of Python class:

With ROS one of the simplest way is to consider a class for each ros node you create.
The node initialization (callback connections) would be performed at instance construction, by considering that ros is already initialized.

```python
class MyNode :
    # Constructor:
    def __init__(self):
        self.goal= PoseStamped
        self.obstacles= []
        # Publisher:
        commandPublisher = rospy.Publisher(
            '/cmd_vel_mux/input/navi',
            Twist, queue_size=10 )
        # Subscriber:
        rospy.Subscriber( 'scan', LaserScan, self.interpret_scan)
        rospy.Subscriber( 'goal', PoseStamped, self.update_goal)
        # Timer:
        rospy.Timer( rospy.Duration(0.1), self.move_command, oneshot=False )
```

Then and still in `MyNode` class, it is possible to define the different callbacks...

```python
    def interpret_scan(self, scanData):
        # Update self.obstacles list form scanData
        ...

    def update_goal(self, goalData):
        # Record goal pose given in goalData in a fixed frame from 
        ...

    def move_command(self, timerData):
        # Compute command toward self.goal by avoiding self.obstacles
        ...
```

Finally, the ros node script would look like :

```python
rospy.init_node('move-to-and-safelly', anonymous=True)
myNode= MyNode()
rospy.spin()
```

