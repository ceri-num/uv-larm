# ROS Services

ROS philosophy aims to connect piece of programs. 
Each piece of programs (ROS node) communicates through topics in order to define the robot behavior.

The interacting nodes form together a software architecture to control a robot.
Services have been developed to allow punctual requests on the control of the robot.
It’s mainly useful to supervise a running software architecture.

You can find a complete tutorial on [wiki.ros.org](https://wiki.ros.org/rospy/Tutorials).

--- 

## TicTac architecture:

Let’s start with a simple ROS architecture based on **2** nodes:

- MyTicTac: sending a succession of messages at a fixed frequency.
- MyEcho: printing the messages.

![TicTac architecture](resources/tictac.svg)

--- 

### TicTac Publisher

TicTac is a pure ROS Publisher node. It means that the node exists only to publish data over a ROS topic.

**tictac.py**: 

```python
#!/bin/python3
import rospy
from std_msgs.msg import String

# Program parameter:
rospy.init_node('tictac', anonymous=True)
pub = rospy.Publisher('tic', String, queue_size=10)
rate = rospy.Rate(1) # 10hz
msgs= ['tic', 'tac']

# Program loop:
i= 0
while not rospy.is_shutdown():
    pub.publish(msgs[i])
    i= (i+1)%len(msgs)
    rate.sleep()
```

--- 

### Echo Subscriber

This node simply print in information ROS output the content of messages in the ROS topic *'tic'* - **echo.py**: 

```python
#!/bin/python3
import rospy
from std_msgs.msg import String

# Program parameter:
rospy.init_node('echo', anonymous=True)
topic= 'tic'

# ROS callback:
def echo(mesage):
    rospy.loginfo( mesage.data )

# ROS configuration:
rospy.Subscriber(topic, String, echo)

# run ROS node:
rospy.spin()
```

--- 

### Launch file

the Tictac architecture is materialized with a specific launch file:

```xml
<launch>
<node name="MyTicTac" pkg="prof" type="tictac.py" />
<node name="MyEcho" pkg="prof" type="echo.py" output="screen" />
</launch>
```

Try a RQT graph to valid it (`rosrun rqt_graph rqt_graph`).


--- 

## Add Service functionality

As we say, services have been developed to allow one shoot actions on the control of the robot.
In the philosophies topics represent open and persistent broadcast communication channels while services connect nodes for a punctual messages interaction.
It’s mainly useful to supervise a running software architecture without to stop and relaunch nodes with new configuration.

--- 

For instance, we want a service that changes the TicTac rate frequency.

We can see that this service didn't exist yet. `rosservice list` will list only default ros node service (to control logs).

- First, ROS request to create the new service.
- Second, we have to modify our `tictac.py` node to allow this service on it. MyTicTac will become a publisher and a service node.
- Then we will have to call this service.

--- 

### Define the new service messages

cf: [ROS Tutorial: msg and srv creation](https://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)

On a new `srv/Rate.srv` file :

```
Float64 Rate
---
Int64 Ok
```

--- 

On the package configuration `package.xml` file, uncomment:

```
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

--- 

On the package configuration `CMakeList.txt` file,
add: 

```
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)
```

and:

```
add_service_files(
  FILES
  Rate.srv
)
```

--- 

and:

```
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)
```

and: `CATKIN_DEPENDS message_runtime` in `catkin_package(... ...)`

--- 

Then, don't ask why and rebuild your package (`catkin_make clean && catkin_make`).

--- 

### Grand a node with a new service

On `tictac.py` we add service definition.

First you need to import services messages definition.

```python 
from your_package import srv
```

--- 

Second to define a new callback function for your service.

```python 
# ROS callback
def rate_service( data ):
    global rate
    rate= rospy.Rate( data.Value )
    return 1
```

--- 

And finally to connect this callback to a new node service:

```python
# ROS node configuration
rospy.Service('rate', srv.Rate, rate_service)
```

--- 

Complete `TicTac.py` file: 

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from prof import srv

# Program parameter:
rospy.init_node('tictac', anonymous=True)
pub = rospy.Publisher('tic', String, queue_size=10)
rate = rospy.Rate(1) # 1hz
msgs= ['tic', 'tac']

# ROS callback
def rate_service( data ):
    global rate
    rate= rospy.Rate( data.Value )
    return 1

# ROS node configuration
rospy.Service('rate', srv.Rate, rate_service)

# Program loop:
i= 0
while not rospy.is_shutdown():
    # publishing
    pub.publish(msgs[i])
    # increment
    i= (i+1)%len(msgs)
    # Wait
    rate.sleep()
```

--- 

### Call your new service

In your first terminal relaunch your program `roslaunch prof tictac.launch`.

Then in your second terminal you can view the new service, and modify the TicTac rate:

```bash
rosservice list
rosservice info /rate
rosservice call /rate 2.0
```

---

## Going further

More documentation on the [ROS tutorial](https://wiki.ros.org/ROS/Installation):

- You can for instance call a service automatically from another node [see Client Node](https://wiki.ros.org/rospy_tutorials/Tutorials/WritingServiceClient)
- Move from Services API to Action API if it is required that your robot adapts it behavior to fullfy your request (i.e. the Services responces is met to be instantaneous).

### Exercices

- I want to add or remove messages on the 'TicTac' list of messages ('tic, 'tac', ...).
- I want to name my services automatically with the node name (`/MyTicTac_rate` for instance if the node is named `MyTicTac`)


