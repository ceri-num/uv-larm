# 04 - Handle transformation in node (and parameters)

<!-- passer à tf2 http://wiki.ros.org/tf2 -->

In the last **move** tutorial, we focused on controlling the robot to perform a movement by avoiding the obstacles.
The idea of this tutorial is to complete the *reactive move* node in order to permits a robot to reach a goal position.

To be efficient, you have to record the goal position in a frame, static in the environment while the command is computed accordingly to the frame attached to the mobile base.

## First estimation of robot position

*Open Loop Control* consider that there is a gap between the applied commands and the effective movement.
Control process continuously observe the pose estimation correct commands.
We aim to move the robot until it reaches a target position.

The topic *odom* provide a first estimation of the robot movements by observing wheels' movements.
By cumulating *odom* information, it is possible to maintain an estimation of the pose (position, orientation) of the robot on its environment.

Lucky for us, turtlebot already provides such pose estimation by maintaining transformation data that permit connecting */odom* frame (matching the start pose of the robot in the environment) with */base_footprint* frame (matching the actual pose of the robot).
A specific topic in ROS (**tf**) concentrates the transformation information linking the frames together. 

* [wiki.ros.org/tf](http://wiki.ros.org/tf)
<!-- * [Tutorials](http://wiki.ros.org/tf/Tutorials) -->

Avec tf2: [tf2 listener](http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29)

### To do:

- Echo *odom* topic and manipulate turtlebot
- Echo *tf* topic and manipulate turtlebot


## Test Driven Development

First you have to set up a minimal setup permitting testing that you have a node dealing with different frames to compute the appropriate velocity commands.

### To do:

- Generate a launch file **test_move_to** to set the position of a frame accordingly to another.

```xml
<node pkg="tf" type="static_transform_publisher" name="base_footprint_in_odom"
  args="2.3 4.0 0.01 0.8 0.0 0.0 /odom /base_footprint 80" />
```
- Open *rviz*, get a visualization of the two frames, in `/odom` main frame and save the configuration in your *ROS* package.

- Update the launch file to automatically launch *rviz* with the appropriate configuration.

```xml
<node pkg="rviz" type="rviz" name="rviz" args="-d $(find your_pkg)/rviz/test_move.rviz" />
```

- Create a new *Python* script *move-to.py* and add it to your launch file in a way you can debug it (with output activated).

```xml
<node pkg="your_pkg" type="move-to.py" name="gogogo" output="screen" />
```

- By executing your launch file you want to observe information in the terminal and in *rviz* when you publish a goal position (*2D Nav Goal* button in *rviz* window).
Nothing appends but your test is ready.
You can configurate the *2D Nav Goal* in `rviz/panel/tool properties` to publish in `/goal` topic.


## Manipulate the goal position from a frame to another

The idea is to record a received goal in `/odom` frame (a static frame in the environment) and manipulate it in `base_footprint` frame when computing the robot velocity toward the goal.
In python code, you first have to initialize a *tf.TransformListener*, then, you would be capable of transforming any pose in any frame toward any other frames which are connected through transformations.

### Setup:

- In your *move_to.py* script, create a subscriber callback to a topic *goal*, and print it each time you receive one. Test with rviz that *move-to.py* receive the goal correctly, and in the appropriate frame.

- Check for frames' connectivity with **view_frame** from **tf** package. `/odom` and `/base_footprint` should be connected.

### Transform:

Create a publisher and a second callback function, activated frequently to compute and send the velocity command to the robot. In this callback we are interested in the first steps that would transform the global *goal* position from `/odom` to  `/base_footprint`.

To do that, first we require to subscribe to `tf` topics, to listen for all the transformations messages and to maintain the computation of the positions and the orientations over all the frames.
In fact, a ros library `tf` permits to do that for us, while a **TransformListener** is initialized.

```python
# Initialize a global variable
tfListener = tf.TransformListener()
```

- [tf API](https://docs.ros.org/en/jade/api/tf/html/python/)

Then considering that `goal` is a stamped pose (i.e. a pose: position and orientation with a ROS header, including the `frame_id` and the `time_stamp` of the goal), tf will provide 

In the subscriber *init_node*:

```python
local_goal= tfListener.transformPose("/base_footprint", goal)
```

- Print the `local_goal` and play with *rviz* to validate that the transformation is correct.

- [transformPose API](https://docs.ros.org/en/jade/api/tf/html/python/tf_python.html#tf.Transformer.transformPose)


### Going further

Potentionnally it is possible to get a transformation toward a target frame from a reference frame with the function [lookupTransform](https://docs.ros.org/en/jade/api/tf/html/python/tf_python.html#tf.Transformer.lookupTransform).
It is useful (i.e. save some computations) if you plan to operate several transformations. A transformation at a given time is defined by a translation and a rotation. The rotation is stored in a compact and efficient structure for transformation computation: [Quaternion](https://en.wikipedia.org/wiki/Quaternion) (very famous in 3D engines...).


## Manage parameters with ROS parameters

Finally, to develop a beautiful **ROS** node, you can use [ROS parameters](http://wiki.ros.org/Parameter%20Server)
Depending on the node architecture, the reference frame or the goal_topic would be different.
In python, *rospy* provide tools to get parameters from *ROS*-configuration (launch file for instance) if exist:

```python
try:
    variable= rospy.get_param('~' + "parameter_name")
except KeyError:
    variable= default_value
```

the `try`-`except` python syntax permits developers to handle exceptions, throughout if a piece of code did not terminate normally.

### To do:

- Configure the *frame_id* and *goal* topic as *ROS* parameter.

- Adapt *rviz* configuration and the launch file.


## Move Forward:

Now you have the element to develop a complete **move-to** node...

- Compute and publish the appropriate velocity toward a goal position
- Stop the robot if it is close enough to the position.
- Set variables as *ROS* parameters (maximal speeds, command topic, ...)
- Handle the fact that the `goal` is received in different frames.
- Avoid the obstacles...

<!-- Do it in ROS2: https://robohub.org/exploring-ros2-using-wheeled-robot-3-moving-the-robot/ -->