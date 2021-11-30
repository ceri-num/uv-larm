# 04 - Handle transformation in node (and parameters)

In the last **move-to** tutorial, we focused on controlling the robot to perform a predetermined movement potentially by avoiding the obstacles.
The idea of this tutorial is to finalize a *reactive move* node that permits a robot to reach a goal position.

To be efficient, you have to record the goal position in a frame, static in the environment while the command is computed accordingly to the frame attached to the mobile base.

## First estimation of robot position

*Open Loop Control* consider that there is a gap between the applied commands and the effective movement.
Control process continuously observe the pose estimation correct commands.
We aim to move the robot until it reaches a target position.

The topic *odom* provide a first estimation of the robot movements by observing wheels' movements.
By cumulating *odom* information, it is possible to maintain an estimation of the pose (position, orientation) of the robot on its environment.

Lucky for us, turtlebot already provides such pose estimation by maintaining transformation data that permit connecting */odom* frame (matching the start pose of the robot in the environment) with */base_footprint* frame (matching the actual pose of the robot).

### To do:

- Echo *odom* topic and manipulate turtlebot
- Echo *tf* topic and manipulate turtlebot
- Understand how *transformation* works in ROS
  * [wiki.ros.org/tf](http://wiki.ros.org/tf)
  * [Tutorials](http://wiki.ros.org/tf/Tutorials)


## Test Driven Development

First you have to set up an environment permitting testing that you have a node dealing with different frames to compute the appropriate commands.

### To do:

- Generate a launch file to set the position of a frame accordingly to another.

```xml
<node pkg="tf" type="static_transform_publisher" name="base_link_in_odom"
  args="2.3 4.0 0.01 0.8 0.0 0.0 /odom /base_link 80" />
```
- Open *rviz*, get a visualization of the two frames and save the configuration in your *ROS* package.

- Update the launch file to automatically launch *rviz* with the appropriate configuration.

```xml
<node pkg="rviz" type="rviz" name="rviz" args="-d $(find your_pkg)/rviz/test_move.rviz" />
```

- Create a new *Python* script *move-to.py* and add it to your launch file in a way you can debug it.

```xml
<node pkg="your_pkg" type="move-to.py" name="gogogo" output="screen" />
```

- By executing your launch file you want to observe information in the terminal and in *rviz* when you publish a goal position (*2D Nav Goal* button in *rviz* window).
Nothing appends but your test is ready.

## Record a goal position

The idea is to record a received goal in */odom* frame which is static in the environment (ie. the goal will not move with the robot).
However, the goal could be communicated in a different frame (relatively to the robot for instance)

In python code, you first have to initialize a *tf.TransformListener*, then, you would be capable of transforming any pose in any frame toward any other frames which are connected through transformation.

### To do:

- In your *move-to.py* script, create a subscriber to a topic *goal*, and print it each time you receive one. Test with rviz that *move-to.py* receive the goal correctly.

- Check for frames' connectivity with [rqt_tf_tree](https://wiki.ros.org/rqt_tf_tree)

- After receiving the goal, transform it toward */odom* frame to record it with static coordinates, then print it.

After *init_node*:

```python
# Initialize global variable
_trans = tf.TransformListener()
```

In the subscriber *init_node*:

```python
recorded_goal= _trans.transformPose("/odom", goal)
```

- Play with *rviz* to validate the transformation is correct.


## Manage parameters with ROS parameters

Depending on the node architecture, the reference frame or the goal_topic would be different.
In python, *rospy* provide tools to get parameters from *ROS*-configuration (launch file for instance) if exist:

```python
try:
    variable= rospy.get_param('~' + "parameter_name")
except KeyError:
    variable= default_value
```

### To do:

- Configure the *frame_id* and *goal* topic as *ROS* parameter.

- Adapt *rviz* configuration and the launch file.

- To notice that a correction is available on the [github of the tutorial](https://github.com/ceri-num/uv-larm/tree/master/files/handle-tf-pkg).

## Move Forward:

You just have to transform again goal position in local frame and use it to compute appropriate commands.

Do not forget to set all global variables you use as *ROS* parameters (maximum speeds, command topic, ...)
