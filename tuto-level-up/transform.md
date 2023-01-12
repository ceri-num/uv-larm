# Move Forward

The idea here is to develop a move strategy to permits a robot to reach positions successively, in a cluttered environment.
To do that, the node subscribes to a topic `goals` to get a position to reach.

This tutorial supposes that you perform the tutorial "[Move the Robot](1.2-move.md)".
A correction is proposed on [tbot package](https://bitbucket.org/imt-mobisyst/mb6-tbot/src/master/tbot_pytools/tbot_pytools/reactive_move.py).

The main difficulty here consists in following positioning kwoledge of the goals while the robot is moving.

## Record a goal position

It supposes that you play with at least 2 frames.
A local frame is attached to the robot and moving with it.
A global frame permits to localize the local frame (and so the robot) in the environment.
It supposes that your global frame is fixed in the environment.

Classically, we use the map frame for global referring system, but without map it is possible to use the `odom` (from robot odometer).
The robot is defined with different frame: `base_link` at the gravity center of the robot. `base_footprint` as a projection of `base_link` on the floor.

### Understand frame and transformations

By starting any sensor as the laser publishing data it its own frame, it would be impossible for _rviz2_ to display the information into `map` frame.
The `map` and `laser` frames are independent.

Start the laser and rviz2:

```console
# First console
ros2 run urg_node urg_node_driver --ros-args -p serial_port:=/dev/ttyACM0
# Second console
rviz2
```

Then the package `tf2_tools` provides with a process that generates a graph of the connection between the frames.

```console
#third console
ros2 run tf2_tools view_frames.py
evince frames.pdf
```

In _ROS_ `tf` stand for transformation.
It is a central tool permitting getting space-temporal information from a frame to another.
It supposes that specific messages are published into dedicated topic `tf`.
At this step, no transform are published...

It is possible to generate a static transformation (it supposes that the laser is fixed in the environment at a specific position)

```console
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 "map" "laser"
```

You can validate with `view_frame` that the 2 frames ares connected.

The first 3 numbers fix the translation. It is the potion of the `laser` center into `map`. The next 4 numbers give the rotation.
In fact, the publisher generates a [TransfromStamped mesage](https://docs.ros.org/en/jade/api/geometry_msgs/html/msg/TransformStamped.html) and the rotation is based on quaternion definition (cf. [wikipedia](https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation) for details...)
Display the frames in _rviz2_ (add > axes > set reference frame) and play with different configurations (kill and restart the `static_transform_publisher`). 

For a simple robot, it can be dozens of frames and it grows with robot parts (legs, arms).
_ROS_ provide a tool (state publisher) to publish transform regarding how the frames are interconnected.
The tbot launch file of the `tbot_start` package already starts state publisher based on a description of the tbot (kobuki robot in IMT Nord Europe configuration).

Start a launch file (`bringup` for instance) and generate the frame graph (`view_frame`).
In basic configuration, the robot provides a first pose estimation in global `odom` frame (ie. transformation between `odom` and `base_link`).

- Oficial documentation about tf2: [docs.ros.org](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Tf2-Main.html).


### Publish a pose in a specific frame.

Naturally, _ROS_ also provide C++ and Python library to manipulate transformation and permits developers to get pose coordinate from a frame to another.

The idea is to a declare a tf2 listener, an object that will subscribe to transform topics and maintain transformation data.
Then it is possible to recompute pose coordinates in any connected frames.  

More on : [wiki.ros.org](http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29).

The idea in our context is to develop a node `localGoal` that will remember a pose in a global frame and publish at a given frequence the pose in another local frame.

In our new node class, first in the constructor we have to declare the elements permitting the node to listen and keep the transformation available, a `listerner` and a `buffer`.

```python
class LocalGoal(Node):
    def __init__(self):
        super().__init__('goal_keeper')
        # Transform tool:
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        ...
```

The node is also defined with it onw attrituts: for instance a target local frame, a goal pose (position orientation).

```python
    def __init__(self):
        ...
        # Node Attribute:
        self.local_frame= 'base_link'
        self.global_goal= Pose()
        self.global_goal.position.x= (float)(1)
        self.global_goal.position.y= (float)(2)
        ...
```

And finally we require a timer with a callback function to publish continuously the goal pose.

```python
    def __init__(self):
        ...
        # Local Goal Publisher:
        self.create_timer(0.1, self.publish_goal)
```

We can now address the interesting question: _How to transform a position defined into a frame in another frame ?_
It consists in building a Transform object from the reference and target frames.
While a listener was declared on a transform buffer, it is possible to create this object from that tool (if exist).

The Transform object is generated with the method `lookup_transform(target_frame, reference_frame, time)`.
This method gets a `target_frame` (the frame id in which we want the pose) a `reference_frame` (the frame id in which the pose is currently defined) and a time.
In fact, the transformations are dynamically.
The position and orientation of elements (the robot(s), robot parts, ...)  change continuously and it is possible to get transformation in the present or in the past.
To get the current transformation a `node.time.Time()` permits to get the current time.

The lookup is not guaranteed to achieve.
It can fail in case of a gap in the transforms or obsolete transforms.
In case of fail, an exception is thrown accordingly the python exception manager (more on [w3schools](https://www.w3schools.com/python/python_try_except.asp)).

Finally, inside our `publish_goal` call back, getting a transform will look like: 

```python
    def publish_goal(self):
        currentTime= rclpy.time.Time()
        # Get Transformation
        try:
            stampedTransform= self.tf_buffer.lookup_transform(
                        self.local_frame,
                        'odom',
                        currentTime)
        except TransformException as tex:
            self.get_logger().info( f'Could not transform the goal into {self.local_frame}: {tex}')
            return
        ...
```

The transform is a stamped transform (ie. defined in a given time) defined by `geometry_msgs` package.
The pose transformation is already defined in a ros method of `tf2_geometry_msgs` package (things are never simple in ros...):

```python
    def publish_goal(self):
        ...
        # Compute goal in local coordinates
        stampedGoal= StampedPose()
        stampedGoal.pose= self.globalGoal
        stampedGoal.header.frame_id= 'odom'
        stampedGoal.header.stamp= currentTime
        localGoal = tf2_geometry_msgs.do_transform_pose( stampedGoal, stampedTransform )
        ...
```

You have just to only publish the local goal and integrate it in our ros package.


## Permit autonomous navigation 

The goal poses itself is not interesting.
The objective now is to include this code into the reactive move node in order to permits the robot to reach a decided destination, by avoiding obstacles.

1. Get the goal pose from a topic pose (a pose can be published with rviz).
2. Convert the received goal pose into a global frame (`odom`).
3. Control the robot to reach the goal (the control is in the local frame)
4. Stop the robot if it is close enough to the position.


## Going further - Path following

Rather than a unique pose, it could be interesting to define a succession of pose to follow (a path).
That for the reactive move has to manage a list of pose and switch from a pose to the next one each time it is expected.
