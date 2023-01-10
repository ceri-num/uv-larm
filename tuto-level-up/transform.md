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

Classically, we use the map frame for global refering system, but without map it is possible to use the `odom` (from robot odometer).
The robot is defined with different frame: `base_link` at the gravity center of the robot. `base_footprint` as a projection of `base_link` on the floor.

### Understand frame and transformations

By starting any sensor as the laser publishing data it its own frame, it would be impossible for _rviz2_ to display the information into `map` frame.
The `map` and `laser` frames are independent.

Start the laser and rviz2.
Then the package `tf2_tools` provides with a process that generates a graph of the connection between the frames.

```console
ros2 run tf2_tools view_frame
evince frame.pdf
```

In _ROS_ `tf` stand for transformation.
It is a central tool permitting to get space-temporal information from a frame to another.
It supposes that specific messages are published into dedicated topic `tf`.

For instance, it is possible to generate a static transformation (it supposes that the laser is fixed in the environment at a specific position)

```console
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 "map" "laser"
```

You can validate with `view_frame` that the 2 frames ares connected.

The first 3 numbers fix the translation. It is the potion of the `laser` center into `map`. The next 3 numbers give the rotation.
Display the frames in _rviz2_ and play with different configurations.

For a simple robots it can be dossen of frames and it grows with robot parts (legs, arms).
_ROS_ provide a tool (state publisher) to plublish transform regarding how the frames are interconnected.
The tbot launch file of the `tbot_start` package already start state publisher based on a description of the tbot (kobuki robot in IMT Nord Europe configuration).

Start a launch file (`bringup` for instance) and generate the frame graph (`view_frame`).


### Record a pose in a specific frame.



## Permit autonomous navigation 

http://wiki.ros.org/tf2_ros


## Clean node

Record goal in global frame (`odom`), whatever the initial reference frame.

Make global frame parametrable (default `map`)




## Going futher:


- Manage a list of goal positions