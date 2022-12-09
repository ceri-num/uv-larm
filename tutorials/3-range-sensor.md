
# Range sensor

*Range sensors* are robot sensor permiting to detect obstacles and determin a distance to it.
Basic range sensor (infrared, utrasonic, laser) produce a unique mesure considering a given direction at a time.
By making the sensor rotating, it is possible to get messurements on a plan arraound the sensor. 

Hokuhyo, equiping the **Tbot**, is typically a kind of rotating lidar sensor (**l**aser **i**maging or **li**ght **d**etection **a**nd **r**anging).
The goal here is to integrate an almost 360 obstacle detection to generate safe robot movement.

More complexe lidar permits 3D messurements (i.e. in several plans at a time).


## Get Scan Data

## From Scan to Point-Cloud

## Avoid obstacles







## OLD VERSION:

## Avoid obstacles :

We get the basics.

The next move is to detect obstacles with a laser-range and to avoid it.

Hokuyo laser range is a sensor compliant with ROS by using the *urg_node* package. Connect the sensor, run *urg_node* node in *urg_node* ros package. 

A new topic appears `/scan` streaming the LaserRange data. It is possible to visualize it with *rviz* (fixed frame initially on `map` has to be set on `laser` frame, i.e. at this point it is impossible for the system to know where is the laser in the map).

1. Launch `rviz`
2. Add topic `laserScan`
3. Change reference frame to `laser`

The game is to integrate incoming information from `/scan` to `move.py` script to avoid obstacles in front of the robot.

1. Add dependence to scan messages. 
2. Subscribe to `/scan` topics and associate a call back function.
3. Transform the `/scan` distances into obstacle positions
4. Make the robot turning right or left to avoid close obstable on the left  or on the right (positive or negative value on the `Twist angular z` attribut).

Without a turtlebot, you can play with reccorded sensor stream (a bagfile).
cf. tutorials: [record bag files](http://wiki.ros.org/ROS/Tutorials/Recording%20and%20playing%20back%20data), [play bag files](http://wiki.ros.org/ROS/Tutorials/reading%20msgs%20from%20a%20bag%20file).

You can get a bag file in `mb6-data` repository:

```bash
wget https://bitbucket.org/imt-mobisyst/mb6-data/raw/master/tbot_bags/bags/tbot_bag_first_loop.bag -O move.bag
```

then run it: 

```bash
rosbag play move.bag
```

### From laser scan to point cloud

[laser scan](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html) in the sensor messages package, provide both the recorded bean distances and the meta information permiting to convert the distances on points in a regular carthesian frame (i.e. the angle between beams).

In a python script, the conversion to put in a call-back function attached to `/scan` topic, would look-like this:

```python
obstacles= []
angle= data.angle_min
for aDistance in data.ranges :
    if 0.1 < aDistance and aDistance < 5.0 :
        aPoint= [ 
            math.cos(angle) * aDistance, 
            math.sin( angle ) * aDistance
        ]
        obstacles.append( aPoint )
    angle+= data.angle_increment
rospy.loginfo( str(
    [ [ round(p[0], 2), round(p[1], 2) ] for p in  obstacles[0:10] ] 
) + " ..." )
```

Ideally, it is possible to publish this result in a pointCloud mesage and to visualize it on rviz...


### Avoid obstacle

If the laser position is coherent on the robot, it is possible to consider that the obstacle point list is provided in the same frame than the robot movement.

To avoid obstacle:

1. Determine if obstacle points are in front of th robot and in a given raduis.
2. Apply a rotation to the robot until the obstacle leave the danger zone.

Astuce: to share variable from a callback funtion to another in python, it is possible to use global variable (i.e. the obstacle list). [Exemple on w3schools](https://www.w3schools.com/python/gloss_python_global_variables.asp).
