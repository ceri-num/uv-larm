# Range sensor

*Range sensors* are robot sensor permitting to detect obstacles and determine a distance to it.
Basic range sensors (infrared, ultrasonic, laser) produce a unique measure considering a given direction at a time.
By making the sensor rotating, it is possible to get measurements on a plan around the sensor.

Hokuhyo, equipping the **Tbot**, is typically a kind of rotating lidar sensor (**l**aser **i**maging or **li**ght **d**etection **a**nd **r**anging).
The goal here is to integrate an almost 360 obstacle detection to generate safe robot movement.

More complex lidar permits 3D measurements (i.e. in several plans at a time).


## Get Scan Data

Well, let’s visualize the laser scan in rviz2.
For that, verify that the user has the right to read data on the device.
By connecting the laser sensor, a access file appears in Linux `/dev` directory named `ttyACM0`.
Verify the owner of the file:

```console
ls -la /dev
```

Normally `/dev/ttyACM0` is owned by user `root` and group `dialout` with `crw-rw----` right, mining that owner and all members of the group can read (`r`) and write (`w`) and all the other users have no access to the resource.
Verify that `bot` is a member of the group `dialout`

```console
cat /etc/group
```

Cool. let run a driver to convert device I/O to ros messages:

```console
ros2 run urg_node urg_node_driver --ros-args -p serial_port:=/dev/ttyACM0
```

From that point data are streamed in `\scan` topic.
It is possible to check it with `ros2 topic list` and `ros2 topic echo scan`.

Now you can visualize it on `rviz2` program.
Start `rviz2` in a terminal, _add_ a flux _laserScan_ and configure it in `/scan` topic.
Nothing appears and it is normal.
_Rviz_ global option is configured on _map_ frame, and nothing permits to set the position of the laser sensor in the map.
The laser-scan frame is named _laser_.
Change this information into global options and set the laser-scan size to `0,1` for a better display.

Stop everything.

Perform the same exercise to visualize simulated LaserScan from Gazebo simulator:

<!-- TODO: Provide console example -->

The simulator work on __ROS1__ and at this point the scan topic is not visible from `ROS2` command.
You have to start a dynamic bridge before to listen this topic in Rviz2.
Attention the laser-scan frame has changed.


## A first node logging the scan.

First we will initialize our node `scan_echo` in a python ROS2 packages (the `tuto_move` from the [Move the Robot](tutorials/2-move.md) tutorial for instance).

Edit a new file `tuto_move/tuto_move/scan_echo.py` with a very simple code :

```python
def main():
    print('Move move move !')

if __name__ == '_main__' :
    main()
```

Then, you have to inform ROS for the existence of your new node.
In your package `setup.py` file, add your node in the `console_scripts` [list](https://www.w3schools.com/python/python_lists.asp) of the `entry_points` [dictionary](https://www.w3schools.com/python/python_dictionaries.asp).
The new list item would look like `'scan_echo = tuto_move.scan_echo:main'`.

Test your `scan_echo` node.

The idea is to connect [sensor_msgs LaserScan](https://docs.ros2.org/foxy/api/sensor_msgs/msg/LaserScan.html).
That for, add dependency in the package configuration (`package.xml`)and import the msgs class in your python scrip.

Test your `scan_echo` node.

Now it is possible to create a class subscribing to the `scan` topic and to log the result:

```python
#!python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanInterpret(Node):

    def __init__(self):
        super().__init__('scan_interpreter')
        self.create_subscription( LaserScan, 'scan', self.scan_callback, 10)

    def scan_callback(self, scanMsg):
        self.get_logger().info( f"scan:\n{scanMsg}" )

def main(args=None):
    rclpy.init(args=args)
    scanInterpret = ScanInterpret()
    rclpy.spin(scanInterpret)
    scanInterpret.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()
```

Test your `scan_echo` node, with the hokuyo laser and on simulation.

Modify the logger to print only the information into the `header` and the number of ranges.


## From LaserScan to PointCloud

**LaserScan** provides both the recorded bean distances (ranges) and the meta information permitting converting the distances on points in a regular Cartesian frame (i.e. the angle between beans).

In a python, the conversion would look like this:

```python
obstacles= []
angle= scanMsg.angle_min
for aDistance in scanMsg.ranges :
    if 0.1 < aDistance and aDistance < 5.0 :
        aPoint= [
            math.cos(angle) * aDistance,
            math.sin( angle ) * aDistance
        ]
        obstacles.append( aPoint )
    angle+= scanMsg.angle_increment
```

The exercise consists in modifying the scan callback function to generate the point cloud list.
To log a sample of the point cloud:

```python
sample= [ [ round(p[0], 2), round(p[1], 2) ] for p in  obstacles[10:20] ]
self.get_logger().info( f" obs({len(obstacles)}) ...{sample}..." )
```

Finally, it is possible to publish this result in a [pointCloud](https://docs.ros2.org/foxy/api/sensor_msgs/msg/PointCloud.html) message and to visualize it on rviz2 in a superposition of the LaserScan.

_PointCloud_ is based on `geometry_msgs.Point32` with float coordinate.
The creation of _Point32_ will require explicite cast.

```python
aPoint= Point32()
aPoint.x= (float)(math.cos(angle) * aDistance)
aPoint.y= (float)(math.sin( angle ) * aDistance)
aPoint.z= (float)(0)
```

## Infinit Safe Move

Create a new node `reactive_move` that will command the robot velocities in a way that the robot will avoid the obstacles.
Develop your solution based on the simulation (the tbot multiplexer sends command both on robot and simulation topic. So use `tbot_pytool multiplexer` and use `multi/cmd_navi` to publish in).

1. Determine a rectangle in front of the robot and get the point cloup obstacles in this rectangle.
2. If an obstacle is present in the right part of the rectangle, turn left.
3. If an obstacle in present in the left part of the rectangle, turn right.
4. Otherwise move in straight line.
5. Calibrate the rectangle configuration and the speeds to get a coherent and safe control.
6. Add rules to better control the robot in a dead end scenario

Test your solution on a real robot.

IMPORTANT - For a better security, implement the control function independently from the scan callback and activate your control at a desired frequency by using a timer.
Stop the robot if no scan was arrived the last second and test it by disconnecting the laser.