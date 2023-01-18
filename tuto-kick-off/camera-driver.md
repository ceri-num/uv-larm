# Camera Driver

This tutorial cover the basis of the integration of a new sensor in _ROS 2_ environment.
In our case we will integrate [Realsense D400 RGBDi](https://www.intelrealsense.com/introducing-intel-realsense-d400-product-family/) camera.


## Drivers

First, the integration of a sensor require to identify the driver (the piece of code permiting to communicate with a devices - hardware level) and the API (Application Programming interface).

Concretly, we mostly seek for the appropriate [librairies](https://en.wikipedia.org/wiki/Library_(computing)) correctly integrated to our system.

Idéaly the community already support the desired librairies (like for [libsdl2](https://www.libsdl.org/) for instance, simple C lib "to provide low level access to audio, keyboard, mouse, joystick, and graphics hardware").
By searching for `libsdl2` with Ubuntu-Aptitude we will find several packages ready to be installed:

```console
apt search libsdl2
```

- `libsdl2-x.x` runing librairies (installed if programs use _SDL2_)
- `libsdl2-dev` development file (to install if you plan to develop a program based on _SDL2_)
- and some extra libs.

Other wise, we have to build/compile the driver from source code.
In case of realsense, [librealsense](https://github.com/IntelRealSense/librealsense) recommand to use `vcpkg` to build and install it.

Normally, after installation, you can run a small script to request the cam (more on [dev.intelrealsense.com](https://dev.intelrealsense.com/docs/python2)): 

```python
#!/usr/bin/env python3

###############################################
##              Simple Request               ##
###############################################

import pyrealsense2 as rs

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

print( f"Connect: {device_product_line}" )
for s in device.sensors:
    print( "Name:" + s.get_info(rs.camera_info.name) )
```

Copy the code on a `test-camera.py` file and process it (`python3 test-camera.py`).
Try this script with severals cameras, not all the Realsense provide for IMU (accelerations) information.

## OpenCV2 - the queen of the vision librairie.

Next we can try to visualise the image flux in a windows.
For that we will use [OpenCV2](https://opencv.org/) librairy (an open source computer vision library).

The next script, adapted from the oficial documentation, connect the camera, and display both the image and distance image in an infinite loop (`while True`).

1. Based on librealsense, the script activates the expected data flux, with the wanted configuration (848x480 imagein a given format at 60 Hertz):

```python
config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)
```

2. Start the aquisition process (`pipeline.start(config)`)

3. Still with librealsense, the script wait for incomming data and get them:

```
frames = pipeline.wait_for_frames()

depth_frame = frames.first(rs.stream.depth)
color_frame = frames.first(rs.stream.color)
```

4. Then, the reminder of the script consists in converting and displaying the data based on _Numpy_ and _OpenCV_


```python
#!/usr/bin/env python3
## Doc: https://dev.intelrealsense.com/docs/python2

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import signal, time, numpy as np
import sys, cv2, rclpy

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

print( f"Connect: {device_product_line}" )
found_rgb = True
for s in device.sensors:
    print( "Name:" + s.get_info(rs.camera_info.name) )
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True

if not (found_rgb):
    print("Depth camera equired !!!")
    exit(0)

config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)

# Capture ctrl-c event
isOk= True
def signalInteruption(signum, frame):
    global isOk
    print( "\nCtrl-c pressed" )
    isOk= False

signal.signal(signal.SIGINT, signalInteruption)

# Start streaming
pipeline.start(config)

count= 1
refTime= time.process_time()
freq= 60

sys.stdout.write("-")

while isOk:
    # Wait for a coherent tuple of frames: depth, color and accel
    frames = pipeline.wait_for_frames()

    color_frame = frames.first(rs.stream.color)
    depth_frame = frames.first(rs.stream.depth)
    
    if not (depth_frame and color_frame):
        continue
    
    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

    depth_colormap_dim = depth_colormap.shape
    color_colormap_dim = color_image.shape

    sys.stdout.write( f"\r- {color_colormap_dim} - {depth_colormap_dim} - ({round(freq)} fps)" )

    # Show images
    images = np.hstack((color_image, depth_colormap))

    # Show images
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', images)
    cv2.waitKey(1)
    
    # Frequency:
    if count == 10 :
        newTime= time.process_time()
        freq= 10/((newTime-refTime))
        refTime= newTime
        count= 0
    count+= 1

# Stop streaming
print("\nEnding...")
pipeline.stop()
```

To notice the use of `signal` python librairy that permit to catch and process interuption signal (`ctrl-c`).

## Publish sensor-data

Stating from the previous script, the goal is to encapsulated the connection to the camera into a _ROS2_ _Node_ in a `tuto_vision` package (only the camera image flux).

Considering previous developed _ROS2_ _Node_ :

- We want to keep the control on the infinite loop.
- We will publish `sensor_msgs/image`. The [ros2 documentation](https://github.com/ros2/common_interfaces/tree/galactic/sensor_msgs) is verry poor, but [_ROS1_ wiki](http://wiki.ros.org/sensor_msgs?distro=noetic) remains valuable.


### Node structure:

To control the infinite loop we will prefer `spin_once` to `spin`.
To notice that `spin_once` process once the ROS2 instructions.
It blocks until an event occurs.
It is possible to overpass that by specifying a timeout (`spin_once(myNode, timeout_sec=0.01)`).

At the end we want a ROS2 Node that connect the camera and publish continously the images (color and depth images).
The python function of the Node will look like:

```python
# Node processes:
def process_img(args=None):
    rclpy.init(args=args)
    rsNode= Realsense()
    while isOk:
        rsNode.read_imgs()
        rsNode.publish_imgs()
        rclpy.spin_once(rsNode, timeout_sec=0.001)
    # Stop streaming
    print("Ending...")
    rsNode.pipeline.stop()
    # Clean end
    rsNode.destroy_node()
    rclpy.shutdown()
```

You ca start from a blanc class `Realsence` and fill the differents methods step by steps (testing your code at each steps):

```python
# Realsense Node:
class Realsense(Node):
    def __init__(self, fps= 60):
        super().__init__('realsense')

    def read_imgs(self):
        pass

    def publish_imgs(self):
        pass
```

### Publish Images:

- `sensor_msgs` include [header](http://docs.ros.org/en/api/std_msgs/html/msg/Header.html) to state for spacio-temporal information. Mainly the reference frame (ie. `cam` for instance) and time. For time stamp, `get_clock()` permits to get a clock of a Node instance (`node= Node()` or `self` in case of ineritance) then `now()` and `to_msg()` methods respectivelly provide curent `time()` and convert it into a msg compliant format.

```python
msg.header.stamp = node.get_clock().now().to_msg()
```

Then it is possible to feed `sensor_msgs/image` attributs (starting with `msg.encoding= "bgr8"` seems a good idea.)

However, a librairy provides some tool to work both with ROS and OpenCV ([cv_bridge](http://wiki.ros.org/cv_bridge)).
The code for image to ROS message is:

```python
from cv_bridge import CvBridge

self.bridge=CvBridge()

msg_image = self.bridge.cv2_to_imgmsg(color_image,"bgr8")
msg_image.header.stamp = self.get_clock().now().to_msg()
msg_image.header.frame_id = "image"
self.image_publisher.publish(msg_image)
```

The code for depth image to ROS message is:

```python
from cv_bridge import CvBridge

self.bridge=CvBridge()

# Utilisation de colormap sur l'image depth de la Realsense (image convertie en 8-bit par pixel)
depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

msg_depth = self.bridge.cv2_to_imgmsg(depth_colormap,"bgr8")
msg_depth.header.stamp = msg_image.header.stamp
msg_depth.header.frame_id = "depth"
self.depth_publisher.publish(msg_depth)
```

### Some test:

At this point it is not relevant any more to show the images inside a `CV2` window or to compute and print a frequency.

The frequency can be conputed with ROS2 tool: `ros2 topic hz \img`.

The images are displayable into rviz2 program.

## Going futher:

Play with the other streams provided with the _RealSens_ sensor.

Code to add both infrared channels

```python
self.config.enable_stream(rs.stream.infrared, 1, 848, 480, rs.format.y8, 60)
self.config.enable_stream(rs.stream.infrared, 2, 848, 480, rs.format.y8, 60)

self.infra_publisher_1 = self.create_publisher(Image, 'infrared_1',10) 
self.infra_publisher_2 = self.create_publisher(Image, 'infrared_2',10)
infra_image_1 = np.asanyarray(infra_frame_1.get_data())
infra_image_2 = np.asanyarray(infra_frame_2.get_data())

in the loop :

    infra_frame_1 = frames.get_infrared_frame(1)
    infra_frame_2 = frames.get_infrared_frame(2)
    # Utilisation de colormap sur l'image infrared de la Realsense (image convertie en 8-bit par pixel)
	infra_colormap_1 = cv2.applyColorMap(cv2.convertScaleAbs(infra_image_1, alpha=0.03), cv2.COLORMAP_JET)
		
	# Utilisation de colormap sur l'image infrared de la Realsense (image convertie en 8-bit par pixel)
	infra_colormap_2 = cv2.applyColorMap(cv2.convertScaleAbs(infra_image_2, alpha=0.03), cv2.COLORMAP_JET)	
    
    msg_infra = self.bridge.cv2_to_imgmsg(infra_colormap_1,"bgr8")
    msg_infra.header.stamp = msg_image.header.stamp
    msg_infra.header.frame_id = "infrared_1"
    self.infra_publisher_1.publish(msg_infra)

    msg_infra = self.bridge.cv2_to_imgmsg(infra_colormap_2,"bgr8")
    msg_infra.header.stamp = msg_image.header.stamp
    msg_infra.header.frame_id = "infrared_2"
    self.infra_publisher_2.publish(msg_infra)

```
