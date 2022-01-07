# Image processing with 3D sensor

Vision based 3D sensor provides both an image, and an estimation of the position of each pixel.

## Install RealSense

Install RealSense drivers: ([from instruction here](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md))


First you want to configure the software manager (`apt`) to follow Intel Realsense repository.

1. Register the server's public key: `sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE` or, if you are on a restricted network with closed ports... `sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE`
2. Then add the server to the list of repositories: `sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u`

Now, you can easily install the libraries and tools:

```bash
sudo apt install \
    librealsense2-dkms \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg \
```

Connect the Intel RealSense depth camera and run: `realsense-viewer`.

## Install appropriate ROS packages

Ressources:

* [wiki ros](http://wiki.ros.org/RealSense) - The interesting pkg is the **realsense2_camera**
* [Installation](https://github.com/IntelRealSense/realsense-ros#installation-instructions)

Installation:

```bash
sudo apt-get install \
    ros-noetic-librealsense2 \
    ros-noetic-realsense2-camera \
    ros-noetic-realsense2-description
```

Then you can launch the camera to publish the data into ROS topics:

```bash
roslaunch realsense2-camera rs_camera.launch
```

And observe the topic and mainly the one publishing the images (`camera/color/image_raw`) with `rostopic list`,  `rostopic info` and  `rostopic hz`.

You can visualise the image with:

```bash
rosrun image_view image_view image:=camera/color/image_raw
```

## ROS Image format.

Image are published as 'sensor_msgs/Image' in `camera/color/image_raw`

* [ROS doc](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)

So the pixel value are stored in `img.data` array but several tool to convert ROS image to OpenCV images already exist (for instance [cv_bridge](http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython))
