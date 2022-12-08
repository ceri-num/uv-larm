# Frequent Asked Question...

## How to install Ubuntu ?

Ubuntu is a fork of the Debian project, a Linux-based desktop operating system.

  - Official website : <https://ubuntu.com/>
  - French community : <https://www.ubuntu-fr.org/>

Notice that, ubuntu can be installed in double boot mode in parallel to a Microsoft OS on your personal computer.
It is not recommended to use Ubuntu+ROS in a virtual machine (the performances would be  poor).

### To-do:
  - Install Ubuntu **20.04 LTS** (Long Term Supported version) from live USB-Key
  - Ideally, use all the hard disk (you can split the disk in advance for [double-boot install](https://help.ubuntu.com/community/WindowsDualBoot))
  - Configure "bot" username and "bot" password.
  - Configure network
  - Login and upgrade your installation

```bash
sudo apt update
sudo apt upgrade
```

## There is no Wifi on my dell-xps13 ?

1. Connect with cable
2. Get the appropriate drivers: [killer driver](https://support.killernetworking.com/knowledge-base/killer-ax1650-in-debian-ubuntu-16-04/)

```bash
sudo add-apt-repository ppa:canonical-hwe-team/ backport-iwlwifi
sudo apt-get update
sudo apt-get install backport-iwlwifi-dkms
```

3. reboot

## Catkin_create_pkg - invalid email ?

you can use the `-m` option to force an author name.

```bash
catin_create_pkg -m AuthorName package_name [dependencies...]
```

## `opencv_createsamples`

The latest OpenCV does not include  `opencv_createsamples`.
Let's compile an older version (~5 min on labs PC).

```bash
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout 3.4.17
mkdir build
cd build
cmake -D'CMAKE_BUILD_TYPE=RELEASE' ..
make -j8

ls -l bin/opencv_createsamples
```

## How to get an aligned depth image to the color image ?

 you can use the `align_depth:=true` ROS parameter. The aligned image is streamed in a specific topic. (tks Orange group)

```bash
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```
