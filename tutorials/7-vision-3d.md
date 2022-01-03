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
