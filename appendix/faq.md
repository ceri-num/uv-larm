# Frequent Asked Question...

## There is no Wifi on my dell-xps13 ???

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
cmake -D CMAKE_BUILD_RELEASE ..
make -j8

ls -l bin/opencv_createsamples
```

