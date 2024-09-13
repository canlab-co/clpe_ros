# About

This repository contains ROS 1 and ROS 2 drivers for the CANLAB CLPE-G-Series system.
CLPE-G-Series is a multi camera grabber board for N2.0 GMSL camera.
This drivers exposes camera images and information from CLPE-G-Series as ROS messages using the image_transport framework.

**This branch contains the `ROS 1` driver. For the `ROS 2` driver, please switch to the `main` branch.**

# System Requirements

Requirements:
  * CANLAB CLPE-G-Series with supplied PC running [Ubuntu 20.04](https://releases.ubuntu.com/20.04/)
  * [CLPE_G_Series_SDK](https://github.com/canlab-co/CLPE_G_CANLAB_SDK/tree/noetic)
  * [ROS 1 Noetic](http://wiki.ros.org/noetic)

> Note: It is strongly recommended to install `Ubuntu 20.04` on the PC shipped with CLPE-G-Series. This way ROS 1 binaries can be [installed as debian packages](http://wiki.ros.org/noetic/Installation/Ubuntu). If the PC is running `Ubuntu 18.04`, ROS 1 will need to be [built from source](http://wiki.ros.org/noetic/Installation/Source) which is not officially supported on `Ubuntu 18.04`.
Alternatively, containers or VM may be used but the CLPE-G-Series drivers has to be properly passed through.

# Setup

## System dependencies
```bash
sudo apt update && sudo apt install git cmake wget gcc g++ libgstreamer-plugins-base1.0-dev python3-colcon* python3-rosdep python3-vcstool -y
```

## ROS 1 Installation
Depending on the version of Ubuntu installed, follow the instructions in the links above to install ROS 1 Noetic binaries or from source.
To source ROS 1
```bash
source /opt/ros/noetic/setup.bash # if binaries are installed
source ~/ros1_noetic/install/setup.bash # if installed from source following link above
```

## Install CLPE SDK and ROS 1 Driver
Create workspace
```bash
mkdir -p ~/ws_clpe/src
cd ~/ws_clpe/
wget https://raw.githubusercontent.com/canlab-co/clpe_ros/noetic/clpe.repos
vcs import src < clpe.repos
```

Install dependencies

```bash
cd ~/ws_clpe
# source your ROS 1 workspace before this command
sudo rosdep init # if you have not done this before
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro noetic -y
```
Build the driver

```bash
cd ~/ws_clpe
# source your ROS 1 workspace if you have not already
catkin_make install --cmake-args -DCMAKE_BUILD_TYPE=Release
# To also build the benchmarking scripts
catkin_make install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCLPE_ROS_BUILD_BENCHMARKS=On
```

# Run the driver

```bash
cd ~/ws_clpe
source install/setup.bash
** UYVY **
roslaunch clpe_ros clpe_ros.launch password:=<sudo-password> slave:=n encoding:=yuv422 timestamp:=xavier
** JPEG **
roslaunch clpe_ros clpe_ros.launch password:=<sudo-password> slave:=n encoding:=jpeg timestamp:=local
```

> Note: If `sudo-password` is a number, say `42`, you should pass it as `password:=\'42\'`\
> If you have not equipped slave CLPE-G-Series, you should enter `slave:=n`

By default the driver will publish topics per camera (X).
* /clpe_ros/cam_X/image_raw: The raw image published as `sensor_msgs::Image`. The default encoding is `yuv422`. For other supported encodings, see Configuration below.

## Visualizing in Rviz

By default, images are published in yuv encoding. Rviz does not support yuv so it needs to be converted to a different format.

```bash
roslaunch clpe_ros clpe_ros.launch password:=<sudo-password> slave:=y encoding:=bgr8 timestamp:=xavier
```

Note that this conversion incurs significant overhead so should be avoided if performance is important.

# Configuration

The driver supports the following ROS parameters to configure its behavior. The `password`, `slave`, `encoding` and `timestamp` parameters can be overwritten at run time. Other parameters can be modified within the [config file](config/clpe_config.yaml)

| Key | Description | Default |
|-|-|-|
| password | sudo password | *\*Required\** |
| slave | check slave, supported arguments are: y, n. Defaults to n. | n |
| encoding | Image encoding, supported formats are: bgr8, bgra8, rgb8, rgba8, mono16, yuv422, jpeg. Defaults to yuv422. Note that encodings other than yuv422 and jpeg incurs conversion overhead. | yuv422 |
| timestamp | Image timestamp, supported formats are: xavier, local. Defaults to xavier. | xavier |
| cam_{n}_enable | Enable camera | true |
| cam_{n}_pose | Pose relative to the base, 6 values corresponding to [x, y, z, roll, pitch, yaw] | [0,0,0,0,0,0] |
| cam_{n}_frame_id | Defines the frame_id all static transformations refers to | base_link |
| cam_{n}_image_latch | Controls latching for camera image messages | false |
| cam_{n}_image_queue_size | Controls queue size for camera image messages | 10 |
| cam_{n}_info_latch | Controls latching for camera info messages | false |
| cam_{n}_info_queue_size | Controls queue size for camera info messages | 10 |

Unless specified otherwise, all parameters are read-only, that is, they must be initialized at startup and cannot be changed at runtime.
