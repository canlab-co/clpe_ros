# About

This repository contains ROS 1 and ROS 2 drivers for the CANLAB CLPE-G-NVP2650D system.
CLPE-G-NVP2650D is a multi camera grabber board for N2.0 GMSL camera.
This drivers exposes camera images and information from CLPE-G-NVP2650D as ROS messages using the image_transport framework.

**This branch contains the `ROS 1` driver. For the `ROS 2` driver, please switch to the `main` branch.**

# System Requirements

Requirements:
  * CANLAB CLPE-G-NVP2650D with supplied PC running [Ubuntu 20.04](https://releases.ubuntu.com/20.04/)
  * [CLPE_G_NVP2650D_SDK](https://github.com/canlab-co/CLPE_G_NVP2650D_SDK)
  * [ROS 1 Noetic](http://wiki.ros.org/noetic)

> Note: It is strongly recommended to install `Ubuntu 20.04` on the PC shipped with CLPE-G-NVP2650D. This way ROS 1 binaries can be [installed as debian packages](http://wiki.ros.org/noetic/Installation/Ubuntu). If the PC is running `Ubuntu 18.04`, ROS 1 will need to be [built from source](http://wiki.ros.org/noetic/Installation/Source) which is not officially supported on `Ubuntu 18.04`.
Alternatively, containers or VM may be used but the CLPE-G-NVP2650D drivers has to be properly passed through.

# Setup

## System dependencies
```bash
sudo apt update && sudo apt install git cmake wget gcc python3-colcon* python3-rosdep python3-vcstool -y
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
roslaunch clpe_ros clpe_ros.launch password:=<sudo-password> encoding:=yuv422
```

> Note: If `sudo-password` is a number, say `42`, you should pass it as `password:=\'42\'`

By default the driver will publish two topics per camera (X).
* /clpe_ros/cam_X/image_raw: The raw image published as `sensor_msgs::Image`. The default encoding is `yuv422`. For other supported encodings, see Configuration below.
* /clpe_ros/cam_X/camera_info: The intrinsics of the camera published as a `sensor_msgs::CameraInfo` message.
* /clpe_ros/cam_X/clpe_camera_info: The intrinsics of the camera published as a clpe_ros_msgs::ClpeCameraInfo message.

## Visualizing in Rviz

By default, images are published in yuv encoding. Rviz does not support yuv so it needs to be converted to a different format.

```bash
roslaunch clpe_ros clpe_ros.launch password:=<sudo-password> encoding:=bgr8
```

Note that this conversion incurs significant overhead so should be avoided if performance is important.

# Configuration

The driver supports the following ROS parameters to configure its behavior. The `password` and `encoding` parameters can be overwritten at run time. Other parameters can be modified within the [config file](config/clpe_config.yaml)

| Key | Description | Default |
|-|-|-|
| password | sudo password | *\*Required\** |
| encoding | Image encoding, supported formats are: bgr8, bgra8, rgb8, rgba8, mono16, yuv422. Defaults to yuv422. Note that encodings other than yuv422 incurs conversion overhead. | yuv422 |
| cam_{n}_enable | Enable camera | true |
| cam_{n}_pose | Pose relative to the base, 6 values corresponding to [x, y, z, roll, pitch, yaw] | [0,0,0,0,0,0] |
| cam_{n}_frame_id | Defines the frame_id all static transformations refers to | base_link |
| cam_{n}_image_latch | Controls latching for camera image messages | false |
| cam_{n}_image_queue_size | Controls queue size for camera image messages | 10 |
| cam_{n}_info_latch | Controls latching for camera info messages | false |
| cam_{n}_info_queue_size | Controls queue size for camera info messages | 10 |

Unless specified otherwise, all parameters are read-only, that is, they must be initialized at startup and cannot be changed at runtime.
