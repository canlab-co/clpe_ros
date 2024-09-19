# About

This repository contains ROS 1 and ROS 2 drivers for the CANLAB CLPE-G-Series system.
CLPE-G-Series is a multi camera grabber board for N2.0 GMSL camera.
This drivers exposes camera images and information from CLPE-G-Series as ROS messages using the image_transport framework.

**This branch contains the `ROS 2` driver. For the `ROS 1` driver, please switch to the `noetic` branch.**

# System Requirements

Requirements:
  * CANLAB CLPE-G-Series with supplied PC running [Ubuntu 20.04](https://releases.ubuntu.com/20.04/)
  * [CLPE_G_Series_SDK](https://github.com/canlab-co/CLPE_G_CANLAB_SDK)
  * [ROS 2 Foxy](https://docs.ros.org/en/foxy/index.html) or [ROS 2 Galactic](https://docs.ros.org/en/galactic/index.html)

> Note: It is strongly recommended to install `Ubuntu 20.04` on the PC shipped with CLPE-G-Series. This way ROS 2 binaries can be [installed as debian packages](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html). If the PC is running `Ubuntu 18.04`, ROS 2 will need to be [built from source](https://docs.ros.org/en/foxy/Installation/Ubuntu-Development-Setup.html) which is not officially supported on `Ubuntu 18.04`.
Alternatively, containers or VM may be used but the CLPE-G-Series drivers has to be properly passed through.


# Setup

## System dependencies
```bash
sudo apt update && sudo apt install git cmake wget gcc g++ libgstreamer-plugins-base1.0-dev python3-colcon* python3-rosdep python3-vcstool -y
```

## ROS 2 Installation
Depending on the version of Ubuntu installed, follow the instructions in the links above to install ROS 2 binaries or from source.
To source ROS 2
```bash
source /opt/ros/foxy/setup.bash # if binaries are installed
source ~/ros2_foxy/install/setup.bash # if installed from source following link above
```
> Note: Replace `foxy` with `galactic` if using ROS 2 Galactic

## Install CLPE SDK and ROS 2 Driver
Create workspace
```bash
mkdir -p ~/ws_clpe2/src
cd ~/ws_clpe2/
wget https://raw.githubusercontent.com/canlab-co/clpe_ros/main/clpe.repos
vcs import src < clpe.repos
```

Install dependencies

```bash
cd ~/ws_clpe2
# source your ROS 2 workspace before this command
sudo rosdep init # if you have not done this before
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

Build the driver

```bash
cd ~/ws_clpe2
# source your ROS 2 workspace if you have not already
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
# To also build the benchmarking scripts
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCLPE_ROS_BUILD_BENCHMARKS=On
```

# Run the driver

## DDS Configuration
For better image transport performance over DDS, we recommend using [FastDDS](https://github.com/eProsima/Fast-DDS) with Shared Memory Transport enabled.
First copy the the `fastdds.xml` config file to a suitable directory, eg. `$HOME/fastdds.xml`
```bash
cd ~/ws_clpe2/src
cp canlab-co/clpe_ros/fastdds.xml ~/
```

Next add these two lines to your `~/.bashrc`
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/fastdds.xml
```

Make sure to `source ~/.bashrc` first on all terminals before launching any ROS 2 nodes including the driver.

## Launch

```bash
cd ~/ws_clpe2
source install/setup.bash
** UYVY **
ros2 launch clpe_ros clpe_ros.launch.py password:=<sudo-password> slave:=n encoding:=yuv422 timestamp:=xavier
** JPEG **
ros2 launch clpe_ros clpe_ros.launch.py password:=<sudo-password> slave:=n encoding:=jpeg timestamp:=local
```

> Note: If `sudo_password` is a number, say `42`, you should pass it as `password:=\'42\'`\
> If you have not equipped slave CLPE-G-Series, you should enter `slave:=n`

By default the driver will publish topics per camera X.
* /clpe/cam_X/image_raw: The raw image published as `sensor_msgs::msg::Image`.
* /clpe/cam_X/compressed: The JPEG image published as `sensor_msgs::msg::CompressedImage`.
* The default encoding is `yuv422`. For other supported encodings, see Configuration below.


## ROS2 Component

The driver is also supported as a ROS2 component. To run it, start the component container

```bash
ros2 run rclcpp_components component_container
```

In another terminal, load the component

```bash
ros2 component load /ComponentManager clpe_ros clpe::ClpeComponentNode -p password:=<password>
```

Check that the component is loaded with

```bash
ros2 component list
```

# Configuration

The driver supports the following ROS parameters to configure its behavior. The `password`, `slave`, `encoding` and `timestamp` parameters can be overwritten at run time. Other parameters can be modified within the [launch file](launch/clpe_ros.launch.py).
> Note: If encoding is "jpeg", timestamp must be set to "local".

| Key | Description | Default |
|-|-|-|
| password | sudo password | *\*Required\** |
| slave | check slave, supported arguments are: y, n. Defaults to n. | n |
| encoding | Image encoding, supported formats are: bgr8, bgra8, rgb8, rgba8, mono16, yuv422, jpeg. Defaults to yuv422. Note that encodings other than yuv422 and jpeg incurs conversion overhead. | yuv422 |
| timestamp | Image timestamp, supported formats are: xavier, local. Defaults to xavier. | xavier |
| cam_{n}_enable | Enable camera | true |
| cam_{n}_pose | Pose relative to the base, 6 values corresponding to [x, y, z, roll, pitch, yaw] | [0,0,0,0,0,0] |
| cam_{n}_frame_id | Defines the frame_id all static transformations refers to | base_link |
| cam_{n}_image_qos | Sets the QoS by which the topic is published. Available values are the following strings: SYSTEM_DEFAULT, PARAMETER_EVENTS, SERVICES_DEFAULT, PARAMETERS, DEFAULT, SENSOR_DATA, HID_DEFAULT (= DEFAULT with depth of 100), EXTRINSICS_DEFAULT (= DEFAULT with depth of 1 and transient local durabilty). | SENSOR_DATA |
| cam_{n}_info_qos | Sets the QoS by which the topic is published. Available values are the following strings: SYSTEM_DEFAULT, PARAMETER_EVENTS, SERVICES_DEFAULT, PARAMETERS, DEFAULT, SENSOR_DATA, HID_DEFAULT (= DEFAULT with depth of 100), EXTRINSICS_DEFAULT (= DEFAULT with depth of 1 and transient local durabilty). | SYSTEM_DEFAULT |

Unless specified otherwise, all parameters are read-only, that is, they must be initialized at startup and cannot be changed at runtime.
