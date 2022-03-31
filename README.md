# About

This repository contains ROS 1 and ROS 2 drivers for the CANLAB CLPE-G-NVP2650D system.
CLPE-G-NVP2650D is a multi camera grabber board for N2.0 GMSL camera.
This drivers exposes camera images and information from CLPE-G-NVP2650D as ROS messages using the image_transport framework.

This branch contains the `ROS 2` driver. For the `ROS 1` driver, please switch to the `noetic` branch.

# System Requirements

Requirements:
  * CANLAB CLPE-G-NVP2650D with supplied PC running [Ubuntu 20.04](https://releases.ubuntu.com/20.04/)
  * [ROS 2 Foxy](https://docs.ros.org/en/foxy/index.html) or [ROS 2 Galactic](https://docs.ros.org/en/galactic/index.html)

> Note: It is strongly recommended to install `Ubuntu 20.04` on the PC shipped with CLPE-G-NVP2650D. This way ROS 2 binaries can be [installed as debian packages](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html). If the PC is running `Ubuntu 18.04`, ROS 2 will need to be [built from source](https://docs.ros.org/en/foxy/Installation/Ubuntu-Development-Setup.html) which is not officially supported on `Ubuntu 18.04`.
Alternatively, containers or VM may be used but the CLPE-G-NVP2650D drivers has to be properly passed through.


# Setup

## System dependencies
```bash
sudo apt update && sudo apt install git cmake python3-colcon* -y
sudo apt install python3-rosdep
```

## ROS 2 Installation
Depending on the version of Ubuntu installed, follow the instructions in the links above to install ROS 2 binaries or from source.
To source ROS 2
```bash
source /opt/ros/foxy/setup.bash # if binaries are installed
source ~/ros2_foxy/install/setup.bash # if installed from source following link above
```

## Install ROS 2 Driver
Create workspace
```bash
mkdir -p ~/ws_clpe/src
cd ~/ws_clpe/src
git clone https://github.com/osrf/canlab
```

Install dependencies

```bash
cd ~/ws_clpe
# source your ROS 2 workspace before this command
sudo rosdep init # if you have not done this before
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

Build the driver

```bash
cd ~/ws_clpe
# source your ROS 2 workspace if you have not already
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

# Run the driver

```bash
cd ~/ws_clpe
source install/setup.bash
ros2 launch clpe_ros clpe_ros.launch.py password:=<sudo-password>
```

> Note: If `sudo_password` is a number, say `42`, you should pass it as `password:=\'42\'`

By default the driver will publish two topics per camera (X).
* /clpe/cam_X/image_raw: The raw image published as `sensor_msgs::msg::Image`. The default encoding is `yuv422`. For other supported encodings, see Configuration below.
* /clpe/cam_X/camera_info: The intrinsics of the camera published as a `sensor_msgs::msg::CameraInfo` message.

# Configuration

The driver supports the following ros parameters to configure it's behavior.

| Key | Description | Default |
|-|-|-|
| password | sudo password | *\*Required\** |
| encoding | Image encoding, supported formats are: bgr8, bgra8, rgb8, rgba8, mono16, yuv422. Defaults to yuv422. Note that encodings other than yuv422 incurs conversion overhead. | yuv422 |
| cam_{n}_enable | Enable camera | true |
| cam_{n}_pose | Pose relative to the base, 6 values corresponding to [x, y, z, roll, pitch, yaw] | [0,0,0,0,0,0] |
| cam_{n}_frame_id | Defines the frame_id all static transformations refers to | base_link |
| cam_{n}_image_qos | Sets the QoS by which the topic is published. Available values are the following strings: SYSTEM_DEFAULT, PARAMETER_EVENTS, SERVICES_DEFAULT, PARAMETERS, DEFAULT, SENSOR_DATA, HID_DEFAULT (= DEFAULT with depth of 100), EXTRINSICS_DEFAULT (= DEFAULT with depth of 1 and transient local durabilty). | SENSOR_DATA |
| cam_{n}_info_qos | Sets the QoS by which the topic is published. Available values are the following strings: SYSTEM_DEFAULT, PARAMETER_EVENTS, SERVICES_DEFAULT, PARAMETERS, DEFAULT, SENSOR_DATA, HID_DEFAULT (= DEFAULT with depth of 100), EXTRINSICS_DEFAULT (= DEFAULT with depth of 1 and transient local durabilty). | SYSTEM_DEFAULT |

Unless specified otherwise, all parameters are read-only, that is, they must be initialized at startup and cannot be changed at runtime.
