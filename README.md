# About

<!-- FIXME: Confirm product description? -->
This is the ROS driver for CANLAB CLPE-G-NVP2650D. CLPE-G-NVP2650D is a multi camera grabber board for N2.0 GMSL camera. This drivers exposes camera images and information from CLPE-G-NVP2650D as ROS messages using the image_transport framework.

# Running

Requirements:
  * CANLAB CLPE-G-NVP2650D supplied PC
  * ROS2 foxy

The PC that comes with CLPE-G-NVP2650D should already contain a suitable ROS2 distro. If it is not available, ROS2 foxy needs to be built from source as the default OS of the PC is not supported. See https://docs.ros.org/en/foxy/Installation/Ubuntu-Development-Setup.html for instructions. Alternatively, containers or VM may be used but the CLPE-G-NVP2650D drivers has to be properly passthroughed.

```bash
source /opt/ros/foxy/setup.bash
ros2 launch clpe_ros clpe_ros.launch.py password:=<sudo-password>
```

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

# Building

## Requirements

* CANLAB CLPE-G-NVP2650D supplied PC
* ROS2 foxy
* rosdep
* colcon
* git

The PC that comes with CLPE-G-NVP2650D should already contain a suitable ROS2 distro. If it is not available, ROS2 foxy needs to be built from source as the default OS of the PC is not supported. See https://docs.ros.org/en/foxy/Installation/Ubuntu-Development-Setup.html for instructions. Alternatively, containers or VM may be used but the CLPE-G-NVP2650D drivers has to be properly passthroughed.

## Building

```bash
mkdir -p <workspace>/src
cd <workspace>/src
```

Obtain sources

```bash
git clone https://github.com/osrf/canlab
```

Install dependencies

```bash
cd <workspace>
rosdep install --from-paths src -i
```

Compile

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=DEBUG --packages-up-to clpe_ros
```
