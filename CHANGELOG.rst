^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clpe_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
0.1.1 (2022-08-24)
------------------
* remove eeprom

0.1.0 (2022-04-20)
------------------
* Provides ROS 1 Noetic driver for CANLAB CLPE-G-NVP2650D vision system
* By default the driver will publish three topics for each camera X.
  * /clpe_ros/cam_X/image_raw: The raw image published as sensor_msgs::msg::Image. The default encoding is yuv422. For other supported encodings, see Configuration.
  * /clpe_ros/cam_X/camera_info: The intrinsics of the camera published as a sensor_msgs::msg::CameraInfo message.
  * /clpe_ros/cam_X/clpe_camera_info: The intrinsics of the camera published as a clpe_ros_msgs::ClpeCameraInfo message.
* Contributors: Teo Koon Peng, Yadunund Vijay
