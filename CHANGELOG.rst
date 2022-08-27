^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clpe_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2022-08-24)
------------------
* remove EEPROM

0.1.0 (2022-04-20)
------------------
* Provides ROS 2 driver for CANLAB CLPE-G-NVP2650D vision system
* Support ROS 2 Foxy and ROS 2 Galactic
* By default the driver will publish three topics for each camera X.
  * /clpe/cam_X/image_raw: The raw image published as sensor_msgs::msg::Image. The default encoding is yuv422. For other supported encodings, see Configuration.
  * /clpe/cam_X/camera_info: The intrinsics of the camera published as a sensor_msgs::msg::CameraInfo message.
  * /clpe/cam_X/clpe_camera_info: The intrinsics of the camera published as a clpe_ros_msgs::msg::ClpeCameraInfo message.
* Contributors: Teo Koon Peng, Yadunund Vijay
