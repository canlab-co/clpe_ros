#!/usr/env python3

import time

import rclpy
from sensor_msgs.msg import Image
from builtin_interfaces.msg import Time

rclpy.init()
node = rclpy.create_node("canlab_benchmark");

def from_ros_time(ros_time: Time) -> float:
  return ros_time.sec + ros_time.nanosec / 1000000000

def on_image(cam_id: int, image: Image) -> float:
  local_time = from_ros_time(node.get_clock().now().to_msg());
  cam_time = from_ros_time(image.header.stamp)
  print(f"cam_{cam_id} latency = {(local_time - cam_time) * 1000000:.3f}us");

for i in range(4):
  node.create_subscription(Image, f"cam_{i}/image_raw", lambda image, i=i: on_image(i, image), 10)

rclpy.spin(node)
