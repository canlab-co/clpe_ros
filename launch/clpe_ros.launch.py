# Copyright 2022 Can-lab Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node


## Configurable parameters ##
'''Available QoS values are the following strings:
SYSTEM_DEFAULT, PARAMETER_EVENTS, SERVICES_DEFAULT, PARAMETERS, DEFAULT,
SENSOR_DATA, HID_DEFAULT (= DEFAULT with depth of 100),
EXTRINSICS_DEFAULT (= DEFAULT with depth of 1 and transient local durability).
'''
parameters = {
  "cam_0_enable": True,
  "cam_0_frame_id": "cam_0_link",
  "cam_0_image_qos": "SENSOR_DATA",
  "cam_0_info_qos": "SYSTEM_DEFAULT",
  "cam_0_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],

  "cam_1_enable": True,
  "cam_1_frame_id": "cam_1_link",
  "cam_1_image_qos": "SENSOR_DATA",
  "cam_1_info_qos": "SYSTEM_DEFAULT",
  "cam_1_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],

  "cam_2_enable": True,
  "cam_2_frame_id": "cam_2_link",
  "cam_2_image_qos": "SENSOR_DATA",
  "cam_2_info_qos": "SYSTEM_DEFAULT",
  "cam_2_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],

  "cam_3_enable": True,
  "cam_3_frame_id": "cam_3_link",
  "cam_3_image_qos": "SENSOR_DATA",
  "cam_3_info_qos": "SYSTEM_DEFAULT",
  "cam_3_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
}

def generate_launch_description():

    # args that can be set from the command line or a default will be used
    password_arg = DeclareLaunchArgument(
        "password", default_value=TextSubstitution(text="0")
    )

    encoding_arg = DeclareLaunchArgument(
        "encoding", default_value=TextSubstitution(text="yuv422")
    )

    canlab_node = Node(
            package='clpe_ros',
            namespace='clpe',
            executable='clpe_ros',
            name='clpe_ros',
            parameters=[parameters, {
                "encoding": LaunchConfiguration('encoding'),
                "password": LaunchConfiguration('password'),
            }]
        )

    return LaunchDescription([
        password_arg,
        encoding_arg,
        canlab_node,
    ])
