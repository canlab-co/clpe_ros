import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    # args that can be set from the command line or a default will be used
    password_arg = DeclareLaunchArgument(
        "password", default_value=TextSubstitution(text="0")
    )

    canlab_config = os.path.join(
        get_package_share_directory("canlab_ros"),
        'config', 'canlab_config.yaml'
    )

    canlab_node = Node(
            package='canlab_ros',
            namespace='canlab',
            executable='canlab_ros',
            name='canlab_ros',
            parameters=[canlab_config, {
                "password": LaunchConfiguration('password'),
            }]
        )

    return LaunchDescription([
        password_arg,
        canlab_node,
    ])
