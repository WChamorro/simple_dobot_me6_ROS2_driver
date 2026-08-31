import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'dobot_e6_driver'

    default_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'dobot_e6_params.yaml',
    )

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Path to the Dobot E6 driver parameter YAML file.',
        ),

        Node(
            package=package_name,
            executable='driver_node',
            name='dobot_e6_driver',
            output='screen',
            emulate_tty=True,
            parameters=[params_file],
        ),
    ])