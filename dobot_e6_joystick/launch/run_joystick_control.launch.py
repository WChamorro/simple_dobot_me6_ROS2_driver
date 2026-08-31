import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'dobot_e6_joystick'

    package_share = get_package_share_directory(
        package_name
    )

    joystick_config = os.path.join(
        package_share,
        'config',
        'joystick.yaml'
    )

    # ============================================================
    # Launch arguments
    # ============================================================

    device_id_argument = DeclareLaunchArgument(
        'device_id',
        default_value='0',
        description='Joystick device ID'
    )

    device_id = LaunchConfiguration(
        'device_id'
    )

    # ============================================================
    # ROS 2 joy node
    # ============================================================

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[
            {
                'device_id': device_id,

                # Deadzone is handled by our Dobot joystick node
                'deadzone': 0.0,

                # Keep /joy publishing even if the stick
                # remains at a constant position
                'autorepeat_rate': 30.0,
            }
        ]
    )

    # ============================================================
    # Dobot E6 joystick controller
    # ============================================================

    dobot_joystick_node = Node(
        package='dobot_e6_joystick',
        executable='joystick_node',
        name='dobot_e6_joystick',
        output='screen',
        parameters=[
            joystick_config
        ]
    )

    # ============================================================
    # Launch description
    # ============================================================

    launch_description = LaunchDescription()

    launch_description.add_action(
        device_id_argument
    )

    launch_description.add_action(
        joy_node
    )

    launch_description.add_action(
        dobot_joystick_node
    )

    return launch_description