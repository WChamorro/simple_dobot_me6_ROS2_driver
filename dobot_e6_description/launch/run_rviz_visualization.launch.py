import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'dobot_e6_description'

    package_share = get_package_share_directory(
        package_name
    )

    urdf_file = os.path.join(
        package_share,
        'urdf',
        'me6_robot.urdf'
    )

    rviz_config_file = os.path.join(
    
        package_share,
        'rviz',
        'visualization.rviz'
    )

    # ========================================================
    # Robot description
    # ========================================================

    robot_description = open(
        urdf_file,
        'r',
        encoding='utf-8'
    ).read()

    # ========================================================
    # Robot State Publisher
    # ========================================================

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description
            }
        ]
    )

    # ========================================================
    # RViz
    # ========================================================

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            rviz_config_file
        ]
    )

    # ========================================================
    # Launch Description
    # ========================================================

    launch_description = LaunchDescription()

    launch_description.add_action(
        robot_state_publisher_node
    )

    launch_description.add_action(
        rviz_node
    )

    return launch_description