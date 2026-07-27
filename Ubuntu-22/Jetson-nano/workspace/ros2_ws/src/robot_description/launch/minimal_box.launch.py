import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    package_share = get_package_share_directory(
        'robot_description'
    )

    urdf_path = os.path.join(
        package_share,
        'urdf',
        'minimal_box.urdf'
    )

    with open(urdf_path, 'r') as file:
        robot_description = file.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description
            }
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz'
    )

    return LaunchDescription([
        robot_state_publisher,
        rviz,
    ])
