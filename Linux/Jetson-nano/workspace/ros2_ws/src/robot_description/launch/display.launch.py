import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import (
    get_package_share_directory
)


def generate_launch_description():

    package_share = get_package_share_directory(
        'robot_description'
    )

    urdf_file = os.path.join(
        package_share,
        'urdf',
        'simple_car.urdf'
    )

    with open(
        urdf_file,
        'r'
    ) as file:

        robot_description = file.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',

        parameters=[
            {
                'robot_description':
                    robot_description
            }
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ])
