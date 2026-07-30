from pathlib import Path

from ament_index_python.packages import (
    get_package_share_directory,
)

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    package_dir = Path(
        get_package_share_directory(
            'ros_car_description'
        )
    )

    urdf_file = (
        package_dir
        / 'urdf'
        / 'ros_car.urdf'
    )

    robot_description = (
        urdf_file.read_text()
    )

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',

            parameters=[{
                'robot_description':
                    robot_description
            }],

            output='screen',
        ),

    ])
