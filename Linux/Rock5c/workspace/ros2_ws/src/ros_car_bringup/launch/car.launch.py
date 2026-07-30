from pathlib import Path

from ament_index_python.packages import (
    get_package_share_directory,
)

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    bringup_dir = Path(
        get_package_share_directory(
            'ros_car_bringup'
        )
    )

    description_dir = Path(
        get_package_share_directory(
            'ros_car_description'
        )
    )

    base_config = (
        bringup_dir
        / 'config'
        / 'base.yaml'
    )

    urdf_file = (
        description_dir
        / 'urdf'
        / 'ros_car.urdf'
    )

    robot_description = (
        urdf_file.read_text()
    )

    base_driver = Node(
        package='ros_car_base',
        executable='base_driver',
        name='base_driver',

        parameters=[
            str(base_config),
        ],

        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',

        parameters=[
            {
                'robot_description':
                    robot_description
            }
        ],

        output='screen',
    )

    return LaunchDescription([
        base_driver,
        robot_state_publisher,
    ])
