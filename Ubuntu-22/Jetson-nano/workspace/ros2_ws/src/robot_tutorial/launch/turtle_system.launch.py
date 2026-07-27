from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim'
    )

    controller_node = Node(
        package='robot_tutorial',
        executable='turtle_controller',
        name='turtle_controller',
        parameters=[
            {
                'target_x': 8.0,
                'linear_speed': 1.0,
                'angular_speed': 1.0,
            }
        ]
    )

    return LaunchDescription([
        turtlesim_node,
        controller_node,
    ])
