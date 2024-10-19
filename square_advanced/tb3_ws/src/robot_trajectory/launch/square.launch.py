from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_trajectory',
            executable='square',
            name='square_trajectory',
            output='screen',
            parameters=[
                {'linear_speed': 0.1, 'angular_speed': 1.57, 'square_length': 2.0}
            ],
            remappings=[
                ('/cmd_vel', '/turtle1/cmd_vel')
            ]
        )
    ])

