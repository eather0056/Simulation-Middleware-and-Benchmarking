from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static transform from world to robot1's odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'robot1/odom']
        ),
        # Static transform from world to robot2's odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['1', '0', '0', '0', '0', '0', 'world', 'robot2/odom']
        ),
        # Static transform from robot1's odom to robot1's base_footprint
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'robot1/odom', 'robot1/base_footprint']
        ),
        # Static transform from robot2's odom to robot2's base_footprint
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'robot2/odom', 'robot2/base_footprint']
        ),
        # Launch the wander_and_avoid node for robot1
        Node(
            package='wander_and_avoid',
            executable='wander_and_avoid',
            name='wander_and_avoid'
        ),
        # Launch the follower_robot node for robot2
        Node(
            package='wander_and_avoid',
            executable='follower_robot',
            name='follower_robot'
        ),
    ])
