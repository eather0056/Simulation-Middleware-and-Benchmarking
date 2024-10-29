from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    kobuki_cmd_ex = Node(package='br2_vff_avoidance_py',
                      executable='goal_navigator',
                      output='screen',
                      parameters=[{
                        'use_sim_time': True
                      }],
                      remappings=[
                        ('input_scan', '/robot1/scan'),
                        ('output_vel', '/robot1/cmd_vel')
                      ])

    ld = LaunchDescription()
    ld.add_action(kobuki_cmd_ex)

    return ld