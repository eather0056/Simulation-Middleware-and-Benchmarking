import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Include the turtle_tf2_demo.launch.py file from the 'learning_tf2_cpp' package
    demo_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('learning_tf2_cpp'),
                'launch',  # Corrected path concatenation
                'turtle_tf2_demo.launch.py'  # No leading slash
            )
        ),
        launch_arguments={'target_frame': 'carrot1'}.items()  # Fixed argument handling
    )

    return LaunchDescription([
        demo_nodes,
        # Launch the dynamic frame broadcaster node
        Node(
            package='learning_tf2_cpp',
            executable='dynamic_frame_tf2_broadcaster',
            name='dynamic_broadcaster',
            output='screen',  # Optional: To show output in the console
        ),
    ])
