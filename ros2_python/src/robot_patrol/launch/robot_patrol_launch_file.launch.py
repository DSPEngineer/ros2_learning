from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the 'patrol_node' from the 'robot_patrol' package
        Node(
            package='robot_patrol',
            executable='patrol_node',
            output='screen',
            emulate_tty=True
        ),
    ])
