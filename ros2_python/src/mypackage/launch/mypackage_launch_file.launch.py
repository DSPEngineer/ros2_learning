# Launch file for the simply.py package
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mypackage',
            executable='simple_node',
            #name='simple_node',
            #parameters=[{'use_sim_time': True}],
            #remappings=[
            #    ('/input_topic', '/output_topic')
            #],
            output='screen',
        )
    ])
