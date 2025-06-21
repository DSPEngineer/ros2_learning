# Launch file for Subscriber node
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mysub',
            executable='simpleSubscriber',
            name='simple_subscriber_node',
            output='screen',
            ## parameters=[{'use_sim_time': True}],
            ## remappings=[('/cmd_vel', '/cmd_vel')]
        )
    ])
# This launch file is used to start the simple subscriber node in the mysub package.
# It specifies the package name, executable name, node name, output type, parameters, and topic remappings.