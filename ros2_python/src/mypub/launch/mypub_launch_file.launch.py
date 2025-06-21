## Launch file for the publisher node

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mypub',
            executable='simplePublisher',
            name='simple_publisher_node',
            output='screen',
            #parameters=[{'use_sim_time': True}],
            #remappings=[('/cmd_vel', '/robot/cmd_vel')]
        )
    ])

# This launch file is used to start the SimplePublisher node from the myPub package.
# It specifies the package name, executable name, node name, output settings,
# parameters, and topic remappings.