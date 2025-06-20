# Simple.py - a sim[ple ROS2 Node in python, example 2
#  Refer to the Python3 for Robotics course ( https://app.theconstruct.ai/courses/58 )

import rclpy
# Import the Node modile fomr the ROS2 python library
from rclpy.node import Node

class myNode(Node):
    def __init__(self):
        # Call the super() constructor to initialize the Node objet with
        # the name 'simple_ros2_node'
        ### super().__init__('simple_ros2_node')
        super().__init__('obi_wan')
        self.get_logger().info('Simple ROS2 Node has been Initialized.')
        # You can add more functionality here if needed
        self.create_timer( 0.2, self.timer_callback )

    def timer_callback(self):
        # This method will be called periodically by the timer
        self.get_logger().info('Timer callback executed.')
        self.get_logger().info('  -- Help me Obi-Wan Kenobi! You are my only hope.')


def main(args=None):
    # Initialize the ROS2 communications (Python client library)
    rclpy.init(args=args)

    # Create a Node instance
    node = myNode()

    # Log a message to indicate that the node has started
    node.get_logger().info('Simple ROS2 Node has been started.')

    # Spin the node to keep it active
    rclpy.spin(node)

    # Shutdown the ROS2 Python client library
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    print("Simple ROS2 Node has been executed successfully.")
# This script initializes a simple ROS2 node that logs a message when started.
# It uses the rclpy library to create and manage the node lifecycle.
# The node is named 'simple_node', and it logs an informational message to the console.
