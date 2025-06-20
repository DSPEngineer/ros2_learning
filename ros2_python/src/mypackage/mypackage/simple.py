# Simple.py - a sim[ple python example
#  Refer to the Python3 for Robotics course ( https://app.theconstruct.ai/courses/58 )

import rclpy
# Import the Node modile fomr the ROS2 python library
from rclpy.node import Node

def main(args=None):
    # Initialize the ROS2 communications (Python client library)
    rclpy.init(args=args)

    print( "Help me Obi-Wan Kenobi! You are my only hope.")
    # Create a Node instance
#    node = Node('simple_node')

    # Log a message to indicate that the node has started
 #   node.get_logger().info('Simple node has been started.')

    # Spin the node to keep it active
#    rclpy.spin(node)

    # Shutdown the ROS2 Python client library
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    print("Simple node has been executed successfully.")
# This script initializes a simple ROS2 node that logs a message when started.
# It uses the rclpy library to create and manage the node lifecycle.
# The node is named 'simple_node', and it logs an informational message to the console.
