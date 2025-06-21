## Simple ROS2 Publisher in Python


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
#from std_msgs.msg import String

## Publisher node's class
class SimplePublisher(Node):

    dx = 1
    val=0

    def __init__(self):
        ## Call te base class with the node's name "simplePublisher"
        super().__init__('simplePublisher')
        ## Publisher initialization to use the cmd_vel topic
        ## The topic type is TwistStamped, and the queue size is set to 10
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        ## Create a timer for the callback function to be called every 0.5 seconds
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info('Simple Publisher Node has been initialized.')

    def timer_callback(self):
        if self.val > 99.0:
            self.dx = -1
        elif self.dx < -99.0:
            self.dx = 1
        self.val += self.dx * 0.5
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = self.val
        self.publisher_.publish( msg )
        self.get_logger().info('Publishing: Linear X: %f, Angular Z: %f' % (msg.twist.linear.x, msg.twist.angular.z))


## Main publisher function
def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    node.get_logger().info('Simple Publisher Node has been started.')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    print("Simple Publisher Node has been executed successfully.")
# This script initializes a simple ROS2 publisher node that publishes TwistStamped messages.
# It uses the rclpy library to create and manage the node lifecycle.
# The node is named 'simple_publisher', and it publishes messages to the 'cmd_vel' topic.
# The messages contain linear and angular velocity data, which are logged to the console.
# The node runs a timer that publishes messages at a regular interval (0.5 seconds).
# The script is designed to be run as a standalone Python program.