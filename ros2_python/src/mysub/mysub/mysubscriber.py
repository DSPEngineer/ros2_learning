## Simple Subscriber for ROS2 learning, in python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
# from std_msgs.msg import String

## Simple Subscriber Node Class
class SimpleSubscriber(Node):
    ## This class defines a simple ROS2 subscriber node that listens to TwistStamped messages.
    def __init__(self):
        super().__init__('simpleSubscriber')
        self.subscription = self.create_subscription(
            TwistStamped,
            '/cmd_vel',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Simple Subscriber Node has been initialized.')

    ## Callback function that is called when a message is received
    def listener_callback(self, msg):
        self.get_logger().info(
            'Received message: Linear X: %f, Angular Z: %f' %
            (msg.twist.linear.x, msg.twist.angular.z)
        )

## Main subscriber function
def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    node.get_logger().info('Simple Subscriber Node has been started.')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    print("Simple Subscriber Node has been executed successfully.")
# This script initializes a simple ROS2 subscriber node that listens to TwistStamped messages.
# It uses the rclpy library to create and manage the node lifecycle.
