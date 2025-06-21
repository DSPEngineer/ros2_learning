#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan

class Patrol(Node):

    def __init__(self):
        super().__init__('patrol_node')

        # Create a subscription to laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10)

        # Create a publisher for publishing velocity commands
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/rosbot_xl_base_controller/cmd_vel',
            5)

        # Create a timer for controlling the robot's movement
        self.control_timer = self.create_timer(
            0.5,  # 500 ms
            self.motion)

        # Initialize the velocity command message
        self.cmd = TwistStamped()
        self.cmd.twist.linear.x = 0.0
        self.cmd.twist.angular.z = 0.0

        # Constants for distance thresholds
        self.min_distance = 0.55
        self.side_threshold = 0.15

        # Initialize variables for laser scan data
        self.left_side = 0.0
        self.front = 0.0
        self.right_side = 0.0


    def laser_callback(self, msg: LaserScan):
        # Process laser scan data and extract  values for different sections
        # of the robot's surroundings
        self.right_side = msg.ranges[2246]
        self.front = msg.ranges[0]
        self.left_side = msg.ranges[748]
        self.get_logger().info('I receive: "%s"' % str(self.front)) 


    def motion(self):
        # Control the robot's movement based on laser scan data

        # Define linear velocity
        linear_vel = 0.5

        # Determine robot's movement based on obstacle detection
        if self.left_side < self.side_threshold and self.front > self.min_distance:
            self.get_logger().warning('Reduce speed and turn!!')
            self.cmd.twist.linear.x = linear_vel * 0.5
            self.cmd.twist.angular.z = -0.15
        elif self.right_side < self.side_threshold and self.front > self.min_distance:
            self.get_logger().warning('Reduce speed and turn!!')
            self.cmd.twist.linear.x = linear_vel * 0.5
            self.cmd.twist.angular.z = 0.15
        elif self.front > self.min_distance:
            self.get_logger().info('Moving forward!!')
            self.cmd.twist.linear.x = linear_vel
            self.cmd.twist.angular.z = 0.0
        elif self.front < self.min_distance:
            self.get_logger().error('Stop and rotate!!')
            if self.compare_sides(self.left_side, self.right_side):
                self.cmd.twist.linear.x = linear_vel * 0.25
                self.cmd.twist.angular.z = 0.35
            else:
                self.cmd.twist.linear.x = linear_vel * 0.25
                self.cmd.twist.angular.z = -0.35

        # Publish velocity command
        self.cmd_pub.publish(self.cmd)

    def compare_sides(self, left, right):
        # Compare distances sensed on left and right sides
        return left >= right 


def main(args=None):
    # Initialize ROS2 node
    rclpy.init(args=args)

    # Create an instance of the Patrol class
    patrol_node = Patrol()

    try:
        # Spin the node to handle callbacks
        rclpy.spin(patrol_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown ROS2 node
        patrol_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()