#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class AppleControlledTurtle(Node):
    def __init__(self):
        super().__init__('apple_controlled_turtle')
        self.apple_detected = False

        # Subscriber to /apple_detected
        self.sub = self.create_subscription(
            Bool,
            'apple_detected',
            self.apple_callback,
            10
        )

        # Publisher to /turtle1/cmd_vel
        self.pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        # Timer to send velocity commands at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_cmd_vel)

        self.get_logger().info('Apple-controlled turtle is running.')

    def apple_callback(self, msg):
        self.apple_detected = msg.data
        if self.apple_detected:
            self.get_logger().info('Apple detected – Turtle moving.')
        else:
            self.get_logger().info('No apple – Turtle stopping.')

    def publish_cmd_vel(self):
        twist = Twist()
        if self.apple_detected:
            twist.linear.x = 1.0
            twist.angular.z = 1.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = AppleControlledTurtle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
