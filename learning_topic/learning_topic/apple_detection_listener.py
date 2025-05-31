#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class AppleDetectionSubscriber(Node):
    def __init__(self):
        super().__init__('apple_detection_subscriber')
        self.subscription = self.create_subscription(
            Bool,
            'apple_detected',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        if msg.data:
            self.get_logger().info('Apple detected!')
        else:
            self.get_logger().info('No apple detected.')

def main(args=None):
    rclpy.init(args=args)
    node = AppleDetectionSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
