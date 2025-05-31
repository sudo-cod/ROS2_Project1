#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleTurtle(Node):
    def __init__(self):
        super().__init__('circle_turtle')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.move_turtle)
        self.get_logger().info('Turtle will now move in a circle!')

    def move_turtle(self):
        msg = Twist()
        msg.linear.x = 1.0    # forward speed
        msg.angular.z = 1.0   # rotate to form a circle
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CircleTurtle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
