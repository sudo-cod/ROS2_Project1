#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool                        # use Bool message type
from cv_bridge import CvBridge
import cv2
import numpy as np

# HSV color range for red
lower_red = np.array([0, 90, 128])
upper_red = np.array([180, 255, 255])

class ImageSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)
        self.sub = self.create_subscription(
            Image, 'image_raw', self.listener_callback, 10)
        self.pub = self.create_publisher(
            Bool, 'apple_detected', 10)              # publishing Bool
        self.cv_bridge = CvBridge()

    def object_detect(self, image):
        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask_red = cv2.inRange(hsv_img, lower_red, upper_red)
        contours, _ = cv2.findContours(
            mask_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        
        for cnt in contours:
            if cnt.shape[0] < 150:
                continue

            (x, y, w, h) = cv2.boundingRect(cnt)
            cv2.drawContours(image, [cnt], -1, (0, 255, 0), 2)
            cv2.circle(image, (int(x+w/2), int(y+h/2)), 5, (0, 255, 0), -1)

            return True  # apple detected

        return False  # no apple found

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')

        apple_found = self.object_detect(image)

        msg = Bool()
        msg.data = apple_found
        self.pub.publish(msg)

        cv2.imshow("object", image)
        cv2.waitKey(50)

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber("apple_detector")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
