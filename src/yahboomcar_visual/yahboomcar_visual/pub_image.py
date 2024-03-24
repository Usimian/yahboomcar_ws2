"""pub_image."""

import cv2 as cv

from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image


class PubImage(Node):
    """PubImage class."""

    def __init__(self, name):
        """Init."""
        super().__init__(name)
        self.bridge = CvBridge()
        self.sub_img = self.create_subscription(Image, "/camera/color/image_raw", self.handleTopic, 500)
        self.pub_img = self.create_publisher(Image, "/image", 500)

    def handleTopic(self, msg):
        """handleTopic."""
        if not isinstance(msg, Image):
            return
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Standardize the input image size
        frame = cv.resize(frame, (640, 480))
        # opencv mat ->  ros msg
        msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.pub_img.publish(msg)


def main():
    """Entrypoint."""
    rclpy.init()
    pub_image = PubImage("pub_image_node")
    rclpy.spin(pub_image)
