import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSaverNode(Node):
    def __init__(self, save_directory, filename_format, frequency):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',  # Replace with your image topic
            self.save_image,
            10
        )
        self.save_directory = save_directory
        self.filename_format = filename_format
        self.frequency = frequency
        self.image_count = 0
        self.previous_time = time.time()
        self.cv_bridge = CvBridge()

    def save_image(self, msg):
        current_time = time.time()
        if current_time - self.previous_time >= 1.0 / self.frequency:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            filename = os.path.join(self.save_directory, self.filename_format % self.image_count)
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f'Saved image: {filename}')
            self.image_count += 1
            self.previous_time = current_time

def main(args=None):
    rclpy.init(args=args)

    save_directory = '/home/robotino/ros2_honies_ws/data'  # Replace with your desired save directory
    filename_format = 'image%04d.jpg'  # Replace with your desired filename format
    frequency = 0.33  # Frequency in Hz (e.g., 0.33 for every 3 seconds)

    node = ImageSaverNode(save_directory, filename_format, frequency)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
