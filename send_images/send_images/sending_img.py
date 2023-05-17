import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import os
import os.path as op
from glob import glob
import time
import numpy as np

from ros_imgtime_msg.msg import Imagetime
from detecting_result_msg.msg import Result


class ImageThrower(Node):
    def __init__(self, image_path):
        super().__init__("image_processor")
        self.image_list = self.load_images(image_path)
        self.image_len = len(self.image_list)
        self.count = 0
        self.image_publisher = self.create_publisher(Imagetime, "sending_image", 10)
        time_period = 10
        for image in self.image_list:
            self.create_timer(time_period, self.time_callback)

    def load_images(self, image_path):
        image_list = glob(op.join(image_path, "*.png"))
        image_list.sort()
        return image_list

    def time_callback(self):
        img_time_msg = Imagetime()
        image = cv2.imread(self.image_list[self.count])
        img_msg = CvBridge().cv2_to_imgmsg(image, "bgr8")
        img_time_msg.image = img_msg
        sending_time = time.time()
        img_time_msg.timestamp = [sending_time]
        self.image_publisher.publish(img_time_msg)
        cv2.imshow("publish image", image)
        cv2.waitKey(2)
        self.count += 1
        if self.count > self.image_len-1:
            self.count = 0


def main(args=None):
    image_path = "/home/ri/lee_ws/kitti_sample"
    rclpy.init(args=args)
    node = ImageThrower(image_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
