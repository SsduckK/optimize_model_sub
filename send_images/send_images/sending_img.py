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
from .recorder import TimeLogger as TL
from .recorder import VisualLogger as VL


class ImageProcessor(Node):
    def __init__(self, image_path, result_path):
        super().__init__("image_processor")
        self.result_path = result_path
        self.time_logger = TL(self.result_path)
        self.visual_logger = VL()
        self.image_list = self.load_images(image_path)
        self.image_len = len(self.image_list)
        self.image_with_timestamp = {}
        self.count = 0
        self.image_publisher = self.create_publisher(Imagetime, "sending_image", 10)
        self.result_subscriber = self.create_subscription(Result, "sending_result", self.subscribe_result, 10)
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
        self.image_with_timestamp[sending_time] = image
        self.count += 1
        if self.count > self.image_len-1:
            self.count = 0

    def subscribe_result(self, result):
        receive_time = time.time()
        total_time = list(result.timestamp)
        total_time.extend([receive_time])
        matched_image = self.get_matched_image(total_time[0])
        bboxes_bytes = result.bboxes
        classes_bytes = result.classes
        scores_bytes = result.scores
        bboxes = np.frombuffer(bytes(list(bboxes_bytes)), np.float32)
        bboxes = bboxes.reshape(-1, 4)
        classes = np.frombuffer(bytes(list(classes_bytes)), np.int64)
        scores = np.frombuffer(bytes(list(scores_bytes)), np.float32)
        self.visual_logger(matched_image, bboxes, classes, scores, self.result_path+"vis/")
        self.time_logger(total_time)

    def get_matched_image(self, timestamp):
        if timestamp in self.image_with_timestamp.keys():
            matched_image = self.image_with_timestamp[timestamp]
            self.image_with_timestamp.pop(timestamp)
            return matched_image


def main(args=None):
    image_path = "/home/ri/lee_ws/kitti_sample"
    result_path = "/home/ri/lee_ws/ros/src/optimize_model_sub/send_images/result/"
    rclpy.init(args=args)
    node = ImageProcessor(image_path, result_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.time_logger.saving_data()
        node.get_logger().info('Keyboard Interrupt (SIGINT')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
