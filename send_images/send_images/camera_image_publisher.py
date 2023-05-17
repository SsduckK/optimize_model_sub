import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import os
import os.path as op
from glob import glob


class CameraImagePublisher(Node):
    def __init__(self):
        super().__init__("camera_pub")
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, "camera_image", 10)
        time_period = 0.01
        self.timer = self.create_timer(time_period, self.time_callback)
        # self.vid_cap = cv2.VideoCapture("http://192.168.0.104:4747/video")
        self.vid_cap = cv2.VideoCapture(0)

    def time_callback(self):
        ret, frame = self.vid_cap.read()
        if ret:
            image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher.publish(image_msg)
            cv2.imshow("publish_frame", frame)
            cv2.waitKey(2)
            self.get_logger().info("publishing Camera Image")


def main(args=None):
    rclpy.init(args=args)
    node = CameraImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("stop")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()