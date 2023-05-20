import os
import os.path as op
import numpy as np
import cv2
import time


class TimeLogger:
    def __call__(self, timestamps, path):
        if not op.exists(path):
            os.mkdir(path)




class VisualLogger:
    def __call__(self, image, bboxes, classes, scores, path):
        if not op.exists(path):
            os.mkdir(path)
        drawn_image = self.draw_annotation(image, bboxes, classes, scores)
        cv2.imshow("image", drawn_image)
        cv2.waitKey(10)
        self.save_image(path, drawn_image)

    def draw_annotation(self, image, bboxes, classes, scores):
        src_img = image.copy()
        for box, cls, score in zip(bboxes, classes, scores):
            cls_score = str(cls) + "/" + str(score)
            dst_img = cv2.rectangle(src_img, (box[0], box[1]), (box[0]+box[2], box[1]+box[3]), (0, 255, 0), 2)
            dst_img = cv2.putText(dst_img, cls_score, (box[0], box[1] + 10), cv2.FONT_ITALIC, 0.35, (0, 255, 0))
        return dst_img

    def save_image(self, path, image):
        cv2.imwrite(path+str(time.time()) + ".png", image)


def time_test():
    pass


def visual_test():
    vis_log = VisualLogger()
    result_path = "/home/ri/lee_ws/ros/src/optimize_model_sub/send_images/result/vis/"
    sample_image = cv2.imread("/home/ri/lee_ws/kitti_sample/000000.png")
    bboxes = [[100, 200, 50, 100], [300, 400, 100, 200]]
    classes = [3, 4]
    scores = [0.8, 0.1]
    vis_log(sample_image, bboxes, classes, scores, result_path)


if __name__ == "__main__":
    visual_test()
