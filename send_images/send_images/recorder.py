import os
import os.path as op
import numpy as np
import cv2
import time
import csv
import datetime
import json


class TimeLogger:
    def __init__(self, path):
        self.path = path
        if not op.exists(self.path):
            os.mkdir(self.path)
        self.frames = 1
        self.time_logs = []
        self.entry = self.load_entry()
        self.time_logs.append(self.entry)

    def __call__(self, timestamp):
        record_data = self.setting_database(self.frames, timestamp)
        self.time_logs.append(record_data)
        self.frames += 1

    def load_entry(self):
        entry = ["frames", "sending", "before model", "after model", "receiving", "|",
                 "send_bmodel", "bmodel_amodel", "amodel_receive", "summary"]
        return entry

    def setting_database(self, frame, timestamp):
        send_bmodel = timestamp[1] - timestamp[0]
        bmodel_amodel = timestamp[2] - timestamp[1]
        amodel_receive = timestamp[3] - timestamp[2]
        summary = send_bmodel + bmodel_amodel + amodel_receive
        timestamp.insert(0, frame)
        timestamp.extend(['|', send_bmodel, bmodel_amodel, amodel_receive, summary])
        return timestamp

    def saving_data(self):
        with open(self.path + str(datetime.date.today()) + ".csv", 'w') as f:
            writer = csv.writer(f)
            for frame in self.time_logs:
                writer.writerow(frame)
        print("saved data")


class VisualLogger:
    def __call__(self, image, bboxes, classes, scores, path):
        if not op.exists(path):
            os.mkdir(path)
        drawn_image = self.draw_annotation(image, bboxes, classes, scores)
        self.save_image(path, drawn_image)

    def draw_annotation(self, image, bboxes, classes, scores):
        src_img = image.copy()
        for box, cls, score in zip(bboxes, classes, scores):
            score = round(score, 3)
            converted_cls = self.convert_class(cls)
            cls_score = str(cls) + "/" + str(score)
            dst_img = cv2.rectangle(src_img, (int(box[0]), int(box[1])),
                                    (int(box[2]), int(box[3])), (0, 255, 0), 2)
            dst_img = cv2.putText(dst_img, cls_score, (int(box[0]), int(box[1]) + 10), cv2.FONT_ITALIC, 0.35, (0, 255, 0))
        return dst_img

    def convert_class(self, cls):
        with open("coco_categories.json") as f:
            file = json.load(f)
            index = cls+1
            return file[index]["supercategory"]


    def save_image(self, path, image):
        cv2.imwrite(path+str(time.time()) + ".png", image)


def time_test():
    result_path = "/home/ri/lee_ws/ros/src/optimize_model_sub/send_images/result/timestamp/"
    time_log = TimeLogger(result_path)
    first_frame = [100, 200, 300, 400]
    second_frame = [200, 300, 400, 500]
    third_frame = [300, 400, 500, 600]
    time_log(first_frame)
    time_log(second_frame)
    time_log(third_frame)
    time_log.saving_data()


def visual_test():
    vis_log = VisualLogger()
    result_path = "/home/ri/lee_ws/ros/src/optimize_model_sub/send_images/result/vis/"
    sample_image = cv2.imread("/home/ri/lee_ws/kitti_sample/000000.png")
    bboxes = [[100, 200, 50, 100], [300, 400, 100, 200]]
    classes = [3, 4]
    scores = [0.8, 0.1]
    vis_log(sample_image, bboxes, classes, scores, result_path)


if __name__ == "__main__":
    # visual_test()
    time_test()
