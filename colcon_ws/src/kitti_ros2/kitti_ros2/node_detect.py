#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import numpy as np
import torch.nn as nn
import torch

from yolov5 import detect_ros
from kitti_ros2.utils_publish import *
from kitti_ros2.utils_detect import *
from interface.msg import YoloLabel, StereoLabel

QUEUE_SZ = 1
RATE = 1.0/10
DETECT_TOPIC = '/kitti_imgraw_L/RGB'


class DetectNode(Node):
    def __init__(self, name):
        super().__init__(name)

        self.frame_sub = self.create_subscription(
            String, 'kitti_frame', self.frame_listener, QUEUE_SZ)
        self.cv_bridge = CvBridge()

        # do object detection
        self.imgDetect_sub = self.create_subscription(
            Image, DETECT_TOPIC, self.image_listener, QUEUE_SZ)

        self.detector = detect_ros.Yolov5Detector()
        self.labels = []
        self.yoloLabel_pub = self.create_publisher(
            YoloLabel, 'detect_labels', QUEUE_SZ)
        self.imgDetect_pub = self.create_publisher(
            Image, DETECT_TOPIC+'_detect', QUEUE_SZ)

    def frame_listener(self, frame_msg):
        self.get_logger().info("Receiving frame: [#%s]" % frame_msg.data)

    def yolo_detect(self, cvImage):
        self.current_image_frame = cvImage
        image_np = np.swapaxes(self.current_image_frame, 0, 2)  # [C,W,H]
        image_np = np.swapaxes(image_np, 1, 2)  # [C,H,W]
        image_tensor = torch.from_numpy(image_np).type(
            torch.float32)  # np.Array -> torch.Tensor
        image_tensor = paddingImage(image_tensor)
        image_np = image_tensor.numpy()  # torch.Tensor -> np.Array

        cvImgRet, detect, labels = self.detector.detectImage(
            image_np, self.current_image_frame, needProcess=True)  # cvImgRet is ready to plot
        return cvImgRet, detect, labels

    def image_listener(self, imgmsg):
        self.get_logger().info("detecting... ")
        cvImage = self.cv_bridge.imgmsg_to_cv2(imgmsg, 'bgr8')
        cvImgRet, detect, labels = self.yolo_detect(cvImage)
        publish_img(self.imgDetect_pub, self.cv_bridge, cvImgRet)

        self.labels = detect
        # self.get_logger().info(self.labels)
        # label syntax:
        # array[
        #  [x, y, w, h, conf, cls],
        #  [x, y, w, h, conf, cls],
        #  [x, y, w, h, conf, cls],
        #  ...
        # ]
        self.yoloLabel = YoloLabel()
        for label in self.labels:
            # self.get_logger().info(label)
            self.yoloLabel.labels_x.append(int(label[0]))
            self.yoloLabel.labels_y.append(int(label[1]))
            self.yoloLabel.labels_w.append(int(label[2]))
            self.yoloLabel.labels_h.append(int(label[3]))
            self.yoloLabel.labels_conf.append(label[4])
            self.yoloLabel.labels_cls.append(int(label[5]))

        self.yoloLabel_pub.publish(self.yoloLabel)


def main(args=None):
    rclpy.init(args=args)
    node = DetectNode("detect_node")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
