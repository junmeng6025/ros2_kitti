#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import message_filters


from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import numpy as np
import torch.nn as nn
import torch

from yolov5 import detect_ros
from interface.msg import YoloLabel, StereoLabel

from kitti_ros2.utils_publish import *
from kitti_ros2.utils_detect import *
import kitti_ros2.utils_stereo as utils_stereo

QUEUE_SZ = 5
RATE = 1.0/10
DETECT_TOPIC = '/kitti_imgraw_L/RGB'


class StereoDetectNode(Node):
    def __init__(self, name):
        super().__init__(name)

        self.frame_sub = self.create_subscription(
            String, 'kitti_frame', self.frame_listener, QUEUE_SZ)
        self.cv_bridge = CvBridge()

        # map depth_array
        self.mfsub_imgL = message_filters.Subscriber(
            self, Image, '/kitti_imgraw_L/gray')
        self.mfsub_imgR = message_filters.Subscriber(
            self, Image, '/kitti_imgraw_R/gray')
        self.ts_threeD = message_filters.TimeSynchronizer(
            [self.mfsub_imgL, self.mfsub_imgR], queue_size=QUEUE_SZ)
        self.ts_threeD.registerCallback(self.threeD_callback)

        self.disparity_pub = self.create_publisher(
            Image, 'kitti_disparity', QUEUE_SZ)
        self.deptharr = []
        self.disparity_map = []

        # do object detection
        self.imgDetect_sub = self.create_subscription(
            Image, DETECT_TOPIC, self.image_listener, QUEUE_SZ)

        self.detector = detect_ros.Yolov5Detector()
        self.labels = []
        self.imgDetect_pub = self.create_publisher(
            Image, DETECT_TOPIC+'_detect', QUEUE_SZ)
        self.imgDetecteDistance_pub = self.create_publisher(
            Image, DETECT_TOPIC+'_detect_distance', QUEUE_SZ)

    def frame_listener(self, frame_msg):
        self.get_logger().info("Receiving frame: [#%s]" % frame_msg.data)

    def threeD_callback(self, imgmsgL, imgmsgR):
        cvImageL = self.cv_bridge.imgmsg_to_cv2(imgmsgL, "bgr8")
        cvImageR = self.cv_bridge.imgmsg_to_cv2(imgmsgR, "bgr8")
        deptharr, disparity_map = utils_stereo.distance_calc(
            cvImageL, cvImageR)
        self.deptharr = deptharr
        self.disparity_map = disparity_map
        publish_disparity(self.disparity_pub,
                          self.cv_bridge, self.disparity_map)

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
        # # label syntax:
        # # [
        # #  [x, y, w, h, conf, cls],
        # #  [x, y, w, h, conf, cls],
        # #  [x, y, w, h, conf, cls],
        # #  ...
        # # ]

        # self.yoloLabel.x = detect[:, 0]
        # self.yoloLabel.y = detect[:, 1]
        # self.yoloLabel.w = detect[:, 2]
        # self.yoloLabel.h = detect[:, 3]
        # self.yoloLabel.conf = detect[:, 4]
        # self.yoloLabel.cls = detect[:, 5]
        img_objdetect_dist = drawLabel_detect_and_distance(
            cvImage, self.labels, self.deptharr)
        publish_img(self.imgDetecteDistance_pub,
                    self.cv_bridge,
                    img_objdetect_dist)


def main(args=None):
    rclpy.init(args=args)
    node = StereoDetectNode("stereo_detect_node")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
