#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import String
import message_filters
import kitti_ros2.utils_stereo as utils_stereo
from kitti_ros2.utils_publish import *
from kitti_ros2.utils_detect import *
from interface.msg import YoloLabel, StereoLabel

QUEUE_SZ = 5


class StereoNode(Node):
    def __init__(self, name):
        super().__init__(name)

        self.frame_sub = self.create_subscription(
            String, 'kitti_frame', self.frame_listener, QUEUE_SZ)
        self.cv_bridge = CvBridge()

        self.detect_sub = self.create_subscription(
            YoloLabel, 'detect_labels', self.detect_listener, QUEUE_SZ)
        self.yololabels = []

        # map depth_array
        self.mfsub_imgL = message_filters.Subscriber(
            self, Image, '/kitti_imgraw_L/gray')
        self.mfsub_imgR = message_filters.Subscriber(
            self, Image, '/kitti_imgraw_R/gray')
        # self.mfsub_yololabels = message_filters.Subscriber(
        #     self, YoloLabel, 'detect_labels')
        self.ts_threeD = message_filters.TimeSynchronizer(
            [self.mfsub_imgL, self.mfsub_imgR], queue_size=QUEUE_SZ)
        self.ts_threeD.registerCallback(self.threeD_callback)
        self.stereolabels = StereoLabel()

        self.disparity_pub = self.create_publisher(
            Image, 'kitti_disparity', QUEUE_SZ)
        self.deptharr = []
        self.disparity_map = []

        self.imgDetecteDistance_pub = self.create_publisher(
            Image, 'imgL_detect_distance', QUEUE_SZ)

    def frame_listener(self, frame_msg):
        self.get_logger().info("Receiving frame: [#%s]" % frame_msg.data)

    def detect_listener(self, yololabels_msg):
        i = 0
        self.yololabels = []
        for i in range(len(yololabels_msg.labels_cls)):
            yololabel = [0] * 6
            yololabel[0] = yololabels_msg.labels_x[i]
            yololabel[1] = yololabels_msg.labels_y[i]
            yololabel[2] = yololabels_msg.labels_w[i]
            yololabel[3] = yololabels_msg.labels_h[i]
            yololabel[4] = yololabels_msg.labels_conf[i]
            yololabel[5] = yololabels_msg.labels_cls[i]
            self.yololabels.append(yololabel)
        # self.get_logger().info(self.yololabels)

    def threeD_callback(self, imgmsgL, imgmsgR):
        cvImageL = self.cv_bridge.imgmsg_to_cv2(imgmsgL, "bgr8")
        cvImageR = self.cv_bridge.imgmsg_to_cv2(imgmsgR, "bgr8")

        deptharr, disparity_map = utils_stereo.distance_calc(
            cvImageL, cvImageR)
        self.deptharr = deptharr
        self.disparity_map = disparity_map
        publish_disparity(self.disparity_pub,
                          self.cv_bridge, self.disparity_map)

        img_objdetect_dist = drawLabel_detect_and_distance(
            cvImageL, self.yololabels, self.deptharr)
        self.yololabels = []
        publish_img(self.imgDetecteDistance_pub,
                    self.cv_bridge,
                    img_objdetect_dist)


def main(args=None):
    rclpy.init(args=args)
    node = StereoNode("stereo_node")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
