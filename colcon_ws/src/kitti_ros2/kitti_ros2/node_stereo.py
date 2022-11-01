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
from interface.msg import YoloLabel, StereoLabel

QUEUE_SZ = 1


class StereoNode(Node):
    def __init__(self, name):
        super().__init__(name)

        # 创建一个QoS原则
        qos_profile = QoSProfile(
            # reliability=QoSReliabilityPolicy.BEST_EFFORT,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=QUEUE_SZ
        )

        # subscribe current frame_id
        self.frame_sub = self.create_subscription(
            String, 'kitti_frame', self.frame_listener, QUEUE_SZ)
        self.cv_bridge = CvBridge()

        # map depth_array
        self.mfsub_imgL = message_filters.Subscriber(
            self, Image, '/kitti_imgraw_L/RGB')
        self.mfsub_imgR = message_filters.Subscriber(
            self, Image, '/kitti_imgraw_R/RGB')
        self.ts_threeD = message_filters.TimeSynchronizer(
            [self.mfsub_imgL, self.mfsub_imgR], queue_size=QUEUE_SZ)
        self.ts_threeD.registerCallback(self.threeD_callback)

        # generate disparity map
        self.mfsub_imgL0 = message_filters.Subscriber(
            self, Image, '/kitti_imgraw_L/gray')
        self.mfsub_imgR0 = message_filters.Subscriber(
            self, Image, '/kitti_imgraw_R/gray')
        self.ts_disparity = message_filters.TimeSynchronizer(
            [self.mfsub_imgL0, self.mfsub_imgR0], queue_size=QUEUE_SZ)
        self.ts_disparity.registerCallback(self.disparity_callback)

        self.disparity_pub = self.create_publisher(
            Image, 'kitti_disparity', QUEUE_SZ)

    def frame_listener(self, frame_msg):
        self.get_logger().info("Receiving frame: [#%s]" % frame_msg.data)

    def threeD_callback(self, imgmsgL, imgmsgR):
        cvImageL = self.cv_bridge.imgmsg_to_cv2(imgmsgL, "bgr8")
        cvImageR = self.cv_bridge.imgmsg_to_cv2(imgmsgR, "bgr8")
        deptharr, disparity_map = utils_stereo.distance_calc(
            cvImageL, cvImageR)

    def disparity_callback(self, imgmsgL0, imgmsgR0):
        imgL0 = self.cv_bridge.imgmsg_to_cv2(imgmsgL0, 'mono8')
        imgR0 = self.cv_bridge.imgmsg_to_cv2(imgmsgR0, 'mono8')
        publish_disparity(self.disparity_pub, self.cv_bridge, imgL0, imgR0)


def main(args=None):
    rclpy.init(args=args)
    node = StereoNode("stereo_node")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
