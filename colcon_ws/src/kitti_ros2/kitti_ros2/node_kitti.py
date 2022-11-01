#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String

from kitti_ros2.utils_data import *
from kitti_ros2.utils_publish import *

QUEUE_SZ = 1
FPS = 10.0

DATAID = '0005'  # string of 4 digits
MAXFRAME_MATCH = {'0009': 447,
                  '0005': 154,
                  '0013': 144,
                  '0002': 77,
                  '0057': 361}

ROOT = '/home/jun/ros_kitti'
DATA_PATH = ROOT+'/rawdata/2011_09_26/2011_09_26_drive_'+DATAID+'_sync/'

IMG_CHN = {'l_gray': 'image_00',
           'r_gray': 'image_01',
           'l_RGB': 'image_02',
           'r_RGB': 'image_03'}


class KittiNode(Node):
    def __init__(self, name):
        super().__init__(name)

        # 创建一个QoS原则
        qos_profile = QoSProfile(
            # reliability=QoSReliabilityPolicy.BEST_EFFORT,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=QUEUE_SZ
        )

        self.frame = 0
        self.frame_pub = self.create_publisher(String, 'kitti_frame', QUEUE_SZ)

        self.cv_bridge = CvBridge()
        # publisher image_raw_RGB
        self.img_pubL = self.create_publisher(
            Image, '/kitti_imgraw_L/RGB', QUEUE_SZ)
        self.img_pubR = self.create_publisher(
            Image, '/kitti_imgraw_R/RGB', QUEUE_SZ)
        # publisher image_raw_gray
        self.img_pubL0 = self.create_publisher(
            Image, '/kitti_imgraw_L/gray', QUEUE_SZ)
        self.img_pubR0 = self.create_publisher(
            Image, '/kitti_imgraw_R/gray', QUEUE_SZ)

        # publisher point cloud
        # self.pcl_pub = self.create_publisher(
        #     PointCloud2, 'kitti_pcl', QUEUE_SZ)
        self.pcd_pub = self.create_publisher(
            PointCloud2, 'kitti_pcd', QUEUE_SZ)

        self.timer = self.create_timer(
            timer_period_sec=1/FPS, callback=self.timer_callback)

    def timer_callback(self):
        # image_raw_RGB
        img_L = read_imgraw(DATA_PATH, self.frame, IMG_CHN['l_RGB'])
        publish_img(self.img_pubL, self.cv_bridge, img_L)
        img_R = read_imgraw(DATA_PATH, self.frame, IMG_CHN['l_RGB'])
        publish_img(self.img_pubR, self.cv_bridge, img_R)

        # image_raw_gray
        img_L0 = read_imgraw(DATA_PATH, self.frame, IMG_CHN['l_gray'])
        publish_img(self.img_pubL0, self.cv_bridge, img_L0)
        img_R0 = read_imgraw(DATA_PATH, self.frame, IMG_CHN['r_gray'])
        publish_img(self.img_pubR0, self.cv_bridge, img_R0)

        # point cloud
        # pcl = read_pcl(DATA_PATH, self.frame)
        # publish_pcl(self.pcl_pub, pcl)

        pcd = read_pcd(DATA_PATH, self.frame)
        publish_pcd(self.pcd_pub, pcd)

        # frame_id
        publish_frame(self.frame_pub, self.frame)
        self.get_logger().info("Publishing frame: [#%010d]" % self.frame)
        self.frame += 1
        if self.frame >= MAXFRAME_MATCH[DATAID]:
            self.frame = 0


def main(args=None):
    rclpy.init(args=args)
    node = KittiNode("kitti_node")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
