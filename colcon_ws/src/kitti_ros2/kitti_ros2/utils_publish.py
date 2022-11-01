#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

import std_msgs.msg as std_msgs
import sensor_msgs.msg as seneor_msgs
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import kitti_ros2.utils_stereo as utils_stereo
import cv2

import numpy as np

FRAME_ID = "map"
QUEUE_SZ = 20
FPS = 10
LIFETIME = 1.0/FPS

DETECTION_COLOR_MAP = {
    'Car': (255, 255, 0),
    'Pedestrian': (0, 226, 255),
    'Cyclist': (141, 40, 255)
}  # color for detection, in format bgr

BBOX_EGO = np.array([[2.15, 0.9, -1.73], [2.15, -0.9, -1.73], [-1.95, -0.9, -1.73], [-1.95, 0.9, -1.73],
                     [2.15, 0.9, -0.23], [2.15, -0.9, -0.23], [-1.95, -0.9, -0.23], [-1.95, 0.9, -0.23]])

# corners of the box: front surface 0-1-5-4-0
#     6 -------- 7
#    /|         /|
#   5 -------- 4 .
#   | |        | |
#   . 2 -------- 3
#   |/         |/
#   1 -------- 0
LINES = [[0, 1], [1, 2], [2, 3], [3, 0]]  # lower surface
LINES += [[4, 5], [5, 6], [6, 7], [7, 4]]  # upper surface
LINES += [[4, 0], [5, 1], [6, 2], [7, 3]]  # connect lower and upper surface
LINES += [[4, 1], [5, 0]]                  # cross the front surface


def publish_frame(frame_pub, frame_id):
    frame_msg = std_msgs.String()
    frame_msg.data = "%010d" % frame_id
    frame_pub.publish(frame_msg)


def publish_img(cam_pub, bridge, cvimg):
    imgmsg = bridge.cv2_to_imgmsg(cvimg, 'bgr8')
    imgmsg.header.frame_id = FRAME_ID
    cam_pub.publish(imgmsg)


def publish_pcd(pcd_pub, pcd):
    def create_pcdmsg(pcd, parent_frame):
        """ Creates a point cloud message.
        Args:
            pcd: pcd data of point cloud
            parent_frame: frame in which the point cloud is defined
        Returns:
            sensor_msgs/PointCloud2 message
        Code source:
            https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0
        References:
            http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html
            http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointField.html
            http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html
        """
        ros_dtype = seneor_msgs.PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize

        points = np.asarray(pcd.points)  # points: Nx3 array of xyz positions.
        data = points.astype(dtype).tobytes()

        fields = [seneor_msgs.PointField(
            name=n, offset=i*itemsize, datatype=ros_dtype, count=1) for i, n in enumerate('xyz')]

        header = std_msgs.Header(frame_id=parent_frame)
        pcd_msg = seneor_msgs.PointCloud2(
            header=header,
            height=1,
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize*3),
            row_step=(itemsize*3*points.shape[0]),
            data=data)
        return pcd_msg

    pcd_msg = create_pcdmsg(pcd, FRAME_ID)
    pcd_pub.publish(pcd_msg)


def publish_pcl(pcl_pub, point_cloud, frame_id=FRAME_ID):
    def create_pclmsg(points, parent_frame):
        """ Creates a point cloud message.
        Args:
            points: Nx3 array of xyz positions.
            parent_frame: frame in which the point cloud is defined
        Returns:
            sensor_msgs/PointCloud2 message
        Code source:
            https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0
        References:
            http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html
            http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointField.html
            http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html
        """
        # In a PointCloud2 message, the point cloud is stored as an byte
        # array. In order to unpack it, we also include some parameters
        # which desribes the size of each individual point.

        ros_dtype = seneor_msgs.PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize  # A 32-bit float takes 4 bytes.

        data = points.astype(dtype).tobytes()

        # The fields specify what the bytes represents. The first 4 bytes
        # represents the x-coordinate, the next 4 the y-coordinate, etc.
        fields = [seneor_msgs.PointField(
            name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate('xyz')]

        # The PointCloud2 message also has a header which specifies which
        # coordinate frame it is represented in.
        header = std_msgs.Header(frame_id=parent_frame)

        return seneor_msgs.PointCloud2(
            header=header,
            height=1,
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            # Every point consists of three float32s.
            point_step=(itemsize * 3),
            row_step=(itemsize * 3 * points.shape[0]),
            data=data
        )

    pcd = create_pclmsg(point_cloud[:, :3], frame_id)
    pcl_pub.publish(pcd)


def publish_disparity(disparity_pub, bridge, imgL0, imgR0):
    disparity = utils_stereo.STEREO.compute(imgL0, imgR0)
    # 归一化函数算法，生成深度图（灰度图）
    disp_grey = cv2.normalize(disparity, disparity, alpha=0,
                              beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

    # 生成深度图（颜色图）
    dis_color = disparity
    dis_color = cv2.normalize(
        dis_color, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    dis_color = cv2.applyColorMap(dis_color, 2)

    # disp_array = np.array(dis_color, dtype=np.float32)
    # depth_array = 980*54/disp_array  # Z = f*B/d

    disparity_pub.publish(bridge.cv2_to_imgmsg(dis_color, "8UC3"))


# def publish_disp3d(disp3d_pub, bridge, disp3d):
#     disp3d_pub.publish(bridge.cv2_to_imgmsg(disp3d, "32FC1"))


def publish_ego_fov(egocar_pub, bold=0.08):
    pass
