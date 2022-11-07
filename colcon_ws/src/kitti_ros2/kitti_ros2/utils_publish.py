#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

import std_msgs.msg as std_msgs
import sensor_msgs.msg as seneor_msgs
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Time, Duration
from tf_transformations import quaternion_from_euler

from cv_bridge import CvBridge
import kitti_ros2.utils_stereo as utils_stereo
import cv2

import numpy as np

FRAME_ID = "map"
QUEUE_SZ = 10
FPS = 10
LIFETIME = 1.0/FPS

DETECTION_COLOR_MAP = {
    'Vehicle': (255, 255, 0),
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


# def publish_disparity_from_imagegray(disparity_pub, bridge, imgL0, imgR0):
#     disparity = utils_stereo.STEREO.compute(imgL0, imgR0)
#     # 归一化函数算法，生成深度图（灰度图）
#     disp_grey = cv2.normalize(disparity, disparity, alpha=0,
#                               beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

#     # 生成深度图（颜色图）
#     dis_color = disparity
#     dis_color = cv2.normalize(
#         dis_color, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
#     dis_color = cv2.applyColorMap(dis_color, 2)

#     # disp_array = np.array(dis_color, dtype=np.float32)
#     # depth_array = 980*54/disp_array  # Z = f*B/d

#     disparity_pub.publish(bridge.cv2_to_imgmsg(dis_color, "8UC3"))


def publish_disparity(disparity_pub, bridge, disparity_map):
    dis_color = disparity_map
    dis_color = cv2.normalize(
        dis_color, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    dis_color = cv2.applyColorMap(dis_color, 2)
    disparity_pub.publish(bridge.cv2_to_imgmsg(dis_color, "8UC3"))


def publish_egoFOV(egocar_pub, bold=0.1):

    # Define FOV lines
    marker = Marker()
    marker.header.frame_id = FRAME_ID
    marker.header.stamp = Time()
    marker.id = 0
    # marker.action = Marker.ADD
    marker.lifetime = Duration(sec=int(LIFETIME))
    marker.type = Marker.LINE_STRIP

    marker.color.r = 1.0
    marker.color.g = 0.5
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.scale.x = bold  # line width

    marker.points = []
    marker.points.append(Point(x=12.0, y=-12.0, z=0.0))  # left up
    marker.points.append(Point(x=0.0, y=0.0, z=0.0))  # center
    marker.points.append(Point(x=12.0, y=12.0, z=0.0))  # right up

    q = quaternion_from_euler(0, 0, 0)  # default quaternion
    marker.pose.orientation.x = q[0]
    marker.pose.orientation.y = q[1]
    marker.pose.orientation.z = q[2]
    marker.pose.orientation.w = q[3]

    # # add the FOV marker to the marker_array
    # marker_array.markers.append(marker)

    egocar_pub.publish(marker)


def publish_egoBBox(bbox3d_pub, corners_ego=BBOX_EGO):
    marker_array = MarkerArray()

    # marker [LINE_LIST]: 3d bbox
    marker = Marker()
    marker.header.frame_id = FRAME_ID
    marker.header.stamp = Time()

    marker.id = -1
    marker.action = Marker.ADD
    marker.lifetime = Duration()  # +0.1 improve the smoothy
    marker.type = Marker.LINE_LIST

    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0

    marker.color.a = 1.0
    marker.scale.x = 0.07  # line width

    marker.points = []
    for l in LINES:
        p1 = corners_ego[l[0]]
        marker.points.append(Point(x=p1[0], y=p1[1], z=p1[2]))
        p2 = corners_ego[l[1]]
        marker.points.append(Point(x=p2[0], y=p2[1], z=p2[2]))

    q = quaternion_from_euler(0, 0, 0)  # default quaternion
    marker.pose.orientation.x = q[0]
    marker.pose.orientation.y = q[1]
    marker.pose.orientation.z = q[2]
    marker.pose.orientation.w = q[3]

    marker_array.markers.append(marker)

    # text_marker [TEXT_VIEW_FACING]: text lable
    text_marker = Marker()
    text_marker.header.frame_id = FRAME_ID
    text_marker.header.stamp = Time()

    text_marker.id = -1 + 1000  # track_id
    text_marker.action = Marker.ADD
    text_marker.lifetime = Duration()  # +0.1 improve the smoothy
    text_marker.type = Marker.TEXT_VIEW_FACING

    # p4 = corners_bbox3d_velo[4]  # upper front left corner (z+0.5)
    p4 = np.mean(corners_ego, axis=0)  # box centre point (z+2)
    text_marker.pose.position.x = p4[0]
    text_marker.pose.position.y = p4[1]
    text_marker.pose.position.z = p4[2]+2

    text_marker.text = 'Ego'  # lable is the track_id

    text_marker.scale.x = 1.0
    text_marker.scale.y = 1.0
    text_marker.scale.z = 1.0

    text_marker.color.r = 1.0
    text_marker.color.g = 1.0
    text_marker.color.b = 1.0

    text_marker.color.a = 1.0

    q = quaternion_from_euler(0, 0, 0)  # default quaternion
    text_marker.pose.orientation.x = q[0]
    text_marker.pose.orientation.y = q[1]
    text_marker.pose.orientation.z = q[2]
    text_marker.pose.orientation.w = q[3]

    marker_array.markers.append(text_marker)

    # publish
    bbox3d_pub.publish(marker_array)


def publish_3d_bbox(bbox3d_pub, corners_bbox3d_velo_list, obj_types, track_IDs, lifetime):

    marker_array = MarkerArray()
    for i, corners_bbox3d_velo in enumerate(corners_bbox3d_velo_list):
        # marker [LINE_LIST]: 3d bbox
        marker = Marker()
        marker.header.frame_id = FRAME_ID
        marker.header.stamp = Time()

        marker.id = i
        marker.action = Marker.ADD
        # *2 improve the smoothy
        marker.lifetime = rclpy.duration.Duration(seconds=2*lifetime).to_msg()
        marker.type = Marker.LINE_LIST

        if obj_types is None:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
        else:
            b, g, r = DETECTION_COLOR_MAP[obj_types[i]]
            marker.color.r = r/255.0
            marker.color.g = g/255.0
            marker.color.b = b/255.0

        marker.color.a = 1.0
        marker.scale.x = 0.1

        marker.points = []
        for l in LINES:
            p1 = corners_bbox3d_velo[l[0]]
            marker.points.append(Point(x=p1[0], y=p1[1], z=p1[2]))
            p2 = corners_bbox3d_velo[l[1]]
            marker.points.append(Point(x=p2[0], y=p2[1], z=p2[2]))

        q = quaternion_from_euler(0, 0, 0)  # default quaternion
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]

        marker_array.markers.append(marker)

        # text_marker [TEXT_VIEW_FACING]: text lable
        text_marker = Marker()
        text_marker.header.frame_id = FRAME_ID
        text_marker.header.stamp = Time()

        text_marker.id = i + 1000  # track_id
        text_marker.action = Marker.ADD
        text_marker.lifetime = rclpy.duration.Duration(
            seconds=2*lifetime).to_msg()
        text_marker.type = Marker.TEXT_VIEW_FACING

        # p4 = corners_bbox3d_velo[4]  # upper front left corner (z+0.5)
        p4 = np.mean(corners_bbox3d_velo, axis=0)  # box centre point (z+2)
        text_marker.pose.position.x = p4[0]
        text_marker.pose.position.y = p4[1]
        text_marker.pose.position.z = p4[2]+2

        text_marker.text = str(track_IDs[i])  # lable is the track_id

        text_marker.scale.x = 1.0
        text_marker.scale.y = 1.0
        text_marker.scale.z = 1.0

        if obj_types is None:
            text_marker.color.r = 0.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
        else:
            b, g, r = DETECTION_COLOR_MAP[obj_types[i]]
            text_marker.color.r = r/255.0
            text_marker.color.g = g/255.0
            text_marker.color.b = b/255.0
        text_marker.color.a = 1.0

        q = quaternion_from_euler(0, 0, 0)  # default quaternion
        text_marker.pose.orientation.x = q[0]
        text_marker.pose.orientation.y = q[1]
        text_marker.pose.orientation.z = q[2]
        text_marker.pose.orientation.w = q[3]

        marker_array.markers.append(text_marker)

    bbox3d_pub.publish(marker_array)


def publish_distance(dist_pub, minPQDs, lifetime):
    marker_array = MarkerArray()
    for i, (minP, minQ, minD) in enumerate(minPQDs):
        # line PQ
        marker = Marker()
        marker.header.frame_id = FRAME_ID
        marker.header.stamp = Time()

        marker.action = Marker.ADD
        marker.lifetime = rclpy.duration.Duration(seconds=2*lifetime).to_msg()
        marker.type = Marker.LINE_STRIP
        marker.id = i

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8  # alpha
        marker.scale.x = 0.07  # scale, i.e. line width

        marker.points = []
        marker.points.append(Point(x=minP[0], y=minP[1], z=0.0))
        marker.points.append(Point(x=minQ[0], y=minQ[1], z=0.0))

        marker_array.markers.append(marker)

        # distance
        text_marker = Marker()
        text_marker.header.frame_id = FRAME_ID
        text_marker.header.stamp = Time()

        text_marker.id = i + 1000
        text_marker.action = Marker.ADD
        text_marker.lifetime = rclpy.duration.Duration(
            seconds=2*lifetime).to_msg()
        text_marker.type = Marker.TEXT_VIEW_FACING

        pos = (minP + minQ)/2.0
        text_marker.pose.position.x = pos[0]
        text_marker.pose.position.y = pos[1]
        text_marker.pose.position.z = 0.0

        text_marker.text = '%.2f' % minD

        text_marker.scale.x = 1.0
        text_marker.scale.y = 1.0
        text_marker.scale.z = 1.0

        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0

        marker_array.markers.append(text_marker)

    dist_pub.publish(marker_array)
