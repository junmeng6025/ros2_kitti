#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import os

import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs

import numpy as np
import open3d as o3d
import struct
from open3d import *

DATAID = '0005'
ROOT = '/home/jun/ros_kitti'
DATA_PATH = ROOT+'/rawdata/2011_09_26/2011_09_26_drive_'+DATAID+'_sync/'

QUEUE_SZ = 10
FPS = 10.0


def convert_kitti_bin_to_pcd(binFilePath):
    size_float = 4
    list_pcd = []
    with open(binFilePath, "rb") as f:
        byte = f.read(size_float * 4)
        while byte:
            x, y, z, intensity = struct.unpack("ffff", byte)
            list_pcd.append([x, y, z])
            byte = f.read(size_float * 4)
    np_pcd = np.asarray(list_pcd)
    pcd = geometry.PointCloud()
    pcd.points = utility.Vector3dVector(np_pcd)
    return pcd


def point_cloud(points, parent_frame):
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
    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize

    data = points.astype(dtype).tobytes()

    fields = [sensor_msgs.PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1) for i, n in enumerate('xyz')]

    header = std_msgs.Header(frame_id=parent_frame)
    pcd_msg = sensor_msgs.PointCloud2(
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


class PCDPublisher(Node):
    def __init__(self, name):
        super().__init__(name)
        self.frame = 0
        self.pcd_publisher = self.create_publisher(
            sensor_msgs.PointCloud2, 'pcd', QUEUE_SZ)
        timer_peroid = 1/FPS
        self.timer = self.create_timer(timer_peroid, self.timer_callback)

    def timer_callback(self):
        pcd_folder = os.path.join(DATA_PATH, 'velodyne_points/data/')
        pcd_filename = '%010d.bin' % self.frame
        pcd_path = os.path.join(pcd_folder, pcd_filename)

        pcd = convert_kitti_bin_to_pcd(pcd_path)
        self.points = np.asarray(pcd.points)
        self.pcd = point_cloud(self.points, 'map')

        self.pcd_publisher.publish(self.pcd)
        self.get_logger().info("Publishing pcd frame [#%010d]" % self.frame)
        self.frame += 1
        if self.frame >= 154:
            self.frame = 0


def main(args=None):
    rclpy.init(args=args)
    pcd_publisher = PCDPublisher('pcd_publisher_node')
    rclpy.spin(pcd_publisher)
    pcd_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
