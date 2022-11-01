#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from sensor_msgs.msg import Image, PointCloud2, PointField
import std_msgs.msg as std_msgs
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import struct
from open3d import *

# IMG_CHN = {'l_gray': 'image_00',
#            'r_gray': 'image_01',
#            'l_RGB': 'image_02',
#            'r_RGB': 'image_03'}


def read_imgraw(path, frame, image_chn, grey_mode=False):
    img_filename = '%010d.png' % frame
    img_path = os.path.join(path, image_chn, 'data')

    if not grey_mode:
        cvimg = cv2.imread(os.path.join(img_path, img_filename))
    else:
        cvimg = cv2.imread(os.path.join(img_path, img_filename), 0)
    return cvimg


def read_pcd(path, frame):
    pcd_folder = os.path.join(path, 'velodyne_points/data/')
    pcd_filename = '%010d.bin' % frame
    pcd_path = os.path.join(pcd_folder, pcd_filename)

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

    pcd = convert_kitti_bin_to_pcd(pcd_path)
    return pcd


def read_pcl(path, frame):
    pcl_folder = os.path.join(path, 'velodyne_points/data/')
    pcl_filename = '%010d.bin' % frame
    pcl_path = os.path.join(pcl_folder, pcl_filename)

    pcl = np.fromfile(pcl_path, dtype=np.float32).reshape(-1, 4)
    return pcl[::2]
