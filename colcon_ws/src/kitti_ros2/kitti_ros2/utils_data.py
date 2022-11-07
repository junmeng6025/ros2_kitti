#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from sensor_msgs.msg import Image, PointCloud2, PointField
import std_msgs.msg as std_msgs
from cv_bridge import CvBridge
import cv2
import numpy as np
import pandas as pd
import os
import struct
from open3d import *
from kitti_ros2.utils_BBox import *

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


# ###########################################  read tracking data  ###########################################
TRACKING_COLUMN_NAMES = ['frame', 'track_id', 'type', 'truncated', 'occluded', 'alpha', 'bbox_left', 'bbox_top',
                         'bbox_right', 'bbox_bottom', 'height', 'width', 'length', 'pos_x', 'pos_y', 'pos_z', 'rot_y']

# {RawDataID : TrackTxtID}
TRACK_MATCH = {'0005': '0000',
               '0009': '0001',
               '0013': '0003',  # or 0017
               }


def read_tracking(root_path, dataID):
    data_path = os.path.join(
        root_path, 'data_tracking_label_2/training/label_02/%s.txt' % TRACK_MATCH[dataID])
    df = pd.read_csv(data_path, header=None, sep=' ')

    df.columns = TRACKING_COLUMN_NAMES
    df = df[df['track_id'] >= 0]  # remove DontCare objects
    # Set all vehicle type to 'Vehicle'
    df.loc[df.type.isin(['Bus', 'Truck', 'Van', 'Tram']), 'type'] = 'Vehicle'
    df = df[df.type.isin(['Vehicle', 'Pedestrian', 'Cyclist'])]
    return df


# ###########################################  read BBox data  ###########################################
BBOX_EGO = np.array([[2.15, 0.9, -1.73], [2.15, -0.9, -1.73], [-1.95, -0.9, -1.73], [-1.95, 0.9, -1.73],
                     [2.15, 0.9, -0.23], [2.15, -0.9, -0.23], [-1.95, -0.9, -0.23], [-1.95, 0.9, -0.23]])


def read_3d_bbox(track_id_ls, bbox3d_ls, calib):
    """
    return: 
        corners_bbox3d_velo_list,
        centers_dict
    """
    corners_bbox3d_velo_list = []
    centers_dict = {}  # track_id: center; for track display
    minPQDs = []  # for distance display
    for track_id, bbox3d in zip(track_id_ls, bbox3d_ls):
        # 3d BBox
        corners_bbox3d_cam2 = compute_3d_box_cam2(*bbox3d)  # in bbox_utils.py
        corners_bbox3d_velo = calib.project_rect_to_velo(
            corners_bbox3d_cam2.T)  # here no need another .T, we get a 8x3 array
        # equivalent to .append()
        corners_bbox3d_velo_list += [corners_bbox3d_velo]

        # track
        centers_dict[track_id] = np.mean(corners_bbox3d_velo, axis=0)[
            :2]  # ignore z

        # distance calc
        minPQDs += [min_distance_cuboids(BBOX_EGO, corners_bbox3d_velo)]

    # corners_bbox3d_velo_list += [BBOX_EGO]
    # track_id_ls = np.append(track_id_ls, -1)
    # type_ls = np.append(type_ls, 'Vehicle')

    centers_dict[-1] = np.array([0, 0])  # record the ego_car as track_id = -1
    # In kitti_run.py:
    # ego_car = Object(center=np.array([0, 0]))
    # ...
    # for track_id in centers_dict:
    #             trackers[track_id] = Object(centers_dict[track_id])

    return corners_bbox3d_velo_list, centers_dict, minPQDs
