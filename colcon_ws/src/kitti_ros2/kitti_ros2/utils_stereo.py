import cv2
import numpy as np
import kitti_ros2.camera_config as camera_config

# PROBLEM:
# unable to match the identical in L and R images preciously

STEREO = cv2.StereoSGBM_create(
    minDisparity=5,
    numDisparities=4*16,
    blockSize=3,
    uniquenessRatio=10,
    speckleRange=100,  # 视差变化阈值
    speckleWindowSize=100,
    disp12MaxDiff=-1,  # 左右视差图的最大容许差异
    P1=8*3*3*3,  # 惩罚系数
    P2=32*3*3*3,
    preFilterCap=1,
    mode=cv2.STEREO_SGBM_MODE_HH
)


def distance_calc(frame1, frame2, stereo=STEREO, rectified=True):
    if not rectified:
        img1_rectified = cv2.remap(
            frame1, camera_config.left_map1, camera_config.left_map2, cv2.INTER_LINEAR)
        img2_rectified = cv2.remap(
            frame2, camera_config.right_map1, camera_config.right_map2, cv2.INTER_LINEAR)
    else:
        img1_rectified = frame1
        img2_rectified = frame2

    imgL = cv2.cvtColor(img1_rectified, cv2.COLOR_BGR2GRAY)
    imgR = cv2.cvtColor(img2_rectified, cv2.COLOR_BGR2GRAY)
    disp = stereo.compute(imgL, imgR).astype(np.float32) / 16.0
    threeD = cv2.reprojectImageTo3D(disp, camera_config.Q)
    return threeD, disp


def calc_xc(df_bbox):
    """
    return a dictionary containing xc of 2d bboxs
    """
    xc_dict = {}
    for typ, xyxy in df_bbox:
        xc_dict[typ] = (xyxy[0] + xyxy[2]) / 2

    return xc_dict
