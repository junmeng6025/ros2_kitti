import cv2
import numpy as np
import kitti_ros2.camera_config as camera_config

window_size = 100
min_disp = 16
num_disp = 96 - min_disp

############ FROZEN ###########
# STEREO = cv2.StereoSGBM_create(
#     minDisparity=5,
#     numDisparities=4*16,
#     blockSize=3,
#     uniquenessRatio=10,
#     speckleRange=100,  # 视差变化阈值
#     speckleWindowSize=100,
#     disp12MaxDiff=-1,  # 左右视差图的最大容许差异
#     P1=8*3*3*3,  # 惩罚系数
#     P2=32*3*3*3,
#     preFilterCap=1,
#     mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
# )
############ FROZEN ###########

############ FROZEN ###########
# more smooth
# STEREO = cv2.StereoSGBM_create(
#     minDisparity=5,
#     numDisparities=6*16,
#     blockSize=5,
#     P1=8 * 3 * 10,
#     P2=32 * 3 * 10,
#     disp12MaxDiff=1,
#     uniquenessRatio=15,
#     speckleWindowSize=31,
#     speckleRange=2,
#     preFilterCap=3,
#     mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
# )
############ FROZEN ###########

# STEREO = cv2.StereoSGBM_create(
#     minDisparity=5,
#     numDisparities=8*16,
#     blockSize=5,
#     uniquenessRatio=10,
#     speckleRange=10,  # 视差变化阈值
#     speckleWindowSize=100,
#     disp12MaxDiff=1,  # 左右视差图的最大容许差异
#     P1=8*3*5*5,  # 惩罚系数
#     P2=32*3*5*5,
#     preFilterCap=15,
#     mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
# )

#### Distance value jumps ####
# STEREO = cv2.StereoSGBM_create(
#     minDisparity=5,
#     numDisparities=4*16,
#     blockSize=7,
#     uniquenessRatio=10,
#     speckleRange=20,  # 视差变化阈值
#     speckleWindowSize=20,
#     disp12MaxDiff=1,  # 左右视差图的最大容许差异
#     P1=8*3*6*6,  # 惩罚系数
#     P2=32*3*6*6,
#     preFilterCap=-1,
#     mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
# )
### GOOD ###
STEREO = cv2.StereoSGBM_create(
    minDisparity=5,
    numDisparities=4*16,
    blockSize=3,
    uniquenessRatio=10,
    speckleRange=20,  # 视差变化阈值
    speckleWindowSize=10,
    disp12MaxDiff=1,  # 左右视差图的最大容许差异
    P1=8*3*4*4,  # 惩罚系数
    P2=32*3*4*4,
    preFilterCap=-1,
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
)
### GOOD ###

### ALSO GOOD ####
# STEREO = cv2.StereoSGBM_create(
#     minDisparity=5,
#     numDisparities=6*16,
#     blockSize=3,
#     uniquenessRatio=15,
#     speckleRange=15,  # 视差变化阈值
#     speckleWindowSize=10,
#     disp12MaxDiff=1,  # 左右视差图的最大容许差异
#     P1=8*3*4*4,  # 惩罚系数
#     P2=32*3*4*4,
#     preFilterCap=-1,
#     mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
# )
### ALSO GOOD ####

# STEREO = cv2.StereoSGBM_create(
#     minDisparity=5,
#     numDisparities=9*16,
#     blockSize=5,
#     uniquenessRatio=10,
#     speckleRange=100,  # 视差变化阈值
#     speckleWindowSize=10,
#     disp12MaxDiff=15,  # 左右视差图的最大容许差异
#     P1=8*3*10,  # 惩罚系数
#     P2=32*3*10,
#     preFilterCap=1,
#     mode=cv2.STEREO_SGBM_MODE_HH
# )

# STEREO = cv2.StereoSGBM_create(
#     minDisparity=min_disp,
#     numDisparities=num_disp,
#     blockSize=3,
#     uniquenessRatio=15,
#     speckleRange=15,  # 视差变化阈值
#     speckleWindowSize=window_size,
#     disp12MaxDiff=-1,  # 左右视差图的最大容许差异
#     P1=8*3*window_size**2,  # 惩罚系数
#     P2=32*3*window_size**2,
#     preFilterCap=1,
#     mode=cv2.STEREO_SGBM_MODE_HH
# )

# STEREO = cv2.StereoSGBM_create(
#     minDisparity=-125,
#     numDisparities=16*16,
#     blockSize=3,
#     uniquenessRatio=15,
#     speckleRange=15,  # 视差变化阈值
#     speckleWindowSize=256,
#     disp12MaxDiff=-1,  # 左右视差图的最大容许差异
#     P1=8*3*3*3,  # 惩罚系数
#     P2=32*3*3*3,
#     preFilterCap=63,
#     mode=cv2.STEREO_SGBM_MODE_HH
# )

# STEREO = cv2.StereoSGBM_create(
#     minDisparity=min_disp,
#     numDisparities=num_disp,
#     blockSize=16,
#     P1=8 * 3 * window_size ** 2,
#     P2=32 * 3 * window_size ** 2,
#     disp12MaxDiff=1,
#     uniquenessRatio=10,
#     speckleWindowSize=150,
#     speckleRange=32
# )


def disp_filter(left_image, right_image):
    """WLS filter"""
    wsize = 31
    max_disp = 128
    sigma = 1.5
    lmbda = 8000.0
    # left_matcher = cv2.StereoBM_create(max_disp, wsize)

    # left_matcher = cv2.StereoSGBM_create(
    #     minDisparity=5,
    #     numDisparities=6*16,
    #     blockSize=5,
    #     P1=8 * 3 * 10,
    #     P2=32 * 3 * 10,
    #     disp12MaxDiff=1,
    #     uniquenessRatio=15,
    #     speckleWindowSize=31,
    #     speckleRange=2,
    #     preFilterCap=63,
    #     mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
    # )

    left_matcher = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=16,
        blockSize=5,
        P1=8*3*10,
        P2=32*3*10,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=20,
        speckleRange=20,
        preFilterCap=-1,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
    )

    right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)
    left_disp = left_matcher.compute(left_image, right_image)
    right_disp = right_matcher.compute(right_image, left_image)

    # Now create DisparityWLSFilter
    wls_filter = cv2.ximgproc.createDisparityWLSFilter(left_matcher)
    wls_filter.setLambda(lmbda)
    wls_filter.setSigmaColor(sigma)
    filtered_disp = wls_filter.filter(
        left_disp, left_image, disparity_map_right=right_disp)

    return filtered_disp.astype(np.float32)


def distance_calc(frame1, frame2, stereo=STEREO, rectified=True, WLSfilter=False):
    # ensure rectified
    if not rectified:
        img1_rectified = cv2.remap(
            frame1, camera_config.left_map1, camera_config.left_map2, cv2.INTER_LINEAR)
        img2_rectified = cv2.remap(
            frame2, camera_config.right_map1, camera_config.right_map2, cv2.INTER_LINEAR)
    else:
        img1_rectified = frame1
        img2_rectified = frame2

    # Ensure image in grey scale
    if len(img1_rectified.shape) == 3:
        imgL = cv2.cvtColor(img1_rectified, cv2.COLOR_BGR2GRAY)
    else:
        imgL = img1_rectified
    if len(img2_rectified.shape) == 3:
        imgR = cv2.cvtColor(img2_rectified, cv2.COLOR_BGR2GRAY)
    else:
        imgR = img2_rectified

    # Calculate disparity
    if not WLSfilter:
        # raw SGBM
        disp = stereo.compute(imgL, imgR).astype(np.float32) / 16.0
    else:
        # SGBM + WLS filter
        disp = disp_filter(imgL, imgR)

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
