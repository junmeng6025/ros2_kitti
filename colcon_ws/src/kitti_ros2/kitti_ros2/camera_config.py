import numpy as np
import cv2

# RGB CAMERA #######################################################################################################################################

# # check "calib_cam_to_cam.txt"
# # R, T relative to cam_0

# # cam_03
# right_camera_matrix = np.array([[9.037596e+02, 0.000000e+00, 6.957519e+02],
#                                 [0.000000e+00, 9.019653e+02, 2.242509e+02],
#                                 [0.000000e+00, 0.000000e+00, 1.000000e+00]])
# right_distortion = np.array(
#     [[-3.639558e-01, 1.788651e-01, 6.029694e-04, -3.922424e-04, -5.382460e-02]])

# # cam_02
# left_camera_matrix = np.array([[9.597910e+02, 0.000000e+00, 6.960217e+02],
#                                [0.000000e+00, 9.569251e+02, 2.241806e+02],
#                                [0.000000e+00, 0.000000e+00, 1.000000e+00]])
# left_distortion = np.array(
#     [[-3.691481e-01, 1.968681e-01, 1.353473e-03, 5.677587e-04, -6.770705e-02]])

# # calc SE in MATLAB
# R = np.matrix([
#     [0.9996, 0.0223, -0.0198],
#     [-0.0222, 0.9998, 0.0017],
#     [0.0198, -0.0013, 0.9998],
# ])

# T = np.array([-0.5327, 0.0080, -0.0054])


# GRAY CAMERA ########################################################################################################################################
# -> ACTUALLY USED FOR SGBM

# check "calib_cam_to_cam.txt"
# R, T relative to cam_0

# cam_01
right_camera_matrix = np.array([[9.895267e+02, 0.000000e+00, 7.020000e+02],
                                [0.000000e+00, 9.878386e+02, 2.455590e+02],
                                [0.000000e+00, 0.000000e+00, 1.000000e+00]])
right_distortion = np.array(
    [[-3.644661e-01, 1.790019e-01, 1.148107e-03, -6.298563e-04, -5.314062e-02]])

# cam_00
left_camera_matrix = np.array([[9.842439e+02, 0.000000e+00, 6.900000e+02],
                               [0.000000e+00, 9.808141e+02, 2.331966e+02],
                               [0.000000e+00, 0.000000e+00, 1.000000e+00]])
left_distortion = np.array(
    [[-3.728755e-01, 2.037299e-01, 2.219027e-03, 1.383707e-03, -7.233722e-02]])

# calc SE in MATLAB
R = np.matrix([
    [0.9994, 0.0186, -0.0308],
    [-0.0189, 0.9998, -0.0084],
    [0.0307, 0.0090, 0.9995],
])

T = np.array([-0.5370, 0.0048, -0.0125])


size = (1242, 375)  # 图像尺寸  the rectified image size
# size = (1382, 512)  # the native image size, see https://github.com/yanii/kitti-pcl/blob/master/KITTI_README.TXT

# 进行立体更正
R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(left_camera_matrix, left_distortion,
                                                                  right_camera_matrix, right_distortion, size, R,
                                                                  T)
# 计算更正map
left_map1, left_map2 = cv2.initUndistortRectifyMap(
    left_camera_matrix, left_distortion, R1, P1, size, cv2.CV_16SC2)
right_map1, right_map2 = cv2.initUndistortRectifyMap(
    right_camera_matrix, right_distortion, R2, P2, size, cv2.CV_16SC2)
