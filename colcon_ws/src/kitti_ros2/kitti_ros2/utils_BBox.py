#!/usr/bin/env python3
import numpy as np


def compute_3d_box_cam2(h, w, l, x, y, z, yaw):
    """
    Return: 3xn in cam2 coordinate
    """
    # https://github.com/pratikac/kitti/blob/master/readme.tracking.txt
    # we don't consider roll and pitch, just the heading orientation, i.e. yaw
    R = np.array([[np.cos(yaw), 0, np.sin(yaw)], [0, 1, 0],
                 [-np.sin(yaw), 0, np.cos(yaw)]])  # rotation matrix

    # x, y, z correspond to l, h, w respectively
    # the positions realtive to box centre
    x_corners = [l/2, l/2, -l/2, -l/2, l/2, l/2, -l/2, -l/2]
    y_corners = [0,   0,   0,    0,    -h,  -h,  -h,  -h]
    z_corners = [w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2]
    # 8 corner points, convient for the later vstack()

    corners_3d_cam2 = np.dot(R, np.vstack([x_corners, y_corners, z_corners]))
    # x, y ,z ist the position of the bottom surface centre point
    corners_3d_cam2 += np.vstack([x, y, z])

    return corners_3d_cam2


def distance_point_to_segment(P, A, B):
    """
    Calculate the min distance of a point P to a segment AB
    Returns thedistance and the Point Q in AB on which the min distance is reached
    """
    AP = P - A
    BP = P - B
    AB = B - A
    if np.dot(AB, AP) >= 0 and np.dot(-AB, BP) >= 0:
        return np.abs(np.cross(AP, AB))/np.linalg.norm(AB), np.dot(AP, AB)/np.dot(AB, AB)*AB + A
    else:
        d_PA = np.linalg.norm(AP)
        d_PB = np.linalg.norm(BP)
        if d_PA < d_PB:
            return d_PA, A
        else:
            return d_PB, B


def min_distance_cuboids(cub1, cub2):
    """
    Compute the minimum distance between two non-overlapping cuboids (3D) of shape (8, 3)
    They are projected to BEV and the minimum distance of the two rectangles are returned
    """
    minD = 1e5
    for i in range(4):
        for j in range(4):
            d, Q = distance_point_to_segment(
                cub1[i, :2], cub2[j, :2], cub2[j+1, :2])
            if d < minD:
                minD = d
                minP = cub1[i, :2]
                minQ = Q

    for i in range(4):
        for j in range(4):
            d, Q = distance_point_to_segment(
                cub2[i, :2], cub1[j, :2], cub1[j+1, :2])
            if d < minD:
                minD = d
                minP = cub2[i, :2]
                minQ = Q

    return minP, minQ, minD
