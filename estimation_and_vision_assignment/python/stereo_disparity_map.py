#!/usr/bin/env python
import rospy
import tf
import tf.transformations as tr
from std_msgs.msg import String, Header, ColorRGBA
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from math import sqrt, cos, sin, pi, atan2
from threading import Thread, Lock
from math import pi, log, exp
import random
import numpy as np
from scipy.optimize import least_squares
import sys
import yaml

import cv2
from matplotlib import pyplot as plt


def compute_depth_map(disparity_map, calib_params):
    """Return the depth map from the disparity. Recall that in the slides
    we saw disparities being expressed in meters whereas here they are 
    expressed in pixels, so you need to modify that formula accordingly,
    using the camera calibration parameters""" 

    # TODO: implement this

    # get the focal length
    K_left = calib_params['K_left']
    K_right = calib_params['K_right']
    f = K_left[0][0]
    baseline = calib_params['baseline']
    cx_right = K_right[0][2]
    cx_left = K_left[0][2]
    # Take care the cx difference between two camera
    disparity_map = disparity_map + cx_right -cx_left

    # Make a matrix that focal x baseline
    f_b = np.ones(disparity_map.shape)*f*baseline

    # (focal x basline) / disp
    depth_map = np.divide(f_b, disparity_map, out=np.zeros_like(f_b), where=disparity_map!=0)
    return depth_map


def find_match_along_epipolar_line(img_left, img_right, row_left, col_left, patch_size):
    """Returns the column that best matches the patch of width path_size around (row_left, col_left)
    in the right image. Since these cameras are stereo rectified (parallel stereo cameras)
    you only need to search along the horizontal epipolar line on the right image,
    corresponding to the pixel (row_left, col_left). Note that pixel disparities cannot
    be negative, otherwise the estimated depth is going to be negative """

    # TODO: implement this

    h, w = img_left.shape
    i = row_left
    j = col_left
    p = patch_size
    
    # Dealing with the pixel on the edge:
    # Note: If the no enough room for the patch, set the start idex to the edge 
    i_start = 0 if i - p//2 < 0 else i - p//2
    i_end = h if i + p//2 > h else i + p//2 -1 + p%2  # Note (-1+p%2) take care of the even/odd patch size to make sure patch within the patch_size
    left_j_start = 0 if j - p//2 < 0 else j - p//2
    left_j_end = w if j + p//2 > w else j + p//2 -1 + p%2

    # The patch on the left image
    left_patch = img_left[i_start:i_end, left_j_start:left_j_end]


    min_cost = float("inf")
    min_match_j = float("inf")

    # Search along the epipolar line
    for j_curr in range(int(j), -1, -1): # reverse order

        # update the current col position
        j_start = 0 if j_curr - p//2 < 0 else j_curr - p//2
        j_end = w if j_curr + p//2 > w else j_curr + p//2 -1 + p%2

        # Note the j_curr approaching to the edge, we cannot get the full patch anymore
        # To care this situation, the size of left_patch need to be changed accordingly
        # Calculation the width difference between the current right_patch and the patch_size
        curr_patch_width_diff = p - (j_end - j_start+1)
        
        # Adjust size of the left_patch that we can still calculate the cost but using the partial patch
        if ( curr_patch_width_diff > 0 ):
            left_patch = img_left[i_start:i_end, left_j_start + curr_patch_width_diff:left_j_end]


        # get the patch on the right image
        right_patch = img_right[i_start:i_end, j_start:j_end]

        # Option1: Sum of absolute differences(SAD)
        # cost = np.mean(np.abs(left_patch - right_patch))#/curr_patch_size

        # Option2: Sum of squared diff(SSD)
        cost = np.mean((left_patch - right_patch)**2)
        # Note the assignment handout use this formula cost = np.sum((left_patch - right_patch)**2). However
        # I use a adpative windows, so I need to average out the total cost.

        # Option3: Normalized cross-correlation(NCC)
        # Note the NCC if finding the most simular patch. So we need to find max_cost instead of the min_cost
        #left_mean = left_patch.mean()
        #left_minus_mean = left_patch - left_mean
        #left_sqrt_sum_minus_mean = np.sqrt(np.sum(left_patch) - left_mean)
        #left_total = (left_minus_mean/left_sqrt_sum_minus_mean)
        #right_mean = right_patch.mean()
        #right_minus_mean = right_patch - right_mean
        #right_sqrt_sum_minus_mean = np.sqrt(np.sum(right_patch) - right_mean)
        #right_total = (right_minus_mean/right_sqrt_sum_minus_mean)
        #cost = np.sum(left_total * right_total)

        # Update the cost if we find the smaller one
        if cost < min_cost:
            min_cost = cost
            min_match_j = j_curr
    
    return min_match_j


def compute_disparity_map(img_left, img_right, calib_params, patch_size, step):
    rows, cols = img_right.shape
    hps = patch_size/2
    
    disparity_map = np.zeros((rows/step, cols/step), dtype='float')
    
    for r in xrange(hps, rows-hps, step):
        print "Computing disparities along row", r
        for c in xrange(hps, cols-hps, step):
            c_right = find_match_along_epipolar_line(img_left, img_right, r, c, patch_size)
            disparity_map[r / step, c / step] = c - c_right  
    return disparity_map

if __name__ == "__main__":

    if len(sys.argv) < 3:
        print "Usage: stereo_disparity_map.py  path/to/image_left.png  path/to/image_right.png params.yaml"
        sys.exit(1)

    
    image0 = sys.argv[1]
    image1 = sys.argv[2]
    yaml_file = sys.argv[3]
    
    imgL = cv2.imread(image0, 0)
    imgR = cv2.imread(image1, 0)

    stream = open(yaml_file, "r")
    yaml_params = yaml.load(stream)
    
    K_left = np.array([yaml_params['fmx_left'], 0.0, yaml_params['cx_left'],
                       0.0, yaml_params['fmy_left'], yaml_params['cy_left'],
                       0.0, 0.0, 1.0]).reshape((3,3))

    K_right = np.array([yaml_params['fmx_right'], 0.0, yaml_params['cx_right'],
                       0.0, yaml_params['fmy_right'], yaml_params['cy_right'],
                       0.0, 0.0, 1.0]).reshape((3,3))

    baseline = yaml_params['baseline']
    
    calib_params = {'K_left': K_left,
                    'K_right': K_right,
                    'baseline': baseline}
    
    print(calib_params)
    
    print "Rows =", imgL.shape[0], "Cols = ", imgL.shape[1]
    disparity_map = compute_disparity_map(imgL, imgR, calib_params, patch_size=yaml_params['patch_size'], step=yaml_params['step'])
    valid = (disparity_map < yaml_params['max_valid_disparity_in_pixels']) * (disparity_map > yaml_params['min_valid_disparity_in_pixels'])

    depth_map = compute_depth_map(disparity_map * valid, calib_params)

    plt.figure()
    plt.imshow(disparity_map * valid, cmap='jet_r')
    plt.colorbar()
    plt.title("Disparity (in pixels)")

    valid = (depth_map < yaml_params['max_valid_depth_in_meters']) * (depth_map > yaml_params['min_valid_depth_in_meters'])
    
    plt.figure()
    plt.imshow(depth_map * valid, cmap='jet_r')
    plt.colorbar()
    plt.title("Depth (in m)")
    
    plt.show()
