# Covert raw RealSense `/camera/depth/image_rect_raw` data to Open3D point cloud data
# Run this first: `roslaunch realsense2_camera rs_camera.launch`

import sys
import copy
import time

import numpy as np
from math import sin, cos, pi

import open3d as o3d

import cv2

class object_mask():
    def __init__(self, pc_array, img, mask_color=(0,0,0)):
        ## ================
        ## image plane
        ## NW(x0,y0)----NE(x1,y0)
        ##  |              |
        ## SW(x1,y0)----SE(x1,y1)
        self.height = img.shape[0]
        self.width = img.shape[1]

        ## ================
        ## selected point cloud
        self.x_min = 10e6
        self.x_max = -10e6
        self.y_min = 10e6
        self.y_max = -10e6
        self.z_min = 10e6
        self.z_max = -10e6

        self.img = img
        self.masked_img = copy.deepcopy(self.img)
        self.pc_array = copy.deepcopy(pc_array)
        self.mask_color = mask_color

    def apply_mask(self):
        ## used the point clouds of selected cluster to extract 
        ## the rod from the RGB image.
        for iy in range(self.height):
            for ix in range(self.width):
                idx = iy*self.width+ix
                ## check if the given (x,y,z) is inside the bounding box
                ## otherwise apply the mask
                p = self.pc_array[idx]
                
                # print(self.pc_array[0])
                if (p[0] > self.x_min) and (p[0] < self.x_max) and \
                   (p[1] > self.y_min) and (p[1] < self.y_max) and \
                   (p[2] > self.z_min) and (p[2] < self.z_max):
                    ## keep the color
                    pass
                else:
                    self.masked_img[iy, ix, 0] = self.mask_color[0]
                    self.masked_img[iy, ix, 1] = self.mask_color[1]
                    self.masked_img[iy, ix, 2] = self.mask_color[2]
                    pass

        # cv2.imshow('image', self.masked_img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

    def img_dilation(self):
        ...


        