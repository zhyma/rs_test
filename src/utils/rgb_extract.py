# Covert raw RealSense `/camera/depth/image_rect_raw` data to Open3D point cloud data
# Run this first: `roslaunch realsense2_camera rs_camera.launch`

import sys
import copy
import time

import numpy as np
from math import sin, cos, pi
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt

import open3d as o3d

import cv2

def display_img(img):
    cv2.imshow('image', img)
    show_window = True
    while show_window:
        k = cv2.waitKey(0) & 0xFF
        if k == 27:#ESC
            cv2.destroyAllWindows()
            show_window = False

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
        self.mask = np.zeros((self.height, self.width), dtype=np.uint8)
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

                if (p[0] > self.x_min) and (p[0] < self.x_max) and \
                   (p[1] > self.y_min) and (p[1] < self.y_max) and \
                   (p[2] > self.z_min) and (p[2] < self.z_max):
                   self.mask[iy, ix] = 1

        kernel = np.ones((21, 21), dtype=np.uint8)
        ## dilation
        self.mask = cv2.dilate(self.mask, kernel, iterations=1)

        # for iy in range(self.height):
        #     for ix in range(self.width):
        #         if self.mask[iy, ix] == 1:
        #             ## selected area, keep the color
        #             # self.masked_img[iy, ix, 0] = self.mask_color[0]
        #             # self.masked_img[iy, ix, 1] = self.mask_color[1]
        #             # self.masked_img[iy, ix, 2] = self.mask_color[2]
        #             pass
        #         else:
        #             ## areas taken as background
        #             self.masked_img[iy, ix, 0] = self.mask_color[0]
        #             self.masked_img[iy, ix, 1] = self.mask_color[1]
        #             self.masked_img[iy, ix, 2] = self.mask_color[2]
        #             pass

        

    def extract_rod(self):
        # ## use RGB to do Kmeans
        # feature_map = []
        # for iy in range(self.height):
        #     for ix in range(self.width):
        #         if self.mask[iy, ix] == 1:
        #             feature_map.append([iy, ix, self.img[iy, ix, 0], self.img[iy, ix, 1], self.img[iy, ix, 2]])

        # feature_map = np.array(feature_map)
        # kmeans = KMeans(n_clusters = 3).fit(feature_map[:,2:5])

        # use HSV to do Kmeans
        hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        # h = hsv[:,:,0]
        # s = hsv[:,:,1]
        # v = hsv[:,:,2]
        feature_map = []
        for iy in range(self.height):
            for ix in range(self.width):
                if self.mask[iy, ix] == 1:
                    feature_map.append([iy, ix, hsv[iy, ix, 0]])

        # feature_map = np.array(temp_map, dtype=np.uint8)
        feature_map = np.array(feature_map)
        print(feature_map.shape)
        plt.hist(hsv[:,:,0].reshape(-1,1), 256)
        plt.show()

        # display_img(h)
        # color_map = []
        # for iy in range(self.height):
        #     for ix in range(self.width):
        #         if self.mask[iy, ix] == 1:
        #             color_map.append(h[iy, ix, 0])

        # color_map = np.array(color_map)
        ## 3 clusters: rod, support, residual background
        ## https://scikit-learn.org/stable/modules/generated/sklearn.cluster.KMeans.html
        kmeans = KMeans(n_clusters = 3).fit(feature_map[:,2].reshape(-1,1))
        centers = kmeans.cluster_centers_

        clusters = np.ones((self.height, self.width, 3), dtype=np.uint8)*255
        # for idx, value in enumerate(kmeans.labels_):
        for idx, value in enumerate(feature_map):
            y = value[0]
            x = value[1]
            clusters[y, x, 0] = hsv[y, x, 0]
            clusters[y, x, 1] = hsv[y, x, 0]
            clusters[y, x, 2] = hsv[y, x, 0]
            # label = kmeans.labels_[idx]
            # if label == 0:
            #     clusters[y, x, 0] = int(0)
            #     clusters[y, x, 1] = int(255)
            #     clusters[y, x, 2] = int(255)
            # elif label ==1:
            #     clusters[y, x, 0] = int(255)
            #     clusters[y, x, 1] = int(0)
            #     clusters[y, x, 2] = int(255)
            # else:
            #     clusters[y, x, 0] = int(255)
            #     clusters[y, x, 1] = int(255)
            #     clusters[y, x, 2] = int(0)

        # 
        # print(centers)
        # for i in range(len(centers)):
        #     print(centers[i])
        #     y = int(centers[i][0])
        #     x = int(centers[i][1])
        #     print((x, y))
        #     cv2.circle(self.img, (y,x), 5, (0, i*50, 0), thickness=10)

        display_img(clusters)
        ...


        