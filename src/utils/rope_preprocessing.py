import sys
import copy

import numpy as np
import cv2
from sklearn.mixture import GaussianMixture

def edge_detection(img, box2d):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    ## extract feature_map from img by using the 2d bounding box
    height = img.shape[0]
    width = img.shape[1]
    feature_map = []
    masked_image = np.zeros((720,1280), dtype=np.uint8)
    for iy in range(height):
        for ix in range(width):
            j = cv2.pointPolygonTest(np.array(box2d), (ix,iy), False)
            if j > 0:
                feature_map.append([iy, ix, hsv[iy, ix, 0]])
                masked_image[iy, ix] = hsv[iy, ix, 2]
            else:
                ...
                masked_image[iy, ix] = 0
    feature_map = np.array(feature_map)
    print(feature_map.shape)


    cv2.imshow('image', masked_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # gmm = GaussianMixture(n_components=2).fit(feature_map[:,2].reshape(-1,1))
    # labels = gmm.predict(feature_map[:,2].reshape(-1,1))

    # unique, counts = np.unique(labels, return_counts=True)
    # d = dict(zip(unique, counts))

    # ## The one with the most samples is the rod
    # ## Pick the one with the less samples
    # if d[0] > d[1]:
    #     mean = gmm.means_[1,0]
    #     std = np.sqrt(gmm.covariances_[1,0,0])
    # else:
    #     mean = gmm.means_[0,0]
    #     std = np.sqrt(gmm.covariances_[0,0,0])

    # color_range = [mean-std, mean+std]

    # # print('====')
    # # print('means:')
    # # print(gmm.means_)
    # # print("std:")
    # # print(np.sqrt(gmm.covariances_))
    # # print(color_range)

    # for iy in range(height):
    #     for ix in range(width):
    #         if hsv[iy, ix, 0] <= color_range[1] and hsv[iy, ix, 0] >= color_range[0]:
    #             masked_image[iy, ix] = hsv[iy, ix, 0]

    # test_img = np.zeros((720,1280,3), dtype=np.uint8)
    # for iy in range(height):
    #     for ix in range(width):
    #         if hsv[iy, ix, 0] <= color_range[1] and hsv[iy, ix, 0] >= color_range[0]:
    #             test_img[iy, ix, 0] = img[iy, ix, 0]
    #             test_img[iy, ix, 1] = img[iy, ix, 1]
    #             test_img[iy, ix, 2] = img[iy, ix, 2]

    # kernel = np.ones((3, 3), dtype=np.uint8)
    # masked_image = cv2.erode(masked_image, kernel, iterations=1)
    # masked_image = cv2.dilate(masked_image, kernel, iterations=1)

    # # cv2.imshow('image', test_img)
    # # cv2.waitKey(0)
    # # cv2.destroyAllWindows()

    # ## the bounding box should only contain the rod and the rope
    # ## (two colors), using 

    # ## get the hue range of the rope

    # low_sigma = cv2.GaussianBlur(masked_image, (3,3),0)
    # high_sigma = cv2.GaussianBlur(masked_image, (5,5),0)
    # dog = low_sigma - high_sigma
    # cv2.imshow('image',dog)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()