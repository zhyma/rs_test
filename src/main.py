import sys
import copy

import numpy as np

import rospy

from rod_finder import rod_finder
from rs2o3d import rs2o3d
from workspace_tf import workspace_tf
from rgb_camera import image_converter

import cv2

## run `roslaunch rs2pcl ar_bc_test.launch` first

def main():
    rospy.init_node('rs2icp', anonymous=True)
    rate = rospy.Rate(10)
    rospy.sleep(1)

    rs = rs2o3d()
    ws_tf = workspace_tf()
    rf = rod_finder()
    ic = image_converter()

    while rs.is_data_updated==False:
        rate.sleep()

    while ws_tf.tf_updated==False:
        ws_tf.get_tf()
        rate.sleep()

    while ic.has_data==False:
        rate.sleep()

    ws_distance = ws_tf.trans[0]
    print(ws_distance)
    img = copy.deepcopy(ic.cv_image)
    # while True:
    #     cv2.imshow("Image window", img)
    #     cv2.waitKey(3)
    #     rate.sleep()
    
    rf.find_rod(rs.pcd, img, ws_distance)


if __name__ == '__main__':
    main()