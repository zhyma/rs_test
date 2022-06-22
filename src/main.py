import sys
import copy

import numpy as np

from utils.rod_finder import rod_finder
from utils.rope_preprocessing import *

import open3d as o3d
import cv2

## run `roslaunch rs2pcl ar_bc_test.launch` first
## the default function
def main():
    import rospy
    from utils.rs2o3d import rs2o3d
    from utils.workspace_tf import workspace_tf
    from utils.rgb_camera import image_converter

    rospy.init_node('rs2icp', anonymous=True)
    rate = rospy.Rate(10)
    rospy.sleep(1)

    rs = rs2o3d()
    ws_tf = workspace_tf()
    rf = rod_finder()
    ic = image_converter()

    while rs.is_data_updated==False:
        rate.sleep()

    print("depth_data_ready")

    while ws_tf.tf_updated==False:
        ws_tf.get_tf()
        rate.sleep()

    print("tf_data_ready")

    while ic.has_data==False:
        rate.sleep()

    print("rgb_data_ready")

    ws_distance = ws_tf.trans[0]
    print(ws_distance)
    img = copy.deepcopy(ic.cv_image)
    # while True:
    #     cv2.imshow("Image window", img)
    #     cv2.waitKey(3)
    #     rate.sleep()
    
    rf.find_rod(rs.pcd, img, ws_distance)

def test_with_files(path):
    img = cv2.imread("./"+ path +"/image.jpeg")
    # pcd = o3d.io.read_point_cloud("./"+ path +"/workspace.pcd")
    # ws_distance = 850/1000.0

    # rf = rod_finder()
    # rf.find_rod(pcd, img, ws_distance)
    
    # ## pre-process rgb image to get "some" rope data
    # ## bb_2d: 2d bounding box for the rod
    # bb_2d = rf.bouding_box_2d

    bb_2d = [[378, 286], [383, 218], [864, 253], [859, 320]]
    print(bb_2d)

    edge_detection(img, bb_2d)


if __name__ == '__main__':
    if len(sys.argv) > 1:
        test_with_files(sys.argv[1])
    else:
        main()