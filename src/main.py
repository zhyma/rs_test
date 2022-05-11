# Covert raw RealSense `/camera/depth/image_rect_raw` data to Open3D point cloud data
# Run this first: `roslaunch realsense2_camera rs_camera.launch`

import sys
import copy
import time

import numpy as np
from math import sin, cos, pi

import rospy
import tf

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2

import open3d as o3d

from rod_finder import rod_finder
from rs2o3d import rs2o3d
from workspace_tf import workspace_tf

#from lib_cloud_conversion_between_Open3D_and_ROS import convertCloudFromOpen3dToRos

## run `roslaunch rs2pcl ar_bc_test.launch` first

def main():
    rospy.init_node('rs2icp', anonymous=True)
    rate = rospy.Rate(10)
    rospy.sleep(1)

    rs = rs2o3d()
    ws_tf = workspace_tf()
    rf = rod_finder()

    while rs.is_data_updated==False:
        rate.sleep()

    while ws_tf.tf_updated==False:
        ws_tf.get_tf()
        rate.sleep()

    ws_distance = ws_tf.trans[0]
    print(ws_distance)
    rf.find_rod(rs.pcd, ws_distance)


if __name__ == '__main__':
    main()