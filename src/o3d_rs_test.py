# Covert raw RealSense `/camera/depth/image_rect_raw` data to Open3D point cloud data
# Run this first: `roslaunch realsense2_camera rs_camera.launch`

import sys
import rospy
import numpy as np

import time

from math import pi

from sensor_msgs.msg import Image, CameraInfo, PointCloud2

import open3d as o3d

from lib_cloud_conversion_between_Open3D_and_ROS import convertCloudFromOpen3dToRos

class rs2pc():
    def __init__(self):
        self.is_k_empty = True
        self.k = [0]*9 # camera's intrinsic parameters
        self.cam_sub = rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.cam_info_callback)
        self.img_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.img_callback)
        self.pcd = o3d.geometry.PointCloud()


    def img_callback(self, data):
        height = data.height
        width = data.width
        np_cloud = np.zeros((height*width,3))
        for iy in range(height):
            for ix in range(width):
                idx = iy*width+ix
                z = data.data[idx]/1000.0
                if z!=0:
                    np_cloud[idx][0] = z*(ix-self.k[2])/self.k[0] #x
                    np_cloud[idx][1] = z*(iy-self.k[5])/self.k[4] #y
                    np_cloud[idx][2] = z

        self.pcd.points = o3d.utility.Vector3dVector(np_cloud)

    def cam_info_callback(self, data):
        if self.is_k_empty:
            for i in range(9):
                self.k[i] = data.K[i]
            self.is_k_empty = False


def main():
    rs = rs2pc()

    rospy.init_node('rs2icp', anonymous=True)
    rospy.sleep(1)

    # rospy.sleep(3)

    while True:
        # print(rs.k)
        o3d.visualization.draw_geometries([rs.pcd])
        time.sleep(1)
    #     rospy.sleep(1)
    #     #rospy.spin()


if __name__ == '__main__':
    
    main()