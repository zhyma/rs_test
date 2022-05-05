# Covert raw RealSense `/camera/depth/image_rect_raw` data to Open3D point cloud data
# Run this first: `roslaunch realsense2_camera rs_camera.launch`

import sys
import rospy
import numpy as np
from math import sin, cos, pi

import time

from math import pi


import open3d as o3d


def main():
    pcd = o3d.io.read_point_cloud("./workspace.pcd")
    o3d.visualization.draw_geometries([pcd])

if __name__ == '__main__':
    
    main()