# Covert raw RealSense `/camera/depth/image_rect_raw` data to Open3D point cloud data
# Run this first: `roslaunch realsense2_camera rs_camera.launch`

import sys
import copy
import time

import numpy as np
from math import sin, cos, pi

import open3d as o3d

import cv2

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp], 
                                      zoom=0.7, front=[-0.85, 0.5, 0.12], 
                                      lookat=[0.67,0.22,0], up=[0,0,1], left=1680)

class object_projection():
    def __init__(self, pc_array, img):
        ## ================
        ## image plane
        ## NW(x0,y0)----NE(x1,y0)
        ##  |              |
        ## SW(x1,y0)----SE(x1,y1)
        self.img_x0 = 10e6
        self.img_x1 = 0
        self.img_y0 = 10e6
        self.img_y1 = 0

        ## ================
        ## selected point cloud
        self.selected_x_min = 10e6
        self.selected_x_max = -10e6
        self.selected_y_min = 10e6
        self.selected_y_max = -10e6
        self.selected_z_min = 10e6
        self.selected_z_max = -10e6

        self.img = img
        self.pcd = pcd

    def space2img():
        ## `/camera/depth/image_rect_raw` coordinate
        ## y (left & right), z (up & down) are on the camera plane, x is the depth
        ...


class rod_finder():
    def __init__(self,downsample_size = 0.005, eps=0.05, min_points=10, cluster_size = 500):
        self.rod_template = o3d.geometry.PointCloud()
        self.downsample_size = downsample_size
        self.eps = eps
        self.min_points = min_points
        self.cluster_size = cluster_size

    def create_cylinder_template(self, r=20/1000.0, l=200/1000.0):
        ## create a rod template here
        t_theta=60 # divide pi into 60 sampling points
        t_l = int(l*1000) # t_l sampling points along the rod
        obj_np_cloud = np.zeros((t_theta*t_l,3))
        for il in range(t_l):
            for it in range(t_theta):
                idx = il*t_theta+it

                obj_np_cloud[idx][2] = r*cos(it*pi/t_theta)
                obj_np_cloud[idx][0] = -r*sin(it*pi/t_theta)
                obj_np_cloud[idx][1] = il/1000.0

        self.rod_template.points = o3d.utility.Vector3dVector(obj_np_cloud)

    def find_rod(self, raw_pcd, img, ws_distance):
        ## pcd is the raw point cloud data
        ## 1. based on the workspace distacne, 
        ## remove point cloud outside that distance
        ## ws_distance, measured in meters

        print("Finding the rod...")

        ## ================
        ## 1. Remove points that beyond the robot
        raw_array = np.asarray(raw_pcd.points)
        op = object_projection(img, raw_array)

        ws_array = []
        for i in range(raw_array.shape[0]):
            # x is the depth direction in RealSense coordiante
            # if env_cloud[i][0] < 850/1000.0:
            if raw_array[i][0] < ws_distance:
                ws_array.append([raw_array[i][0], raw_array[i][1], raw_array[i][2]])

        ws_pcd = o3d.geometry.PointCloud()
        ws_pcd.points = o3d.utility.Vector3dVector(np.asarray(ws_array))

        ## ================
        ## 2. Downsample pcd for clustering to reduce the computational load
        ds_pcd = ws_pcd.voxel_down_sample(voxel_size=self.downsample_size)

        ## ================
        ## 3. Apply DBSCAN clustering
        with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
            labels = np.array(ds_pcd.cluster_dbscan(eps=self.eps, min_points=self.min_points, print_progress=True))

        max_label = labels.max()
        #print(f"point cloud has {max_label + 1} clusters")
        #colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        #colors[labels < 0] = 0
        #ds_pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])

        ## ===============
        ## 4. Select only the front most cluster as the rod/object
        ##    first value: number of points assigned to the label
        ##    second value: x distance (depth) associated with this label
        dist_sum = np.zeros(((max_label+1), 2))
        
        for idx, val in enumerate(labels):
            dist_sum[val,0] += 1
            dist_sum[val,1] += ds_pcd.points[idx][0]

        selected = 0
        min_dist = 10.0e6
        for i in range(max_label+1):
            ## must include more than 500 points (to avoid rope fragments)
            if (dist_sum[i,0] > self.cluster_size):
                dist = dist_sum[i,1]/dist_sum[i,0]
                # print(dist)
                if dist < min_dist:
                    selected = i
                    min_dist = dist

        print("select label", selected, ", with distance ", min_dist)
        selected_pcd = o3d.geometry.PointCloud()
        selected_array = np.zeros((int(dist_sum[selected,0]), 3))
        cnt = 0
        for idx, val in enumerate(labels):
            if val == selected:
                selected_array[cnt,0] = ds_pcd.points[idx][0]
                selected_array[cnt,1] = ds_pcd.points[idx][1]
                selected_array[cnt,2] = ds_pcd.points[idx][2]
                cnt += 1


                if ds_pcd.points[idx][0] < op.selected_x_min:
                    op.selected_x_min = ds_pcd.points[idx][0]
                if ds_pcd.points[idx][0] > op.selected_x_max:
                    op.selected_x_max = ds_pcd.points[idx][0]
                if ds_pcd.points[idx][1] < op.selected_y_min:
                    op.selected_y_min = ds_pcd.points[idx][1]
                if ds_pcd.points[idx][1] > op.selected_y_max:
                    op.selected_y_max = ds_pcd.points[idx][1]
                if ds_pcd.points[idx][2] < op.selected_z_min:
                    op.selected_z_min = ds_pcd.points[idx][2]
                if ds_pcd.points[idx][2] > op.selected_z_max:
                    op.selected_z_max = ds_pcd.points[idx][2]

        selected_pcd.points = o3d.utility.Vector3dVector(np.asarray(selected_array))

        ## ================
        ## 5. Get the geometric information of the cluster
        ## TODO: replace with OpenCV rectangle regconition to get a more accurate center.
        
        print(op.selected_x_min)
        print(op.selected_x_max)
        print(op.selected_y_min)
        print(op.selected_y_max)


        cluster_center = [0.0, 0.0, 0.0]
        for i in range(len(selected_pcd.points)):
            cluster_center[0] += selected_pcd.points[i][0]
            cluster_center[1] += selected_pcd.points[i][1]
            cluster_center[2] += selected_pcd.points[i][2]

        for i in range(3):
            cluster_center[i] = cluster_center[i]/len(selected_pcd.points)

        ## ===============
        ## 6. Create a cylinder template for ICP
        self.create_cylinder_template(r=20/1000.0, l=200/1000.0)

        ## ===============
        ## 7. Apply ICP to register the rod's pose
        target = selected_pcd
        source = self.rod_template
        threshold = 0.02
        trans_init = np.asarray(
                    [[1.0, 0, 0,  cluster_center[0]],
                    [0, 1.0, 0,  cluster_center[1]],
                    [0, 0,  1.0, cluster_center[2]],
                    [0.0, 0.0, 0.0, 1.0]])

        print("Apply point-to-point ICP")
        reg_p2p = o3d.pipelines.registration.registration_icp(source, target, threshold, trans_init,
                o3d.pipelines.registration.TransformationEstimationPointToPoint())
        print(reg_p2p)
        print("Transformation is:")
        print(reg_p2p.transformation)
        print("")
        # draw_registration_result(source, target, reg_p2p.transformation)
        draw_registration_result(source, ds_pcd, reg_p2p.transformation)
