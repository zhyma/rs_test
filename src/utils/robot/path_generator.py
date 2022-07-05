#!/usr/bin/env python3

from math import pi, sin, cos, sqrt, atan2
import numpy as np

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from transforms3d import euler


class path_generator():

    def __init__(self):
        self.waypoint_pub = rospy.Publisher('yumi_waypoint', Path, queue_size=1, latch=True)
        ...

    def generate_nusadua(self, ref_rod_pose, l, r , step_size):
        ## For left hand
        ## curve on x-z plane
        ## from 2d plane curve to world frame path: [xr, adv, yr]
        path = []
        n_samples = 12

        ## T^{ref}_{obj}: from ref to obj
        ## t_gb2ft: from gripper_base to finger_tip: move along z, 12.5mm
        ## t_ft2gb is the inverse matrix (ft to gb)
        t_gb2ft = np.array([[1, 0, 0, 0],\
                            [0, 1, 0, 0],\
                            [0, 0, 1, 0.125],\
                            [0, 0, 0, 1]])
        t_ft2gb = np.linalg.inv(t_gb2ft)
        for i in range(n_samples):
            
            t = 2*pi/n_samples * i
            x = r*cos(t)
            y = r*sin(t)
            ## based on the world coordinate
            xr  = x-(l-t*r)*sin(t) * ( 1)
            yr  = y+(l-t*r)*cos(t) * (-1)
            adv = step_size * i/n_samples

            t_curve2d = np.array([[1, 0, 0, xr],\
                                  [0, 1, 0, adv],\
                                  [0, 0, 1, yr],\
                                  [0, 0, 0, 1]])

            trans = [ref_rod_pose.position.x,\
                     ref_rod_pose.position.y,\
                     ref_rod_pose.position.z]

            rot = [ref_rod_pose.orientation.x,\
                   ref_rod_pose.orientation.y,\
                   ref_rod_pose.orientation.z,\
                   ref_rod_pose.orientation.w]
            
            t_world2rod = quaternion_matrix(rot)
            t_world2rod[:3,3] = trans

            ## transform
            # xr  = ref_rod_pose.position.x + x-(l-t*r)*sin(t) * ( 1)
            # yr  = ref_rod_pose.position.z + y+(l-t*r)*cos(t) * (-1)
            # adv = ref_rod_pose.position.y + step_size * i/n_samples

            ## ?
            t_world2ft = np.dot(world2rod, t_curve2d)

            ## reference frame:world. t_world2gb = t_world2ft*inv(t_gb2ft)
            t_world2gb = np.dot(t_world2ft,t_ft2gb)
            q = quaternion_from_matrix(t_world2gb)
            o = t_world2gb[:3,3]

            pose = Pose()
            pose.position.x = xr
            pose.position.y = adv
            pose.position.z = yr

            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]
            path.append(pose)
        
        return path

    def generate_spiral(self, spiral_params, gripper_states):
        path = []
        # # test to plot a line.
        # for i in range(30):
        #     path.append([0.3, i/100, 0.4])

        
        # s is the center of the rod
        # x (front-back), y (left-right), z (up-down).
        # x and z are used for spiral, y is used for advance
        s = spiral_params
        # s = [0.3, 0.1, 0.4]
        # g is the gripper's starting position
        g = gripper_states
        # g = [0.6, 0.1, 0.42]
        # starting angle theta_0
        t_0 = 0.2
        # ending angle theta_end
        t_e = 0.2+pi*2
        n_samples = 12
        n = -1/2
        # offset theta_offset
        # t_os = 0
        t_os = atan2((g[2]-s[2]),(g[0]-s[0]))

        # r = a\theta^(-1/2)
        r_0 = np.sqrt((s[0]-g[0])**2+(s[2]-g[2])**2)
        a = r_0/np.power(t_0, n)
        for i in range(n_samples):
            dt = (t_e-t_0)*i/n_samples
            r = a*np.power((t_0+dt), n)
            t = -dt + t_os
            x = r*np.cos(t) + s[0]
            # x = -r*np.cos(t) + s[0]
            y = g[1] + 0.005*i
            z = r*np.sin(t) + s[2]
            path.append([x,y,z])
        
        return path

    def generate_line(self, start = [0, 0, 0], stop=[0, 0, 0]):
        path = []

        dist = sqrt((stop[0]-start[0])**2 + (stop[1]-start[1])**2 + (stop[2]-start[2])**2)
        no_of_points = int(dist/0.05) + 1
        print('number of waypoints are: {0}'.format(no_of_points))
        dx = (stop[0]-start[0])/(no_of_points - 1)
        dy = (stop[1]-start[1])/(no_of_points - 1)
        dz = (stop[2]-start[2])/(no_of_points - 1)

        for i in range(no_of_points):
            path.append([start[0]+i*dx, start[1]+i*dx, start[2]+i*dx])

        return path

    def publish_waypoints(self, path):
        """
        Publish the ROS message containing the waypoints
        """

        msg = Path()
        msg.header.frame_id = "world"
        msg.header.stamp = rospy.Time.now()

        for wp in path:
            pose = PoseStamped()
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            pose.pose.position.z = wp[2]

            quat = euler.euler2quat(-pi/2, 0, pi/2, 'sxyz')
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]
            msg.poses.append(pose)

        self.waypoint_pub.publish(msg)
        rospy.loginfo("Published {} waypoints.".format(len(msg.poses)))
        return 
