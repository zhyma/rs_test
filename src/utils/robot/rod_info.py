#!/usr/bin/env python3

import math
from math import pi
import numpy as np

import rospy
from std_msgs.msg import Float64

# tf has been deprecated
import tf
from tf.transformations import quaternion_from_matrix, quaternion_matrix
from transforms3d import euler

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from rs2pcl.msg import CylinderProperties

import sys
import copy
# import rospy
import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg

class rod_info():

    def __init__(self, scene, rate):
        # You need to initializing a node before instantiate the class
        self.scene = scene
        self.rod_state = CylinderProperties()
        self.rate = rate

    def set_info(self, mat, l, r):
        q = quaternion_from_matrix(mat)
        o = mat[:3,3]
        self.rod_state.position.x = o[0]
        self.rod_state.position.y = o[1]
        self.rod_state.position.z = o[2]
        self.rod_state.orientation.x = q[0]
        self.rod_state.orientation.y = q[1]
        self.rod_state.orientation.z = q[2]
        self.rod_state.orientation.w = q[3]
        self.rod_state.l = l
        self.rod_state.r = r

    def scene_add_rod(self):
        updated = False
        cylinder_pose = PoseStamped()
        cylinder_pose.header.frame_id = "world"
        # assign cylinder's pose
        cylinder_pose.pose.position = copy.deepcopy(self.rod_state.position)
        cylinder_pose.pose.orientation = copy.deepcopy(self.rod_state.orientation)
        cylinder_name = "cylinder"
        # add_cylinder(self, name, pose, height, radius)
        self.scene.add_cylinder(cylinder_name, cylinder_pose, self.rod_state.l, self.rod_state.r)

        # ensuring collision updates are received
        start = rospy.get_time()
        seconds = rospy.get_time()
        timeout = 5
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # attached_objects = self.scene.get_attached_objects([cylinder_name])
            # print("attached_objects: ", end=',')
            # print(attached_objects)
            # is_attached = len(attached_objects.keys()) > 0

            is_known = cylinder_name in self.scene.get_known_object_names()

            # if (is_attached) and (is_known):
            #    return True
            if is_known:
                return True

            self.rate.sleep()
            seconds = rospy.get_time()
                
        return False


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("rod_detector", anonymous=True)
    scene = moveit_commander.PlanningSceneInterface()
    detector = rod_detection(scene)

    # wait for a while till the subscriber is ready
    rospy.sleep(1)

    print(detector.scene_add_rod(detector.rod_state))
    print(detector.rod_state)
