#!/usr/bin/env python3

import math
from math import pi
import numpy as np

import rospy
from std_msgs.msg import Float64

# tf has been deprecated
#import tf
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


class rod_detection():

    def __init__(self, scene):
        # You need to initializing a node before instantiate the class
        self.scene = scene
        self.rod_state = CylinderProperties()
        # directly get the true value from Gazebo
        # rospy.Subscriber("/get_rod_properties",CylinderProperties, self.callback)

    def detect(self):
        # mock up pose and dims
        self.rod_state.position.x = 0.35
        self.rod_state.position.y = -0.15
        self.rod_state.position.z = 0.33
        self.rod_state.orientation.x = 0.707
        self.rod_state.orientation.y = 0
        self.rod_state.orientation.z = 0
        self.rod_state.orientation.w = 0.707
        self.rod_state.r = 0.02
        self.rod_state.l = 0.2

    def callback(self, data):
        # self.rod_state.position = copy.deepcopy(data.position)
        # self.rod_state.orientation = copy.deepcopy(data.orientation)
        # self.rod_state.r = data.r
        # self.rod_state.l = data.l
        ...

    def scene_add_rod(self, rod_info):
        print(rod_info.position)
        cylinder_pose = PoseStamped()
        cylinder_pose.header.frame_id = "world"
        # assign cylinder's pose
        cylinder_pose.pose.position = copy.deepcopy(rod_info.position)
        cylinder_pose.pose.orientation = copy.deepcopy(rod_info.orientation)
        cylinder_name = "cylinder"
        # add_cylinder(self, name, pose, height, radius)
        self.scene.add_cylinder(cylinder_name, cylinder_pose, rod_info.l, rod_info.r)

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

            rospy.sleep(0.1)
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
