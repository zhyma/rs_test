import sys
import copy

import numpy as np

from utils.vision.rod_finder import rod_finder

import open3d as o3d
import cv2

## run `roslaunch rs2pcl ar_bc_test.launch` first
## the default function
def main():
    import rospy
    from utils.vision.rs2o3d import rs2o3d
    from utils.vision.workspace_tf import workspace_tf
    from utils.vision.rgb_camera import image_converter

    import moveit_commander
    from utils.robot.detect_rod      import rod_detection
    # from utility.detect_cable    import cable_detection
    from utils.robot.workspace_ctrl  import move_yumi
    from utils.robot.jointspace_ctrl import joint_ctrl
    from utils.robot.path_generator  import path_generator
    from utils.robot.gripper_ctrl    import gripper_ctrl
    from utils.robot.add_marker      import marker

    from geometry_msgs.msg import Pose
    from transforms3d import euler
    import moveit_msgs.msg

    import tf
    from tf.transformations import quaternion_from_matrix, quaternion_matrix

    br = tf.TransformBroadcaster()

    rospy.init_node('wrap_wrap', anonymous=True)
    rate = rospy.Rate(10)
    rospy.sleep(1)

    pg = path_generator()
    gripper = gripper_ctrl()
    goal = marker()


    ##-------------------##
    ## Detect the rod in the first place
    rs = rs2o3d()
    ws_tf = workspace_tf()
    rf = rod_finder()
    ic = image_converter()

    ## There is depth data in the RS's buffer
    while rs.is_data_updated==False:
        rate.sleep()

    print("depth_data_ready")

    # ## Link the transformation of the vision system to the robot coordinate
    # while ws_tf.tf_updated==False:
    #     ws_tf.get_tf()
    #     rate.sleep()

    # print("tf_data_ready")

    listener = tf.TransformListener()
    bc = tf.TransformBroadcaster()

    # (trans_yumi,rot_yumi) = listener.lookupTransform('/world','yumi_base_link', rospy.Time(0))
    # t_base2world = quaternion_matrix(rot_yumi)
    # t_base2world[:3,3] = trans_yumi

    # print(t_base2world)

    t_ar2world = np.array([[0, 0, 1, 0],[1, 0, 0, 0],[0, 1, 0, 0.07],[0, 0, 0, 1]])
    # t_ar2world = np.dot(t_base2world, t_ar2base)

    # print(t_ar2world)

    # q = quaternion_from_matrix(t_ar2world)
    # o = t_ar2world[:3,3]
    # bc.sendTransform(o, q, rospy.Time.now(), "test_frame", "world")

    updated = False
    while updated == False:
        try:
            (trans_cam2ar,rot_cam2ar) = listener.lookupTransform('ar_marker_90','camera_link', rospy.Time(0))
            updated = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            updated = False

        rate.sleep()
    t_cam2ar = quaternion_matrix(rot_cam2ar)
    t_cam2ar[:3,3] = trans_cam2ar
    # t_cam2ar = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

    print(t_cam2ar)

    t_cam2world = np.dot(t_ar2world,t_cam2ar)
    q = quaternion_from_matrix(t_cam2world)
    o = t_cam2world[:3,3]
    
    bc.sendTransform(o, q, rospy.Time.now(), "camera_link", "world")

    ## There is RGB data in the RS's buffer
    while ic.has_data==False:
        rate.sleep()

    print("rgb_data_ready")



if __name__ == '__main__':

    main()