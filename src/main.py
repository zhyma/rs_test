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
    # from utils.vision.workspace_tf import workspace_tf
    from utils.vision.rgb_camera import image_converter

    import moveit_commander
    from utils.robot.detect_rod      import rod_detection
    # from utility.detect_cable    import cable_detection
    from utils.robot.workspace_ctrl  import move_yumi
    from utils.robot.jointspace_ctrl import joint_ctrl
    from utils.robot.path_generator  import path_generator
    from utils.robot.gripper_ctrl    import gripper_ctrl
    from utils.robot.add_marker      import marker

    from utils.workspace_tf          import workspace_tf

    from geometry_msgs.msg import Pose
    from transforms3d import euler
    import moveit_msgs.msg

    import tf
    from tf.transformations import quaternion_from_matrix, quaternion_matrix

    bc = tf.TransformBroadcaster()

    rospy.init_node('wrap_wrap', anonymous=True)
    rate = rospy.Rate(10)
    rospy.sleep(1)

    pg = path_generator()
    gripper = gripper_ctrl()
    goal = marker()

    ##-------------------##
    ## initializing the moveit 
    moveit_commander.roscpp_initialize(sys.argv)
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()
    ctrl_group = []
    ctrl_group.append(moveit_commander.MoveGroupCommander('left_arm'))
    ctrl_group.append(moveit_commander.MoveGroupCommander('right_arm'))

    ## initialzing the yumi motion planner
    yumi = move_yumi(robot, scene, ctrl_group)

    ##-------------------##
    ## reset the robot
    gripper.l_open()
    gripper.r_open()
    j_ctrl = joint_ctrl(ctrl_group)
    j_ctrl.robot_default_l_low()
    j_ctrl.robot_default_r_low()

    gripper.l_open()
    gripper.r_open()

    rospy.sleep(3)

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
    # updated = False
    # while ws_tf.tf_updated==False:
    #     ws_tf.get_tf()
    #     rate.sleep()

    print("tf_data_ready")

    listener = tf.TransformListener()
    # bc = tf.TransformBroadcaster()

    t_ar2world = np.array([[0, 0, 1, 0],[1, 0, 0, 0],[0, 1, 0, 0.07],[0, 0, 0, 1]])

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

    t_cam2world = np.dot(t_ar2world,t_cam2ar)
    q = quaternion_from_matrix(t_cam2world)
    o = t_cam2world[:3,3]
    
    bc.sendTransform(o, q, rospy.Time.now(), "camera_link", "world")
    rospy.sleep(1)

    ## There is RGB data in the RS's buffer
    while ic.has_data==False:
        rate.sleep()

    print("rgb_data_ready")

    rod2cam_trans, _ = ws_tf.get_tf('camera_depth_frame', 'ar_marker_90')
    ws_distance = rod2cam_trans[0]
    print(ws_distance)
    img = copy.deepcopy(ic.cv_image)
    # while True:
    #     cv2.imshow("Image window", img)
    #     cv2.waitKey(3)
    #     rate.sleep()
    
    rf.find_rod(rs.pcd, img, ws_distance)

    ## broadcasting the rod's tf
    trans = rf.rod_transformation
    rod_pos = (trans[0][3], trans[1][3], trans[2][3])
    print("rod's position: ", end="")
    print(rod_pos)
    rod_rot = quaternion_from_matrix(trans)
    print("rod's orientation: ", end="")
    print(rod_rot)
    bc.sendTransform(rod_pos,rod_rot, rospy.Time.now(), 'rod', 'camera_depth_frame')
    rospy.sleep(1)

    # # ##-------------------##
    # # ## rod found, start to do the first wrap

    # rod = rod_detection(scene)
    # rod.get_tf()

    # rod.scene_add_rod(rod.rod_state)
    # ## Need time to initializing
    # rospy.sleep(3)

    # pose_goal = Pose()

    # rod_x = rod.rod_state.position.x
    # rod_y = rod.rod_state.position.y
    # rod_z = rod.rod_state.position.z

    # # x = rod_x
    # # y = 0
    # # z = 0.10
    # # start = [x+0.01, y + 0.1+0.25, z]
    # # stop  = [x+0.01, y + 0.1, z]

    # # goal.show(x=stop[0], y=stop[1], z=stop[2])

    # # path = [start, stop]
    # # cartesian_plan, fraction = yumi.plan_cartesian_traj(ctrl_group, 0, path)
    # # yumi.execute_plan(cartesian_plan, ctrl_group[0])
    # # print("go to pose have the cable in between gripper: ", end="")
    # # rospy.sleep(2)

    # # gripper.l_close()

    # # # ## left gripper grabs the link
    # # # gripper.l_close()

    # # # ##-------------------##
    # # # ## generate spiral here
    # # # # s is the center of the rod
    # # spiral_params = [rod_x, rod_y, rod_z]
    # # # # g is the gripper's starting position
    # # gripper_states = stop
    # # path = pg.generate_spiral(spiral_params, gripper_states)
    # # pg.publish_waypoints(path)

    # # # path1 = path[0:len(path)//2]
    # # # path2 = path[len(path)//2:]

    # # ## motion planning and executing
    # # cartesian_plan, fraction = yumi.plan_cartesian_traj(ctrl_group, 0, path)
    # # ## fraction < 1: not successfully planned
    # # print(fraction)
    # # yumi.execute_plan(cartesian_plan, ctrl_group[0])
    # # rospy.sleep(2)


    # # start = path[-1]
    # # stop = [start[0], start[1], 0.1]

    # # path = [start, stop]
    # # cartesian_plan, fraction = yumi.plan_cartesian_traj(ctrl_group, 0, path)
    # # yumi.execute_plan(cartesian_plan, ctrl_group[0])

    # # gripper.l_open()
    # # gripper.r_open()


def test_with_files(path):
    img = cv2.imread("./"+ path +"/image.jpeg")
    # cv2.imshow('image',img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    pcd = o3d.io.read_point_cloud("./"+ path +"/workspace.pcd")
    # o3d.visualization.draw_geometries([pcd])
    ws_distance = 850/1000.0

    rf = rod_finder()
    rf.find_rod(pcd, img, ws_distance)
    # load
    ...

if __name__ == '__main__':
    if len(sys.argv) > 1:
        test_with_files(sys.argv[1])
    else:
        main()