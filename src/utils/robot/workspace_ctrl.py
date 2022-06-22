#!/usr/bin/env python3

# modified from https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from transforms3d import euler
from geometry_msgs.msg import Pose

# from .spiral import path_generator

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class move_yumi():
    def __init__(self, robot, scene, ctrl_group):
        self.robot = robot
        self.scene = scene
        self.ctrl_group = ctrl_group
        

        planning_frame = self.ctrl_group[0].get_planning_frame()
        print("============ Reference frame: {0}".format(planning_frame))

        # We can also print the name of the end-effector link for this group:
        eef_link = self.ctrl_group[0].get_end_effector_link()
        print("============ End effector: {0}".format(eef_link))

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print("============ Robot Groups:{0}".format(self.robot.get_group_names()))

        # # Sometimes for debugging it is useful to print the entire state of the
        # # robot:
        # print("============ Printing robot state")
        # print(robot.get_current_state())
        # print('')
    
    def go_to_pose_goal(self, group, pose_goal):
        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        
        group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        plan = group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        group.clear_pose_targets()

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_traj(self, groups, side, path):
        waypoints = []
        for wp in path:
          pose = Pose()
          pose.position.x = wp[0]
          pose.position.y = wp[1]
          pose.position.z = wp[2]

          if side == 0:
            # left
            quat = euler.euler2quat(pi, 0, -pi/2, 'sxyz')
          else:
            # right
            quat = euler.euler2quat(pi, 0, pi/2, 'sxyz')
          pose.orientation.x = quat[0]
          pose.orientation.y = quat[1]
          pose.orientation.z = quat[2]
          pose.orientation.w = quat[3]
          waypoints.append(pose)

        (plan, fraction) = groups[side].compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def execute_plan(self, plan, group):
        group.execute(plan, wait=True)

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('yumi_test', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    ctrl_group = []
    ctrl_group.append(moveit_commander.MoveGroupCommander('left_arm'))
    ctrl_group.append(moveit_commander.MoveGroupCommander('right_arm'))

    yumi = move_yumi(robot, scene, ctrl_group)

    # pose_goal = geometry_msgs.msg.Pose()
    # q = euler.euler2quat(pi, 0, -pi/2, 'sxyz')
    # pose_goal.position.x = 0.5
    # pose_goal.position.y = 0.1
    # pose_goal.position.z = 0.42
    # pose_goal.orientation.x = q[0]
    # pose_goal.orientation.y = q[1]
    # pose_goal.orientation.z = q[2]
    # pose_goal.orientation.w = q[3]
    # yumi.go_to_pose_goal(yumi.ctrl_group[0], pose_goal)
    
    # print("moving to the starting point")

    # pg = path_generator()
    # path = pg.generate_path()
    # # pg.publish_waypoints(path)

    # rospy.sleep(2)

    # cartesian_plan, fraction = yumi.plan_cartesian_traj(yumi.group_l, path)
    # yumi.execute_plan(cartesian_plan, yumi.group_l)
