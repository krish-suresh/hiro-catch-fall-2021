#!/usr/bin/env python
from math import cos, pi, sin
from tf.transformations import quaternion_from_euler
from hiro_core.XamyabRobot import XamyabRobot, rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import random
import copy
import numpy as np

# roslaunch ur_gazebo xamyab.launch
# roslaunch xamyab_moveit_config xamyab_moveit_planning_execution.launch

class ThrowTest:
    def __init__(self):
        self.robot = XamyabRobot(visualize_trajectory=False)
        self.default_gripper_quaternion = Quaternion(*quaternion_from_euler(pi, 0, 0))
        self.start = Pose(position=Point(*[0.7, -0.1, 0.4]), orientation=self.default_gripper_quaternion)
        # self.robot.right_manipulator.home()
        # self.robot.right_gripper.close()
        self.move_to(self.start)
    def reset(self):
        self.robot.left_gripper.open()
        rospy.loginfo("Going home")
        self.robot.left_manipulator.home()

    def set_up_environment(self, object_name, transform_point):
        object_pose = PoseStamped()
        object_pose.header.frame_id = self.robot.left_manipulator.get_planning_frame()
        rospy.loginfo("Object {} position {}.".format(object_name, transform_point))
        object_pose.pose = Pose(position=Point(*transform_point))
        self.robot.left_gripper.enable_fingers_collisions(object_name, True)
    def move_to_point(self, point_goal):
        self.move_to(Pose(position=point_goal, orientation=self.default_gripper_quaternion))
    def move_to(self, pose_goal):
        pose_goal.orientation = self.default_gripper_quaternion
        self.robot.right_manipulator.set_pose_goal(pose_goal)
        self.robot.right_manipulator.go(wait=True)
        self.robot.right_manipulator.stop()
        self.robot.right_manipulator.clear_pose_targets()
    def follow_path(self, waypoints):
        (plan, fraction) = self.robot.right_manipulator.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold
        self.robot.right_manipulator.execute(plan, wait=True)
    def test(self):
        waypoints = []
        scale = 1
        wpose = self.robot.right_manipulator.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        self.follow_path(waypoints)
if __name__ == "__main__":
    throw = ThrowTest()
    throw.test()