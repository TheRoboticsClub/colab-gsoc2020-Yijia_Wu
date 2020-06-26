#!/usr/bin/python
import rospy
import rospkg

import sys
import copy
import os
import numpy
import threading

from moveit_commander import RobotCommander,PlanningSceneInterface
from moveit_commander import roscpp_initialize, roscpp_shutdown
from moveit_commander.conversions import pose_to_list
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.msg import Grasp, PlaceLocation
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import String

from math import pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion
from pick_and_place import Pick_Place


class RobotWrapper:
    def __init__(self, arm="irb_120", gripper="robotiq_85"):
        roscpp_initialize(sys.argv)
        #rospy.init_node('pick_and_place')

        self.arm = moveit_commander.MoveGroupCommander(arm)
        self.gripper = moveit_commander.MoveGroupCommander(gripper)

        self.pose = self.arm.get_current_pose().pose
        self.x = self.pose.position.x
        self.y = self.pose.position.y
        self.z = self.pose.position.z
        quaternion = (self.pose.orientation.x,
                    self.pose.orientation.y,
                    self.pose.orientation.z,
                    self.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.roll = euler[0]
        self.pitch = euler[1]
        self.yaw = euler[2]

        self.event = threading.Event()

        #self.joints = self.arm.get_current_joint_values()
        #self.gripper_joint = self.gripper.get_current_joint_values()

    def test_connection(self):
        print("connected with robot wrapper")

    # move one joint of the arm to value
    def set_arm_joint(self, joint_id, value):
        joint_goal = self.arm.get_current_joint_values()
        joint_goal[joint_id-1] = value
        self.arm.go(joint_goal, wait=True)
        self.arm.stop()

    # Forward Kinematics (FK): move the arm by axis values
    def move_joint_arm(self,joint_0,joint_1,joint_2,joint_3,joint_4,joint_5):
        joint_goal = self.arm.get_current_joint_values()
        joint_goal[0] = joint_0
        joint_goal[1] = joint_1
        joint_goal[2] = joint_2
        joint_goal[3] = joint_3
        joint_goal[4] = joint_4
        joint_goal[5] = joint_5

        self.arm.go(joint_goal, wait=True)
        self.arm.stop() # To guarantee no residual movement

    def get_joints_value(self, joint_id):
        joints = self.arm.get_current_joint_values()
        return joints[joint_id-1]

    def set_x(self, value):
        self.x = value
    
    def set_y(self, value):
        self.y = value

    def set_z(self, value):
        self.z = value

    def set_roll(self, value):
        self.roll = value
    
    def set_pitch(self, value):
        self.pitch = value

    def set_yaw(self, value):
        self.yaw = value

    def get_arm_position(self):
        pose = self.arm.get_current_pose().pose
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        return x, y, z

    def get_arm_orientation(self):
        pose = self.arm.get_current_pose().pose
        quaternion = (pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        return roll, pitch, yaw

    def set_random_pose(self):
        self.arm.set_random_target()

    def back_to_home(self):
        self.move_joint_arm(0,0,0,0,0,0)
        self.move_joint_hand(0)
        
    # Inverse Kinematics (IK): move TCP to given position and orientation
    def move_pose_arm(self, roll, pitch, yaw, x, y, z):
        pose_goal = geometry_msgs.msg.Pose()
        quat = quaternion_from_euler(roll,pitch,yaw)
        pose_goal.orientation.x = quat[0]
        pose_goal.orientation.y = quat[1]
        pose_goal.orientation.z = quat[2]
        pose_goal.orientation.w = quat[3]
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        self.arm.set_pose_target(pose_goal)

        self.arm.go(wait=False)

        self.arm.stop() # To guarantee no residual movement
        self.arm.clear_pose_targets()

    # Move the Robotiq gripper by master axis
    def move_joint_hand(self,gripper_finger1_joint):
        joint_goal = self.gripper.get_current_joint_values()
        joint_goal[2] = gripper_finger1_joint # Gripper master axis

        self.gripper.go(joint_goal, wait=False)
        self.gripper.stop() # To guarantee no residual movement

    def get_gripper_joint_value(self):
        joints = self.gripper.get_current_joint_values()
        #print(joints[2])
        return joints[2]

    # stop move groups execution
    def stop_execution(self):
        self.arm.stop()
        self.gripper.stop()

    def stop(self):
        roscpp_shutdown()

    def plan(self):
        pose_goal = Pose()
        quat = quaternion_from_euler(self.roll,self.pitch,self.yaw)
        pose_goal.orientation.x = quat[0]
        pose_goal.orientation.y = quat[1]
        pose_goal.orientation.z = quat[2]
        pose_goal.orientation.w = quat[3]
        pose_goal.position.x = self.x
        pose_goal.position.y = self.y
        pose_goal.position.z = self.z
        self.arm.set_pose_target(pose_goal)

    def execute(self):
        self.arm.go(wait=False)
        self.arm.stop() # To guarantee no residual movement
        self.arm.clear_pose_targets()

    def pick_and_place(self, event):
        self.pick_place = Pick_Place(self.arm, self.gripper)
        self.pick_place.MyAlgorithm(event)