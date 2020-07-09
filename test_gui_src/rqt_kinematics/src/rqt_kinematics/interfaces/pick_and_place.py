#! /usr/bin/env python

import rospy
import rospkg

import sys
import copy
import os
import numpy

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

from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion, Vector3, Point
import threading
import yaml
from model_manager import Object


class Pick_Place:
    def __init__ (self, arm, gripper, object_list):
        self.object_list = object_list
        self.goal_list = {}
        self.set_target_info()

        self.arm = arm
        self.gripper = gripper

        self.arm.set_goal_tolerance(0.01)

        self.scene = PlanningSceneInterface()
        self.robot = RobotCommander()
        rospy.sleep(1)

        # set default grasp message infos
        self.set_grasp_distance(0.1, 0.2)
        self.set_grasp_direction(0, 0, -0.5)

        rospy.sleep(1.0)

    def clean_scene(self, object_name):
        self.scene.remove_world_object(object_name)

    def set_target_info(self):
        filename = os.path.join(rospkg.RosPack().get_path('rqt_kinematics'), 'src','rqt_kinematics', 'interfaces', 'models_info.yaml')
        with open(filename) as file:
            objects_info = yaml.load(file)
            robot_x = objects_info["robot"]["pose"]["x"]
            robot_y = objects_info["robot"]["pose"]["y"]
            robot_z = objects_info["robot"]["pose"]["z"]

            targets = objects_info["targets"]
            target_name = targets.keys()
            for name in target_name:
                position = Point()
                position.x = targets[name]["x"] - robot_x
                position.y = targets[name]["y"] - robot_y
                position.z = targets[name]["z"] - robot_z
                self.goal_list[name] = position

    def build_plannning_scene(self):
        filename = os.path.join(rospkg.RosPack().get_path('rqt_kinematics'), 'src','rqt_kinematics', 'interfaces', 'models_info.yaml')
        with open(filename) as file:
            objects_info = yaml.load(file)
            robot_x = objects_info["robot"]["pose"]["x"]
            robot_y = objects_info["robot"]["pose"]["y"]
            robot_z = objects_info["robot"]["pose"]["z"]
            robot_roll = objects_info["robot"]["pose"]["roll"]
            robot_pitch = objects_info["robot"]["pose"]["pitch"]
            robot_yaw = objects_info["robot"]["pose"]["yaw"]

            targets = objects_info["targets"]
            target_name = targets.keys()
            for name in target_name:
                x = -(targets[name]["y"] - robot_y)
                y = targets[name]["x"] - robot_x
                z = targets[name]["z"] - robot_z
                self.goal_list[name] = (x, y, z)

            objects = objects_info["objects"]
            objects_name = objects.keys()
            for object_name in objects_name:
                name = object_name
                shape = objects[name]["shape"]
                color = objects[name]["color"]

                p = PoseStamped()
                p.header.frame_id = self.robot.get_planning_frame()
                p.header.stamp = rospy.Time.now()

                self.clean_scene(name)
                p.pose.position.x = objects[name]["pose"]["x"] - robot_x
                p.pose.position.y = objects[name]["pose"]["y"] - robot_y
                p.pose.position.z = objects[name]["pose"]["z"] - robot_z

                q = quaternion_from_euler(robot_roll, robot_pitch, robot_yaw)
                p.pose.orientation = Quaternion(*q)

                if shape == "box":
                    x = objects[name]["size"]["x"]
                    y = objects[name]["size"]["y"]
                    z = objects[name]["size"]["z"]
                    p.pose.position.z += z/2
                    size = (x, y, z)
                    self.scene.add_box(name, p, size)

                    height = z
                    width = y
                    length = x
                    self.object_list[name] = Object(p.pose, height, width, length, shape, color)

                elif shape == "cylinder":
                    height = objects[name]["size"]["height"]
                    radius = objects[name]["size"]["radius"]
                    p.pose.position.z += height/2
                    self.scene.add_cylinder(name, p, height, radius)
                    self.object_list[name] = Object(p.pose, height, radius*2, radius*2, shape, color)

                elif shape == "sphere":
                    radius = objects[name]["size"]
                    p.pose.position.z += radius
                    self.scene.add_sphere(name, p, radius)
                    self.object_list[name] = Object(p.pose, radius*2, radius*2, radius*2, shape, color)

                rospy.sleep(1)
            
            obstacles = objects_info["obstacles"]
            obstacles_name = obstacles.keys()
            for obstacle_name in obstacles_name:
                name = obstacle_name

                p = PoseStamped()
                p.header.frame_id = self.robot.get_planning_frame()
                p.header.stamp = rospy.Time.now()

                self.clean_scene(name)
                p.pose.position.x = obstacles[name]["pose"]["x"] - robot_x
                p.pose.position.y = obstacles[name]["pose"]["y"] - robot_y
                p.pose.position.z = obstacles[name]["pose"]["z"] - robot_z

                q = quaternion_from_euler(robot_roll, robot_pitch, robot_yaw)
                p.pose.orientation = Quaternion(*q)

                x = obstacles[name]["size"]["x"]
                y = obstacles[name]["size"]["y"]
                z = obstacles[name]["size"]["z"]
                size = (x, y, z)
                self.scene.add_box(name, p, size)

                rospy.sleep(1)

    def get_object_list(self):
        return self.object_list.keys()

    def get_target_list(self):
        return self.goal_list.keys()

    def get_object_pose(self, object_name):
        return copy.deepcopy(self.object_list[object_name].relative_pose)

    def get_object_info(self, object_name):
        return self.object_list[object_name]

    def get_target_position(self, target_name):
        return self.goal_list[target_name]

    def pose2msg(self, roll, pitch, yaw, x, y, z):
        pose = geometry_msgs.msg.Pose()
        quat = quaternion_from_euler(roll,pitch,yaw)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        return pose

    def msg2pose(self, pose):
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        quaternion = (pose.orientation.x,
                      pose.orientation.y,
                      pose.orientation.z,
                      pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        return roll, pitch, yaw, x, y, z 

    def back_to_home(self):
        self.move_joint_arm(0,0,0,0,0,0)
        self.move_joint_hand(0)
        rospy.sleep(1)

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

    # Inverse Kinematics: Move the robot arm to desired pose
    def move_pose_arm(self, pose_goal):
        self.arm.set_pose_target(pose_goal)

        self.arm.go(wait=True)

        self.arm.stop() # To guarantee no residual movement
        self.arm.clear_pose_targets()

    # Move the Robotiq gripper by master axis
    def move_joint_hand(self,gripper_finger1_joint):
        joint_goal = self.gripper.get_current_joint_values()
        joint_goal[2] = gripper_finger1_joint # Gripper master axis

        self.gripper.go(joint_goal, wait=True)
        self.gripper.stop() # To guarantee no residual movement

    def set_grasp_direction(self, x, y, z):
        self.approach_direction = Vector3()
        self.approach_direction.x = x
        self.approach_direction.y = y
        self.approach_direction.z = z

        self.retreat_direction = Vector3()
        self.retreat_direction.x = -x
        self.retreat_direction.y = -y
        self.retreat_direction.z = -z

    def set_grasp_distance(self, min_distance, desired_distance):
        self.approach_retreat_min_dist = min_distance
        self.approach_retreat_desired_dist = desired_distance

    def generate_grasp(self, eef_orientation, position, width, roll = 0, pitch = 0, yaw = 0):
        now = rospy.Time.now()
        grasp = Grasp()

        grasp.grasp_pose.header.stamp = now
        grasp.grasp_pose.header.frame_id = self.robot.get_planning_frame()
        #grasp.grasp_pose.pose = Pose()

        grasp.grasp_pose.pose.position = position

        # Setting pre-grasp approach
        grasp.pre_grasp_approach.direction.header.stamp = now
        grasp.pre_grasp_approach.direction.header.frame_id = self.robot.get_planning_frame()
        grasp.pre_grasp_approach.direction.vector = self.approach_direction
        grasp.pre_grasp_approach.min_distance = self.approach_retreat_min_dist
        grasp.pre_grasp_approach.desired_distance = self.approach_retreat_desired_dist

        # Setting post-grasp retreat
        grasp.post_grasp_retreat.direction.header.stamp = now
        grasp.post_grasp_retreat.direction.header.frame_id = self.robot.get_planning_frame()
        grasp.post_grasp_retreat.direction.vector = self.retreat_direction
        grasp.post_grasp_retreat.min_distance = self.approach_retreat_min_dist
        grasp.post_grasp_retreat.desired_distance = self.approach_retreat_desired_dist

        if eef_orientation == "horizontal":
            q = quaternion_from_euler(0.0, numpy.deg2rad(pitch), 0.0)
        elif eef_orientation == "vertical":
            q = quaternion_from_euler(0.0, numpy.deg2rad(90.0), numpy.deg2rad(yaw))
        elif eef_orientation == "user_defined":
            q = quaternion_from_euler(numpy.deg2rad(roll), numpy.deg2rad(pitch), numpy.deg2rad(yaw))

        grasp.grasp_pose.pose.orientation = Quaternion(*q)

        grasp.max_contact_force = 1000

        grasp.pre_grasp_posture.joint_names.append("gripper_finger1_joint")
        traj = JointTrajectoryPoint()
        traj.positions.append(0.0)
        traj.time_from_start = rospy.Duration.from_sec(0.5)
        grasp.pre_grasp_posture.points.append(traj)

        grasp.grasp_posture.joint_names.append("gripper_finger1_joint")
        traj = JointTrajectoryPoint()
        traj.positions.append(width)

        traj.time_from_start = rospy.Duration.from_sec(5.0)
        grasp.grasp_posture.points.append(traj)

        return grasp

    # pick up object with grasps
    def pickup(self, object_name, grasps):
        rospy.loginfo('Start picking '+object_name)
        self.arm.pick(object_name, grasps)
        #self.gripper.stop()

        rospy.loginfo('Pick up finished')
        self.arm.detach_object(object_name)
        self.clean_scene(object_name)
        #rospy.sleep(1)

    # place object to goal position
    def place(self, eef_orientation, position, roll = 180, pitch = 0, yaw = 180):
        pose = Pose()
        pose.position = position

        distance = 0.1

        if eef_orientation == "horizontal":
            q = quaternion_from_euler(numpy.deg2rad(180), numpy.deg2rad(pitch), numpy.deg2rad(180))
        elif eef_orientation == "vertical":
            q = quaternion_from_euler(0.0, numpy.deg2rad(90.0), numpy.deg2rad(yaw))
        elif eef_orientation == "user_defined":
            q = quaternion_from_euler(numpy.deg2rad(roll), numpy.deg2rad(pitch), numpy.deg2rad(yaw))

        pose.orientation = Quaternion(*q)
        pose.position.z += distance

        rospy.loginfo('Start placing')
        self.move_pose_arm(pose)
        rospy.sleep(1)
        
        waypoints = []
        wpose = self.arm.get_current_pose().pose
        wpose.position.z -= distance
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.arm.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold
        self.arm.execute(plan, wait=True)

        self.move_joint_hand(0)
        rospy.sleep(1)
        
        # pose.position.z += 0.1
        # self.move_pose_arm(pose)

        waypoints = []
        wpose = self.arm.get_current_pose().pose
        wpose.position.z += distance
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.arm.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold
        self.arm.execute(plan, wait=True)

        rospy.loginfo('Place finished')

