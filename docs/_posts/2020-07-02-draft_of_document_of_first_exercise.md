---
layout: post
title: Draft of document of first exercise
---

**This is a draft of the document of first exercise.**

The goal of this exercise is to learn the underlying infrastructure of Industrial Robot exercises(ROS + MoveIt + our own industrial robotics API) and get familiar with the key components needed for more complex exercises by completing the task of pick and place multiple objects and sorting them by color or shape.

![world](https://raw.githubusercontent.com/TheRoboticsClub/colab-gsoc2020-Yijia_Wu/master/docs/img/new_first_exercise_world.png){: .mx-auto.d-block :}

## Installation
1. Install the [General Infrastructure](https://jderobot.github.io/RoboticsAcademy/installation/#generic-infrastructure) of the JdeRobot Robotics Academy.
2. Install MoveIt, ros_controller ...(TO BE DONE)
3. Install Industrial Robot package(TO BE DONE)

## How to run the exercise
TO launch the exercise, open a terminal windows, navigate to the ROS workspace which contains Industrial Robot folder and execute following command:
```bash
catkin_make
source devel/setup.bash
cd /src/models
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${PWD}
cd ../..
roslaunch irb120_robotiq85_gazebo irb120_robotiq85_gazebo_world.launch 
```
Two different windows will pop up:
- **Gazebo simulator**: A warehouse environment with a industrial robot(robot arm and gripper), multiple objects, a conveyor and two boxes will be shown in Gazebo.
- **Industrial robot teleoperator**: A GUI which provideds following functionalities:
    - A Forward Kinematics teleoperator providing sliders to change the angle of each joint of the robot arm and the gripper. The limits of each joints are shown in the two sides of the slider. The true angle of the joints are shown in the text box beside the sliders.
    - An Inverse Kinematics teleoperator allowing users to set the desired end effector pose. 
        - Plan button can plan the trajectory to the desired pose
        - Excute button can make the robot execute the planned trajectory
        - Plan & Execute button is the integration of last two buttons
        - Stop button allows users to stop moving robot
        - Back to home button can make the robot back to home pose
    - Two update button to update the value of current robot state
    - Code Manager part
        - Two buttons to start and stop the main code you program
        - One button to launch Rviz
        - One button to Respawn all objects in Gazebo and Rviz
        - An info box showing the status of the main code("start" or "stop")

![GUI](https://raw.githubusercontent.com/TheRoboticsClub/colab-gsoc2020-Yijia_Wu/master/docs/img/newGUI.png){: .mx-auto.d-block :}

## How should I solve the exercise
To solve the exercise, you must edit the MyAlgorithm.py file and insert control logic in myalgorithm() function. The path of this file is "rqt_kinematics/src/rqt_kinematics/interfaces/MyAlgorithm.py".
```python
def myalgorithm(self, event):
	############## Insert your code here ###############
    # Move the robot back to home as a start
    self.pick_place.back_to_home()
	
    # insert following two lines where you want to stop the algorithm 
    # with the stop button in GUI
    if not event.isSet():
        return

	####################################################
```  
Multiple APIs can be used to implement your algorithm. They are provided in Pick_Place class, so you should allways add "self.pick_place." as a prefix to following introduced APIs in your algorithm.

## API
### Environment Information
* `get_object_list()` - Return the name list of all objects.
* `get_object_pose(object_name)` - Return the pose of the object.
* `get_object_info(object_name)` - Return the pose, shape, height, eidth, length, color of the object.
* `get_target_list()` - Return the name list of all targets.
* `get_target_position(target_name)` - Return the position of target where we are going to place the objects.

### Basic Robot Movement
* `move_pose_arm(pose_goal)` - Command the robot to make its end effector frame move to the desired pose with inverse kinematics.
* `move_joint_arm(joint_0,joint_1,joint_2,joint_3,joint_4,joint_5)` - Command the robot joints to move to desired joints value
* `move_joint_hand(joint_value)` - Command the gripper joint to move to desired joint value.
* `back_to_home()` - Command the robot arm and gripper to move back to the home pose.

### Setup Grasp message
[Grasp message](http://docs.ros.org/melodic/api/moveit_msgs/html/msg/Grasp.html) contains the informations that the MoveIt pick function requires.
* `set_grasp_distance(min_distance, desired_distance)` - Set the minmum distance and desired distance the gripper translates before and after grasping.
* `set_grasp_direction(x, y, z)` - Set the direction of the gripper approach translation before grasping. Retreat translation distance will set to be the opposite direction of the approach direction.
* `generate_grasp(eef_orientation, position, width[, roll, pitch, yaw])` - Returns the specified Grasp message according to related setup. 
    - `eef_orientaion` is to clarify the desired end effector orientation. 
        - `horizontal`: grasp the object with a horizontal gripper orientation. (default: roll = pitch = yaw = 0. `pitch` is setable.)
        - `vertical`: grasp the object with a vertical gripper orientation. (default: roll = 0, pitch = 90°, yaw = 0. `yaw` is setable.)
        - `user_defined`: grasp the object with a user defined gripper orientation. (default: roll = pitch = yaw = 0. `roll`,`pitch`,`yaw` are all setable)
    - `position` is the position of the end effector when grasping the object
    - `width` is the value the gripper joints should move to grasp the object with a range of [0, 0.8]
    - `roll`,`pitch`,`yaw` are optional parameters with default value 0.

### Pick and Place
* `pickup(object_name, grasps)` - Command the industrial robot to pick up the object with the genrated Grasp messages.
* `place(eef_orientation, position[, roll, pitch, yaw])` - Command the industrial robot to place the currently holding object to goal_position with desired end effector orientation.
    - `eef_orientaion` is to clarify the desired end effector orientation. 
        - `horizontal`: grasp the object with a horizontal gripper orientation. (default: roll = 0, pitch = 0, yaw = 180°. `pitch` is setable.)
        - `vertical`: grasp the object with a vertical gripper orientation. (default: roll = 180°, pitch = 90°, yaw = 180°. `yaw` is setable.)
        - `user_defined`: grasp the object with a user defined gripper orientation. (default: roll = 0, pitch = 0, yaw = 180°. `roll`,`pitch`,`yaw` are all setable)
    - `position` is the position of the end effector when placing the object

## Theory
### Relationship among ROS, MoveIt, Rviz, Gazebo, JdeRobot provided API
TO BE DONE
- Draw a example relationship image
- Introduction of each of them and explanation of the relationship

### Difference between MoveIt(Rviz) and Gazebo
Rviz can show the planning scene of MoveIt.

#### Objects visualization
When we start Gazebo with a world file, the objects in Gazebo will not be automatically added in the planning scene of MoveIt, so we can only see the robot in Rviz without any other objects. If we want MoveIt to know there are some objects in the environment, we should manually add them into the planning scene, but you don't need to worry about it in this exercise as it has been done by us. 

#### Obstacle Avoidance
After adding objects into planning scene, MoveIt will avoid planning trajectory for the robot that will lead to collision. However, because the object in planning scene cannot be updated automatically with their pose in Gazebo, after the robot grasping them, we cannot track their poses any more, so MoveIt cannot take them into consideration of obstacle avoidance.

#### Pick and Place
Moveit provides pick and place functions. We can perform great pick and place in planning scene with them, but at the same time, you will see that the objects cannot be picked up in Gazebo. The reason for it is that Gazebo, as a physical simulator, still cannot simulate the contact between two surface perfectly. Even if we can see the robot pick up the object successfully in Rviz, it might not work in Gazebo.

### Difference between real world and Gazebo Simulator
TO BE DONE

## Hints
### Manipulation Pipeline: pick and place one object
- Get the information about the object you are going to grasp, especially the pose and size of them, and the target position
- Find good grasp motion for grasping that object
- Specify the grasp related information in Grasp message
- Command MoveIt to plan and execute the robot to pick the object with specified Grasp message
- Command Moveit to plan and execute the robot to place the object to target position
TO DO: Draw a flow chart

### Why does the robot cannot move to some desired pose?
The most possible reason is that your specified pose is unreachable for the robot arm, so MoveIt cannot plan a trajectory from current pose to desired pose in limited time. You will see such a warning when this problem happened:
```bash
Fail: ABORTED: No motion plan found. No execution attempted.
```
### Ignorable ERROR and WARNING
- `No p gain specified for pid.`
- `Controller is taking too long to execute trajectory`
- `Controller failed with error GOAL_TOLERANCE_VIOLATED`

## Demonstration video of the solution
[![solution](https://img.youtube.com/vi/HnI6L75zaFk/0.jpg){: .mx-auto.d-block :}](https://youtu.be/HnI6L75zaFk)