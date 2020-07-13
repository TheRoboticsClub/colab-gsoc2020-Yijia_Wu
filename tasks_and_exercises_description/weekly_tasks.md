## Weekly tasks 

#### WEEK 7.  June 13 - July 20

|      | Tasks                                                        | State   |
| ---- | ------------------------------------------------------------ | ------- |
| 1    | Finish Task 4 of last week (automatic gripper closing) just for *Vertical* and *Horizontal* grasp approaches. | ONGOING |
| 2    | Prepare a solution using the Gazebo world for *exercise 2*, adding three spheres of different colors on the conveyor belt. The position of the spheres must be detected using a color filter, converted to a 3D pose (PCL) and changed to the robot frame (TF). After that a complete pick and place must be performed. Once finished the vision API and goals for exercise 2 will be decided. | ONGOING |
| 3    | Explore ways to activate automatic vision-based obstacle detection building the 3D occupation octomap. Its activation will be available via the vision API. | ONGOING |
| 4    | Update GUI for vision exercises: add new plugin with two images side by side: raw and processed by OpenCV, similar to other Academy exercises. As we have two cameras, it would be ideal to select the camera via its namespace. | ONGOING |

#### [WEEK 6.  July 3 - July 12](https://theroboticsclub.github.io/colab-gsoc2020-Yijia_Wu/2020-07-12-week5_6/)

|      | Tasks                                                        | State   |
| ---- | ------------------------------------------------------------ | ------- |
| 1    | Complete some *missing parts* of Exercise 1. Finish documentation. Migrate the infrastructure (modified plugin, GUI, static objects) to a new branch in the [JdeRobot Industrial Robotics](IndustrialRobotics) repo. Open issue to incorporate the new exercises to the [Robotics Academy](https://github.com/JdeRobot/RoboticsAcademy) repo (issue + new branch), adding launch files, Gazebo world, yaml files, student template and exercise documentation there. | DONE    |
| 2    | Mofify some API methods (move_pose_arm(), move_joint_arm(), move-joint_gripper()) to tell the student via *informative messages* that the given angles or pose are out of bounds.  Angle limits and robot workspace (as simple cartesian limits) will be stored in yaml file. | DONE    |
| 3    | Spawn Gazebo objects to pick using a list of their properties (name, geometry, size, position, color) stored in a yaml file. | DONE    |
| 4    | *API enhancement*: Add planning using the desired TCP gripper pose, in addition to plan using arm end-effector pose. Add also automatic gripper closing without passing the link pose (autofit to object size, mimicking how a pneumatic gripper works). | ONGOING |
| 5    | Start preparing the Gazebo world for *exercise 2* (adding cameras, robot manipulator and vacuum gripper as described [here](../exercises_description/Second_Exercise.md). | DONE    |

#### [WEEK 5.  June 27 - July 2](https://theroboticsclub.github.io/colab-gsoc2020-Yijia_Wu/2020-07-02-draft_of_document_of_first_exercise/)

|      | Tasks                                                        | State |
| ---- | ------------------------------------------------------------ | ----- |
| 1    | Check and update the way the *grasp quality* parameter in the grasp message is computed for the pick() method. | DONE  |
| 2    | Update GUI to include *joints angle limits* read from yaml file. Show  those limits in GUI. | DONE  |
| 3    | Refine proposed API to include three *approach and retreat* options for grasping, and its related input parameters: <br />a) **vertical** (yaw angle at grasp point, distance from approach/retreat point) <br/>b) **horizontal** (roll angle at grasp point, distance from appr./retreat point)<br />c) **user defined** (roll, pitch, yaw angles at grasp point, distances from approach/retreat point, vectors for approach/retreat) | DONE  |
| 4    | Complete the new *Exercise 1: pick and place with industrial robotic manipulator*. Finish Gazebo world. Prepare documentation. Record a demo video | DONE  |

#### [WEEK 4.  June 20 - June 26](https://theroboticsclub.github.io/colab-gsoc2020-Yijia_Wu/2020-06-25-week4/)

|      | Tasks                                                        | State |
| ---- | ------------------------------------------------------------ | ----- |
| 1    | Test other kinematic solvers in OMPL library rather than the default RRT to avoid the *ABORTED: Solution found but controller failed during execution* fails | DONE  |
| 2    | Finish the pending functionalities of the GUI and wrapper for Industrial Robotics exercises. | DONE  |
| 3    | Start coding the new *Exercise 1: pick and place with industrial robotic manipulator*. Define API and build Gazebo world. The descriptions for both API and exercise will be described in separate files. | DONE  |

#### [WEEK 3.  June 12 - June 19 ](https://theroboticsclub.github.io/colab-gsoc2020-Yijia_Wu/2020-06-18-week3/)

|      | Tasks                                                        | State   |
| ---- | ------------------------------------------------------------ | ------- |
| 1    | Make the draft GUI for Industrial Robotics exercises work. Implement a new *industrial_wrapper* node to connect our GUI with MoveIt, and to build a new Industrial Robotics API. We'll develop the infrastructure similarly to what we did with our [drone_wrapper](https://github.com/JdeRobot/drones) for drone exercises | DONE    |
| 2    | Test the modified Gazebo grasp fix grasp plugin with rectangular, cylindrical and spherical parts. Test also irregularly shaped objects. Record and upload a demo video. If it doesn't work, see Note below. | DONE    |
| 3    | Prepare a new Gazebo gripper model for ABB IRB120 for a parallel two-finger gripper based on our lab [Schunk KGG gripper](https://schunk.com/es_en/gripping-systems/series/kgg/) (CAD files will be uploaded shortly) | PENDING |

**Note:** Is [this Gazebo set of tools](https://github.com/jsbruglie/grasp)  for grasping a good alternative?

#### [WEEK 1-2.  June 1 - June 11](https://theroboticsclub.github.io/colab-gsoc2020-Yijia_Wu/2020-06-11-week1_2/)

|      | Tasks                                                        | State |
| ---- | ------------------------------------------------------------ | ----- |
| 1    | Prepare a first draft of the *Graphical User Interface* for Industrial Robotics exercises. The complete GUI specifications are given in [this separate file](../exercises_description/Exercises_GUI_description.md) | DONE  |
| 2    | Check available [ROS RQT plugins](http://wiki.ros.org/rqt/Plugins#Robot-interaction_tools_.28rqt_robot_plugins.29), to see which ones could be useful for our GUI | DONE  |
| 3    | Work on the [Gazebo grasp fix plugin from Jennifer Buehler](https://github.com/jenniferBuehler/gazebo-pkgs/wiki/The-Gazebo-grasp-fix-plugin), exploring the way of fine-tuning it to make object grasping more reliable | DONE  |
| 4    | *Extra:* Spend some time to prepare finals                   | DONE  |

**Important:** *remember to upload a blog post sometime before each weekly meeting*

