## Weekly tasks 

#### WEEK 5.  June 27 - July 3

|      | Tasks                                                        | State   |
| ---- | ------------------------------------------------------------ | ------- |
| 1    | Check and update the way the *grasp quality* parameter in the grasp message is computed for the pick() method. | ONGOING |
| 2    | Update GUI to include *joints angle limits* read from yaml file. Show  those limits in GUI. | ONGOING |
| 3    | Refine proposed API to include three *approach and retreat* options for grasping, and its related input parameters: <br />a) **vertical** (yaw angle at grasp point, distance from approach/retreat point) <br/>b) **horizontal** (roll angle at grasp point, distance from appr./retreat point)<br />c) **user defined** (roll, pitch, yaw angles at grasp point, distances from approach/retreat point, vectors for approach/retreat) | ONGOING |
| 4    | Complete the new *Exercise 1: pick and place with industrial robotic manipulator*. Finish Gazebo world. Prepare documentation. Record a demo video | ONGOING |

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

