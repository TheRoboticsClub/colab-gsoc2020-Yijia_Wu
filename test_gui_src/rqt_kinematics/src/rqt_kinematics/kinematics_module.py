import os
import rospy
import rospkg
import numpy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding import QtGui
from python_qt_binding.QtCore import pyqtSignal
from interfaces.robot_wrapper import RobotWrapper
import threading
import time
import resources_rc


class Kinematics(Plugin):
    def __init__(self, context):
        super(Kinematics, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Kinematics')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        self.event = threading.Event()

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_kinematics'), 'resources', 'KinematicsPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.robot = RobotWrapper()
        self.start_pick_place = False

        self._widget.xEdit.editingFinished.connect(self.set_x)
        self._widget.yEdit.editingFinished.connect(self.set_y)
        self._widget.zEdit.editingFinished.connect(self.set_z)
        self._widget.rollEdit.editingFinished.connect(self.set_roll)
        self._widget.pitchEdit.editingFinished.connect(self.set_pitch)
        self._widget.yawEdit.editingFinished.connect(self.set_yaw)

        self._widget.planButton.clicked.connect(self.plan)
        self._widget.executeButton.clicked.connect(self.execute)
        self._widget.planexeButton.clicked.connect(self.planexe)
        self._widget.stopexeButton.clicked.connect(self.stopexe)
        self._widget.homeButton.clicked.connect(self.backtohome)

        self._widget.jointSlider_1.sliderReleased.connect(self.setjoint1)
        self._widget.jointSlider_1.valueChanged.connect(self.viewjoint1)
        self._widget.jointSlider_1.setMinimum(-165)
        self._widget.jointSlider_1.setMaximum(165)
        self._widget.joint1Browser.append(str(round(numpy.rad2deg(self.robot.get_joints_value(1)),2)))

        self._widget.jointSlider_2.sliderReleased.connect(self.setjoint2)
        self._widget.jointSlider_2.valueChanged.connect(self.viewjoint2)
        self._widget.jointSlider_2.setMinimum(-110)
        self._widget.jointSlider_2.setMaximum(110)
        self._widget.joint2Browser.append(str(round(numpy.rad2deg(self.robot.get_joints_value(2)),2)))

        self._widget.jointSlider_3.sliderReleased.connect(self.setjoint3)
        self._widget.jointSlider_3.valueChanged.connect(self.viewjoint3)
        self._widget.jointSlider_3.setMinimum(-110)
        self._widget.jointSlider_3.setMaximum(70)
        self._widget.joint3Browser.append(str(round(numpy.rad2deg(self.robot.get_joints_value(3)),2)))

        self._widget.jointSlider_4.sliderReleased.connect(self.setjoint4)
        self._widget.jointSlider_4.valueChanged.connect(self.viewjoint4)
        self._widget.jointSlider_4.setMinimum(-160)
        self._widget.jointSlider_4.setMaximum(160)
        self._widget.joint4Browser.append(str(round(numpy.rad2deg(self.robot.get_joints_value(4)),2)))

        self._widget.jointSlider_5.sliderReleased.connect(self.setjoint5)
        self._widget.jointSlider_5.valueChanged.connect(self.viewjoint5)
        self._widget.jointSlider_5.setMinimum(-120)
        self._widget.jointSlider_5.setMaximum(120)
        self._widget.joint5Browser.append(str(round(numpy.rad2deg(self.robot.get_joints_value(5)),2)))

        self._widget.jointSlider_6.sliderReleased.connect(self.setjoint6)
        self._widget.jointSlider_6.valueChanged.connect(self.viewjoint6)
        self._widget.jointSlider_6.setMinimum(-180)
        self._widget.jointSlider_6.setMaximum(180)
        self._widget.joint6Browser.append(str(round(numpy.rad2deg(self.robot.get_joints_value(6)),2)))

        self._widget.gripperSlider.sliderReleased.connect(self.setgripper)
        self._widget.gripperSlider.valueChanged.connect(self.viewgripper)
        self._widget.gripperSlider.setMinimum(0)
        self._widget.gripperSlider.setMaximum(80)
        self._widget.gripperBrowser.append(str(round(self.robot.get_gripper_joint_value(),3)))

        self._widget.rvizButton.clicked.connect(self.launchrviz)
        self._widget.start_button.clicked.connect(self.playClicked)
        self._widget.stop_button.clicked.connect(self.stopexe)
        self._widget.stop_button.setCheckable(True)

        self._widget.updatefkButton.clicked.connect(self.updatefk)
        self._widget.updateikButton.clicked.connect(self.updateik)

        self.updatefk()
        self.updateik()

    def updatefk(self):
        self._widget.joint1Browser.clear()
        self._widget.joint1Browser.append(str(round(numpy.rad2deg(self.robot.get_joints_value(1)),2)))
        self._widget.jointSlider_1.setValue(numpy.rad2deg(self.robot.get_joints_value(1)))
        self._widget.joint2Browser.clear()
        self._widget.joint2Browser.append(str(round(numpy.rad2deg(self.robot.get_joints_value(2)),2)))
        self._widget.jointSlider_2.setValue(numpy.rad2deg(self.robot.get_joints_value(2)))
        self._widget.joint3Browser.clear()
        self._widget.joint3Browser.append(str(round(numpy.rad2deg(self.robot.get_joints_value(3)),2)))
        self._widget.jointSlider_3.setValue(numpy.rad2deg(self.robot.get_joints_value(3)))
        self._widget.joint4Browser.clear()
        self._widget.joint4Browser.append(str(round(numpy.rad2deg(self.robot.get_joints_value(4)),2)))
        self._widget.jointSlider_4.setValue(numpy.rad2deg(self.robot.get_joints_value(4)))
        self._widget.joint5Browser.clear()
        self._widget.joint5Browser.append(str(round(numpy.rad2deg(self.robot.get_joints_value(5)),2)))
        self._widget.jointSlider_5.setValue(numpy.rad2deg(self.robot.get_joints_value(5)))
        self._widget.joint6Browser.clear()
        self._widget.joint6Browser.append(str(round(numpy.rad2deg(self.robot.get_joints_value(6)),2)))
        self._widget.jointSlider_6.setValue(numpy.rad2deg(self.robot.get_joints_value(6)))
        self._widget.gripperBrowser.clear()
        self._widget.gripperBrowser.append(str(round(self.robot.get_gripper_joint_value(),3)))
        self._widget.gripperSlider.setValue(self.robot.get_gripper_joint_value()*100)

    def updateik(self):
        x, y, z = self.robot.get_arm_position()
        roll, pitch, yaw = self.robot.get_arm_orientation()
        self._widget.xEdit.setText(str(round(x,4)))
        self._widget.yEdit.setText(str(round(y,4)))
        self._widget.zEdit.setText(str(round(z,4)))
        self._widget.rollEdit.setText(str(round(roll,4)))
        self._widget.pitchEdit.setText(str(round(pitch,4)))
        self._widget.yawEdit.setText(str(round(yaw,4)))

    def launchrviz(self):
        os.system("gnome-terminal -x sh -c \"roslaunch rqt_kinematics rviz.launch\"")

    def playClicked(self):
        self._widget.browser.append("start")
        self.event.set()
        self.t1 = threading.Thread(target = self.robot.pick_and_place, 
                                    args = (self.event,))
        self.t2 = threading.Thread(target = self.stopChecked)
        self.t1.start()
        self.t2.start()
        self.start_pick_place = True

    def stopChecked(self):
        while self.event.isSet():
            if self._widget.stop_button.isChecked():
                self.event.clear()
                break

    def plan(self):
        self.robot.plan()

    def execute(self):
        self.robot.execute()
        self.updatefk()
        self.updateik()
    
    def planexe(self):
        self.robot.plan()
        self.robot.execute()
        self.updatefk()
        self.updateik()

    def stopexe(self):
        self.robot.stop_execution()
        
        if self.start_pick_place == True:
            self.event.clear()
            self.t1.join()
            self.t2.join()
            self._widget.browser.append("stop")
            self._widget.stop_button.toggle()
            self.start_pick_place = False

        self.updatefk()
        self.updateik()
        # cursor = self._widget.browser.textCursor()
        # cursor.movePosition(QtGui.QTextCursor.End)
        # self._widget.browser.setTextCursor(cursor)
        # self._widget.browser.ensureCursorVisible() 

    def backtohome(self):
        self.robot.back_to_home()
        self.updatefk()
        self.updateik()

    def set_x(self):
        self.robot.set_x(float(self._widget.xEdit.text()))

    def set_y(self):
        self.robot.set_y(float(self._widget.yEdit.text()))

    def set_z(self):
        self.robot.set_z(float(self._widget.zEdit.text()))

    def set_roll(self):
        self.robot.set_roll(float(self._widget.rollEdit.text()))

    def set_pitch(self):
        self.robot.set_pitch(float(self._widget.pitchEdit.text()))

    def set_yaw(self):
        self.robot.set_yaw(float(self._widget.yawEdit.text()))

    def viewjoint1(self):
        self._widget.joint1Browser.clear()
        self._widget.joint1Browser.append(str(self._widget.jointSlider_1.value()))

    def viewjoint2(self):
        self._widget.joint2Browser.clear()
        self._widget.joint2Browser.append(str(self._widget.jointSlider_2.value()))

    def viewjoint3(self):
        self._widget.joint3Browser.clear()
        self._widget.joint3Browser.append(str(self._widget.jointSlider_3.value()))

    def viewjoint4(self):
        self._widget.joint4Browser.clear()
        self._widget.joint4Browser.append(str(self._widget.jointSlider_4.value()))

    def viewjoint5(self):
        self._widget.joint5Browser.clear()
        self._widget.joint5Browser.append(str(self._widget.jointSlider_5.value()))

    def viewjoint6(self):
        self._widget.joint6Browser.clear()
        self._widget.joint6Browser.append(str(self._widget.jointSlider_6.value()))

    def viewgripper(self):
        self._widget.gripperBrowser.clear()
        self._widget.gripperBrowser.append(str(self._widget.gripperSlider.value()/100.0))

    def setjoint1(self):
        self._widget.joint1Browser.clear()
        self._widget.joint1Browser.append(str(self._widget.jointSlider_1.value()))
        angle = numpy.deg2rad(self._widget.jointSlider_1.value())
        self.robot.set_arm_joint(1, angle)
        self.updateik()

    def setjoint2(self):
        self._widget.joint2Browser.clear()
        self._widget.joint2Browser.append(str(self._widget.jointSlider_2.value()))
        angle = numpy.deg2rad(self._widget.jointSlider_2.value())
        self.robot.set_arm_joint(2, angle)
        self.updateik()

    def setjoint3(self):
        self._widget.joint3Browser.clear()
        self._widget.joint3Browser.append(str(self._widget.jointSlider_3.value()))
        angle = numpy.deg2rad(self._widget.jointSlider_3.value())
        self.robot.set_arm_joint(3, angle)
        self.updateik()

    def setjoint4(self):
        self._widget.joint4Browser.clear()
        self._widget.joint4Browser.append(str(self._widget.jointSlider_4.value()))
        angle = numpy.deg2rad(self._widget.jointSlider_4.value())
        self.robot.set_arm_joint(4, angle)
        self.updateik()

    def setjoint5(self):
        self._widget.joint5Browser.clear()
        self._widget.joint5Browser.append(str(self._widget.jointSlider_5.value()))
        angle = numpy.deg2rad(self._widget.jointSlider_5.value())
        self.robot.set_arm_joint(5, angle)
        self.updateik()

    def setjoint6(self):
        self._widget.joint6Browser.clear()
        self._widget.joint6Browser.append(str(self._widget.jointSlider_6.value()))
        angle = numpy.deg2rad(self._widget.jointSlider_6.value())
        self.robot.set_arm_joint(6, angle)
        self.updateik()

    def setgripper(self):
        self._widget.gripperBrowser.clear()
        self._widget.gripperBrowser.append(str(self._widget.gripperSlider.value()/100.0))
        self.robot.move_joint_hand(self._widget.gripperSlider.value()/100.0)

    def getgrippervalue(self):
        return self.robot.get_gripper_joint_value()*100

    def setRobotWrapper(self, robot):
        self.robot = robot
        #self.gripperSlider.setValue(self.getgrippervalue())

    def getRobotWrapper(self):
        return self.robot 