import sys
import os
import resources_rc
from PyQt5.QtWidgets import QMainWindow, QApplication, QLineEdit
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import *
from gui.form import Ui_MainWindow
from gui.widgets.cameraWidget import CameraWidget
from gui.widgets.logoWidget import LogoWidget
import numpy


class MainWindow(QMainWindow, Ui_MainWindow):

    updGUI=pyqtSignal()
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self.camera1=CameraWidget(self)

        self.start_button.clicked.connect(self.playClicked)
        #self.start_button.setCheckable(True)

        self.stop_button.clicked.connect(self.stopClicked)
        self.updGUI.connect(self.updateGUI)

        self.jointSlider_1.sliderReleased.connect(self.setjoint1)
        self.jointSlider_1.setMinimum(-165)
        self.jointSlider_1.setMaximum(165)

        self.jointSlider_2.sliderReleased.connect(self.setjoint2)
        self.jointSlider_2.setMinimum(-110)
        self.jointSlider_2.setMaximum(110)

        self.jointSlider_3.sliderReleased.connect(self.setjoint3)
        self.jointSlider_3.setMinimum(-110)
        self.jointSlider_3.setMaximum(70)

        self.jointSlider_4.sliderReleased.connect(self.setjoint4)
        self.jointSlider_4.setMinimum(-160)
        self.jointSlider_4.setMaximum(160)

        self.jointSlider_5.sliderReleased.connect(self.setjoint5)
        self.jointSlider_5.setMinimum(-120)
        self.jointSlider_5.setMaximum(120)

        self.jointSlider_6.sliderReleased.connect(self.setjoint6)
        self.jointSlider_6.setMinimum(-180)
        self.jointSlider_6.setMaximum(180)

        self.gripperSlider.sliderReleased.connect(self.setgripper)
        self.gripperSlider.setMinimum(0)
        self.gripperSlider.setMaximum(80)

        self.xEdit.editingFinished.connect(self.set_x)
        self.yEdit.editingFinished.connect(self.set_y)
        self.zEdit.editingFinished.connect(self.set_z)
        self.rollEdit.editingFinished.connect(self.set_roll)
        self.pitchEdit.editingFinished.connect(self.set_pitch)
        self.yawEdit.editingFinished.connect(self.set_yaw)

        self.planButton.clicked.connect(self.plan)
        self.executeButton.clicked.connect(self.execute)
        self.planexeButton.clicked.connect(self.planexe)
        self.stopexeButton.clicked.connect(self.stopexe)

        # self.start_button.setIcon(QIcon(':images/play.png'))
        # self.stop_button.setIcon(QIcon(':images/stop.png'))

        # pixmap = QPixmap(':images/jderobot.png')
        # self.logo.setPixmap(pixmap)
        # self.logo.setScaledContents(True)

    def plan(self):
        self.robot.plan()

    def execute(self):
        self.robot.execute()
    
    def planexe(self):
        self.robot.plan()
        self.robot.execute()

    def stopexe(self):
        self.robot.stop_execution()

    def set_x(self, text):
        self.robot.set_x(float(text))

    def set_y(self, text):
        self.robot.set_y(float(text))

    def set_z(self, text):
        self.robot.set_z(float(text))

    def set_roll(self, text):
        self.robot.set_roll(float(text))

    def set_pitch(self, text):
        self.robot.set_pitch(float(text))

    def set_yaw(self, text):
        self.robot.set_yaw(float(text))

    def playClicked(self):
        #print("start")
        #text = self.le.text()
        self.browser.append("start")
        #os.system("rosrun irb120_robotiq85_gazebo test_pick_and_place.py")
        self.pick_place.start()

    def stopClicked(self):
        #print("stop")
        self.browser.append("stop")
        #self.pick_place.stop()

    def updateGUI(self):
        #print 'update gui'
        self.camera1.updateImage()

    def setjoint1(self):
        angle = numpy.deg2rad(self.jointSlider_1.value())
        print(angle)
        self.robot.set_arm_joint(1, angle)

    def setjoint2(self):
        angle = numpy.deg2rad(self.jointSlider_2.value())
        #print(angle)
        self.robot.set_arm_joint(2, angle)

    def setjoint3(self):
        angle = numpy.deg2rad(self.jointSlider_3.value())
        #print(angle)
        self.robot.set_arm_joint(3, angle)

    def setjoint4(self):
        angle = numpy.deg2rad(self.jointSlider_4.value())
        #print(angle)
        self.robot.set_arm_joint(4, angle)

    def setjoint5(self):
        angle = numpy.deg2rad(self.jointSlider_5.value())
        #print(angle)
        self.robot.set_arm_joint(5, angle)

    def setjoint6(self):
        angle = numpy.deg2rad(self.jointSlider_6.value())
        #print(angle)
        self.robot.set_arm_joint(6, angle)

    def setgripper(self):
        self.robot.move_joint_hand(self.gripperSlider.value()/100.0)

    def getgrippervalue(self):
        return self.robot.get_gripper_joint_value()*100

    def setRobotWrapper(self, robot):
        self.robot = robot
        self.gripperSlider.setValue(self.getgrippervalue())

    def getRobotWrapper(self):
        return self.robot 

    def setPickPlace(self, pick_place):
        self.pick_place = pick_place

    def getCamera(self):
        return self.camera

    def setCamera(self,camera):
        self.camera=camera

    def setAlgorithm(self, algorithm ):
        self.algorithm=algorithm

    def getAlgorithm(self):
        return self.algorithm

    def closeEvent(self, event):
        #self.algorithm.kill()
        self.camera.stop()
        self.robot.stop()
        event.accept()


# app = QApplication(sys.argv)

# window = MainWindow()
# window.show()
# app.exec()
