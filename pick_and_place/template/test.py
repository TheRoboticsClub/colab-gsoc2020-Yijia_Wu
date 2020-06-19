# General imports
import sys

# Practice imports
from gui.loadui import MainWindow
from gui.threadGUI import ThreadGUI
from MyAlgorithm import MyAlgorithm
from PyQt5.QtWidgets import QApplication
from interfaces.camera import ListenerCamera
from interfaces.motors import PublisherMotors
from interfaces.robot_wrapper import RobotWrapper
from interfaces.pick_and_place import Pick_Place

if __name__ == "__main__":

    camera = ListenerCamera("/camera/rgb/image_raw")
    #motors = PublisherMotors("/F1ROS/cmd_vel", 4, 0.3)
    robot = RobotWrapper()
    pick_place = Pick_Place(robot.arm, robot.gripper)
    #algorithm=MyAlgorithm(camera, motors)

    app = QApplication(sys.argv)
    myGUI = MainWindow()
    myGUI.setCamera(camera)
    myGUI.setRobotWrapper(robot)
    myGUI.setPickPlace(pick_place)
    # myGUI.setMotors(motors)
    #myGUI.setAlgorithm(algorithm)
    myGUI.show()


    t2 = ThreadGUI(myGUI)
    t2.daemon=True
    t2.start()


    sys.exit(app.exec_())