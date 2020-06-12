import sys
import resources_rc
from PyQt5.QtWidgets import QMainWindow, QApplication, QLineEdit
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import *
from gui.form import Ui_MainWindow
from gui.widgets.cameraWidget import CameraWidget
from gui.widgets.logoWidget import LogoWidget

#camera

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

        # self.start_button.setIcon(QIcon(':images/play.png'))
        # self.stop_button.setIcon(QIcon(':images/stop.png'))

        # pixmap = QPixmap(':images/jderobot.png')
        # self.logo.setPixmap(pixmap)
        # self.logo.setScaledContents(True)

    def playClicked(self):
        #print("start")
        #text = self.le.text()
        self.browser.append("start")

    def stopClicked(self):
        #print("stop")
        self.browser.append("stop")

    def updateGUI(self):
        #print 'update gui'
        self.camera1.updateImage()

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
        event.accept()


# app = QApplication(sys.argv)

# window = MainWindow()
# window.show()
# app.exec()
