import sys
import os
from PyQt5.QtWidgets import *
from PyQt5 import uic
from goyoung_msgs.msg import Mode
from goyoung_msgs.msg import Savefile
from goyoung_msgs.msg import Checkpoint
current_file_path = os.path.abspath(__file__)
current_directory = os.path.dirname(current_file_path)
ui_file = current_directory + "/page1.ui"

form_secondwindow = uic.loadUiType(ui_file)[0]

class SecondWindow(QDialog, form_secondwindow):
    def __init__(self, ros_node_wrapper):
        super(SecondWindow, self).__init__()
        self.ros_node_wrapper = ros_node_wrapper
        self.path = (os.environ.get('HOME') or os.environ.get('USERPROFILE')) + '/ros2_ws/src/Goyoung/goyoung_motion/save_file/'
        self.initUI()
        # self.show()
        self.toggle_fullscreen = QAction("Toggle Fullscreen", self)
        self.toggle_fullscreen.setShortcut("F5")
        self.toggle_fullscreen.triggered.connect(self.toggleFullScreen)
        self.addAction(self.toggle_fullscreen)
        
        if self.ros_node_wrapper.node.gui_full_screen:
            self.showFullScreen()
        # self.showFullScreen()

    def toggleFullScreen(self):
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()
        
    def initUI(self):
        self.setupUi(self)
        self.record_button.clicked.connect(self.record)
        self.check_interrupt.clicked.connect(self.checkpoint)
        self.save_button.clicked.connect(self.Home)

    def checkpoint(self):
        check_ = Checkpoint()
        check_.check = True
        self.ros_node_wrapper.node.motion_check_pub.publish(check_)
        
    def record(self):
        file_name = self.path + self.filename.toPlainText()
        right1 = self.Right_1.isChecked()
        right2 = self.Right_2.isChecked()
        right3 = self.Right_3.isChecked()
        right4 = self.Right_4.isChecked()
        left1 = self.Left_1.isChecked()
        left2 = self.Left_2.isChecked()
        left3 = self.Left_3.isChecked()
        left4 = self.Left_4.isChecked()
        chest = self.Chest.isChecked()
        mode = Mode()
        mode.file = file_name
        mode.mode = 1
        mode.motor = [right1, right2, right3, right4, left1, left2, left3, left4, chest]
        self.ros_node_wrapper.node.mode_pub.publish(mode)
    
    def Home(self):
        mode = Mode()
        mode.mode = 0
        save_flag = Savefile()
        save_flag.save = True
        self.ros_node_wrapper.node.save_pub.publish(save_flag)
        self.ros_node_wrapper.node.mode_pub.publish(mode)
        self.close()