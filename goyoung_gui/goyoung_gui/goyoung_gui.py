import sys
import threading
import os
import time
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import *
from PyQt5.QtGui import *

import rclpy
from rclpy.node import Node
from goyoung_msgs.msg import Mode

current_file_path = os.path.abspath(__file__)
current_directory = os.path.dirname(current_file_path)
if current_directory not in sys.path:
    sys.path.append(current_directory)
from page1 import SecondWindow
from page2 import ThirdWindow
from page4 import FifthWindow
from page3 import FourthWindow
from goyoung_ros import RosNodeWrapper

os.chmod((os.environ.get('HOME') or os.environ.get('USERPROFILE')) + '/ros2_ws/src/Goyoung/goyoung_gui/goyoung_gui/goyoung_gui.py', 0o777)

ui_file = current_directory + "/goyoung_gui.ui"
form_class = uic.loadUiType(ui_file)[0]

global ros_thread
global ros_node_wrapper

class WindowClass(QMainWindow, form_class):
    def __init__(self, ros_node_wrapper, is_main_window=False):
        super().__init__()
        self.ros_node_wrapper = ros_node_wrapper
        self.is_main_window = is_main_window
        self.debug = self.ros_node_wrapper.node.debug # debug 모드에 따라 로봇의 표정 동작 여부 결정 (True: 모션 저장 및 테스트 GUI 실행, False: 표정 동작 O)
        self.gui_full_screen = self.ros_node_wrapper.node.gui_full_screen
        print("Debug mode:", self.debug)
        self.initUI()
        self.isFullScreen = self.gui_full_screen
        if self.isFullScreen:
            self.showFullScreen()
        if not self.debug:
            self.robotFunction()

    def initUI(self):
        self.setupUi(self)
        self.motion_generate.clicked.connect(self.generateFunction)
        self.motion_play.clicked.connect(self.playFunction)
        self.time_save.clicked.connect(self.saveFunction)
        self.robot_execute.clicked.connect(self.robotFunction)
        self.toggle_fullscreen = QAction("Toggle Fullscreen", self)
        self.toggle_fullscreen.setShortcut("F5")
        self.toggle_fullscreen.triggered.connect(self.toggleFullScreen)
        self.addAction(self.toggle_fullscreen)

    def toggleFullScreen(self):
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()

    def generateFunction(self):
        self.is_main_window = False
        self.close()
        self.second = SecondWindow(self.ros_node_wrapper)
        self.second.exec_()
        self.show()
        self.is_main_window = True

    def playFunction(self):
        self.is_main_window = False
        self.close()
        self.third = ThirdWindow(self.ros_node_wrapper)
        self.third.exec_()
        self.show()
        self.is_main_window = True

    def saveFunction(self):
        self.is_main_window = False
        self.close()
        self.fourth = FourthWindow(self.ros_node_wrapper)
        self.fourth.exec_()
        self.show()
        self.is_main_window = True

    def robotFunction(self):
        self.is_main_window = False
        self.close()
        self.fourth = FifthWindow(self.ros_node_wrapper)
        self.fourth.exec_()
        self.show()
        mode = Mode()
        mode.mode = 0
        self.ros_node_wrapper.node.mode_pub.publish(mode)
        self.is_main_window = True

    def closeEvent(self, event):
        if self.is_main_window and self.ros_node_wrapper.node.shutdown_flag == False:
            print("closeEvent")
            self.ros_node_wrapper.node.shutdown_flag = True
        super().closeEvent(event)

def run_ros_node():
    global ros_node_wrapper
    ros_node_wrapper = RosNodeWrapper()
    ros_node_wrapper.spin()

def main(args=None):
    global ros_thread
    ros_thread = threading.Thread(target=run_ros_node, daemon=True)
    ros_thread.start()
    
    app = QApplication(sys.argv)
    myWindow = WindowClass(ros_node_wrapper, is_main_window=True)
    myWindow.show()
    exit_code = app.exec_()

    if ros_thread.is_alive():
        ros_thread.join()
        
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
