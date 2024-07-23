import sys
import os
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import QUrl, QTimer, pyqtSlot
from PyQt5.QtMultimedia import QMediaPlayer, QMediaContent
from PyQt5.QtMultimediaWidgets import QVideoWidget
from PyQt5.QtGui import QImage, QPixmap
import cv2
from goyoung_msgs.msg import Mode

current_file_path = os.path.abspath(__file__)
current_directory = os.path.dirname(current_file_path)
ui_file = current_directory + "/page4.ui"

form_thirdwindow = uic.loadUiType(ui_file)[0]

class FifthWindow(QDialog, form_thirdwindow):
    def __init__(self, ros_node_wrapper):
        super(FifthWindow, self).__init__()
        self.ros_node_wrapper = ros_node_wrapper

        self.initUI()
        self.toggle_fullscreen = QAction("Toggle Fullscreen", self)
        self.toggle_fullscreen.setShortcut("F5")
        self.toggle_fullscreen.triggered.connect(self.toggleFullScreen)
        self.addAction(self.toggle_fullscreen)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.cap = cv2.VideoCapture("/home/goyoung/ros2_ws/src/Goyoung/goyoung_gui/data/basic.mp4")
        self.timer.start(30)  # 30 ms 간격으로 프레임 업데이트
        mode = Mode()
        mode.mode = 3
        self.ros_node_wrapper.node.mode_pub.publish(mode)
        
        self.ros_node_wrapper.video_signal.connect(self.change_video)
        if self.ros_node_wrapper.node.gui_full_screen:
            self.showFullScreen()

    def toggleFullScreen(self):
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()

    def initUI(self):
        self.setupUi(self)
        self.videoWidget = QVideoWidget(self)
        self.videoWidget.setGeometry(self.videolabel.geometry())

    def update_frame(self):
        ret, frame = self.cap.read()
        if ret:
            self.rgbImage = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.convertToQtFormat = QImage(self.rgbImage.data, self.rgbImage.shape[1], 
                                            self.rgbImage.shape[0], QImage.Format_RGB888)
            self.pixmap = QPixmap(self.convertToQtFormat)
            self.p = self.pixmap.scaled(self.videolabel.width(), self.videolabel.height())
            self.videolabel.setPixmap(self.p)
        else:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    @pyqtSlot(int)
    def change_video(self, video_number):
        video_files = {
            0: "/home/goyoung/ros2_ws/src/Goyoung/goyoung_gui/data/basic.mp4",
            1: "/home/goyoung/ros2_ws/src/Goyoung/goyoung_gui/data/on.mp4",
            2: "/home/goyoung/ros2_ws/src/Goyoung/goyoung_gui/data/smile.mp4",
            3: "/home/goyoung/ros2_ws/src/Goyoung/goyoung_gui/data/off.mp4",
        }
        self.video_path = video_files.get(video_number, "/home/goyoung/ros2_ws/src/Goyoung/goyoung_gui/data/basic.mp4")
        self.cap.release()
        self.cap = cv2.VideoCapture(self.video_path)
        self.timer.start(30)

    def Home(self):
        self.close()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = FifthWindow(None)
    window.show()
    sys.exit(app.exec_())
