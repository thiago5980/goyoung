import sys
import os
import yaml

from PyQt5.QtWidgets import *
from PyQt5 import uic
from goyoung_msgs.msg import Mode

current_file_path = os.path.abspath(__file__)
current_directory = os.path.dirname(current_file_path)
ui_file = current_directory + "/page3.ui"

form_thirdwindow = uic.loadUiType(ui_file)[0]

class FourthWindow(QDialog, form_thirdwindow):
    def __init__(self, ros_node_wrapper):
        super(FourthWindow, self).__init__()
        self.yaml_path = (os.environ.get('HOME') or os.environ.get('USERPROFILE')) + '/ros2_ws/src/Goyoung/goyoung_motion/save_file/'
        self.save_path = (os.environ.get('HOME') or os.environ.get('USERPROFILE')) + '/ros2_ws/src/Goyoung/goyoung_motion/save_file/'
        self.ros_node_wrapper = ros_node_wrapper
        self.initUI()
        # self.show()
        self.toggle_fullscreen = QAction("Toggle Fullscreen", self)
        self.toggle_fullscreen.setShortcut("F5")
        self.toggle_fullscreen.triggered.connect(self.toggleFullScreen)
        self.addAction(self.toggle_fullscreen)
        
        # self.showFullScreen()
        if self.ros_node_wrapper.node.gui_full_screen:
            self.showFullScreen()
            
        self._action = []
        self._action_index = 0
        self.sequence = []
        
        
    def toggleFullScreen(self):
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()
            
    def initUI(self):
        self.setupUi(self)
        self.play_button.clicked.connect(self.play)
        self.save_button.clicked.connect(self.Home)
        self.save_time.clicked.connect(self.SaveTime)
        
    def play(self):
        file_name = self.yaml_path + self.filename.toPlainText()
        self.save_path = self.save_path + self.filename.toPlainText()
        base, extension = self.save_path.rsplit('.', 1)
        self.save_path = base + '_time.' + extension
        if self._action_index == 0:
            self._action = self.getYaml(file_name)
            self.Number.setText(str(self._action[self._action_index]))
            
    def Home(self):
        if (self.sequence == []):
            self.close()
            return
        
        yaml_data = {
        "size": len(self.sequence),
        "sequences": self.sequence
        }
        
        with open(self.save_path, 'w') as file:
            yaml.dump(yaml_data, file, default_flow_style=False)
        
        self.sequence = []
        self._action = []
        self._action_index = 0
        self.close()

    def SaveTime(self):
        # print(self._action_index)
        
        self.sequence.append({'action':self._action[self._action_index], 'start_time': float(self.S_time.toPlainText()), 'end_time': float(self.E_time.toPlainText())})
        self.S_time.clear()
        self.E_time.clear()
        self._action_index += 1
        
        if self._action_index >= len(self._action):
            self.Number.setText("Finish")
            return
        
        self.Number.setText(str(self._action[self._action_index]))
        
    
    def getYaml(self, yaml_path):
        with open(yaml_path, 'r') as file:
            documents = yaml.safe_load_all(file)
            actions = []

            for data in documents:
                sequences = data.get('sequences', [])
                for sequence_dict in sequences:
                    action = sequence_dict.get('action')
                    if action is not None:
                        actions.append(int(action))
        return actions
    
    


            