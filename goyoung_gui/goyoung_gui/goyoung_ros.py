import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from goyoung_msgs.msg import Mode, Shutdown, Savefile, Checkpoint
from std_msgs.msg import Int8
import subprocess
from PyQt5.QtCore import QObject, pyqtSignal

class ModeChange(Node):
    def __init__(self):
        super().__init__('mode_change')
        qos_profile = QoSProfile(depth=10)
        self.mode_pub = self.create_publisher(Mode, 'mode', qos_profile)
        self.save_pub = self.create_publisher(Savefile, 'save_file', qos_profile)
        self.motion_check_pub = self.create_publisher(Checkpoint, 'motion_checkpoint', qos_profile)
        self.shutdown_computer = self.create_subscription(Shutdown, 'shutdown_computer', self.sub_shutdown_msg, qos_profile) # shutdown_computer topic에 메시지가 올 때마다 sub_shutdown_msg 함수 호출
        self.declare_parameter('debug', False)
        self.debug = self.get_parameter('debug').value
        
        self.declare_parameter('full_screen', False)
        self.gui_full_screen = self.get_parameter('full_screen').value
        # self.change_video = self.create_subscription(Int8, 'change_video', self.sub_change_video, qos_profile)
        self.shutdown_flag = False

    # def sub_change_video(self, msg):
    #     self.get_logger().info('Received change video message')
    #     self.get_logger().info(f'Video number: {msg.data}')
    #     self.mode_pub.publish(Mode(mode=msg.data))
        
    def sub_shutdown_msg(self, msg): # shutdown_computer topic에 메시지가 올 때마다 호출되는 함수 (로봇 종료)
        self.get_logger().info('Received shutdown message')
        self.shutdown_flag = True
        if msg.shutdown:
            subprocess.call(["sudo", "shutdown", "-h", "now"])

class RosNodeWrapper(QObject):
    video_signal = pyqtSignal(int)

    def __init__(self):
        super().__init__()
        rclpy.init()
        sensor_qos_profile = QoSProfile(depth=10)
        self.node = ModeChange()
        self.node.create_subscription(Int8, 'change_video', self.sub_change_video, sensor_qos_profile)
        self.video_msg = 0

    def sub_change_video(self, msg): # goyoung_tcp ROS2 Node에서 받아온 video_number에 따라 video 변경
        print(f'Video number: {msg.data}')
        if self.video_msg != msg.data:
            self.video_signal.emit(msg.data)
            self.video_msg = msg.data

    def spin(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self.node.destroy_node()
            rclpy.shutdown()
