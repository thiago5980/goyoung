import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    debug_param = LaunchConfiguration(
        'debug_param',
        default=os.path.join(
            get_package_share_directory('goyoung_main'),
            'params',
            'setting.yaml'
        )
    )
    return LaunchDescription([
        Node(
            package='goyoung_audio',
            executable='goyoung_audio',
            name='goyoung_audio',
            # output='screen',
        ),
        Node(
            package='goyoung_tcp',
            executable='tcp_server',
            name='tcp_server',
            # output='screen',
        ),
        Node(
            package='goyoung_can',
            executable='motor_can_node',
            name='motor_can_node',
            parameters=[debug_param],
            # output='screen',
        ),
        Node(
            package='goyoung_main',
            executable='goyoung_main',
            name='goyoung_main',
            parameters=[debug_param],
            # output='screen',
        ),
        Node(
            package='goyoung_gui',
            executable='goyoung_gui',
            name='goyoung_gui',
            parameters=[debug_param],
            # output='screen',
        ),
        Node(
            package='goyoung_motion',
            executable='motion_execute',
            name='motion_execute',
            # output='screen',
        ),
        Node(   
            package='goyoung_motion',
            executable='motion_record',
            name='motion_record',
            # output='screen',
        ),

    ])