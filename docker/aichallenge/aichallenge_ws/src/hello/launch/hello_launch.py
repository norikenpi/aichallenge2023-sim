# \\192.168.128.181\aichallenge2023-sim\docker\aichallenge\aichallenge_ws\src\aichallenge_submit\autoware_universe_launch\tier4_control_launch\launch\control.launch.py
# 上記のlaunchファイルに同じこと書いてある。

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="hello",
            namespace="hello",
            executable="hello_node",
        )
    ])