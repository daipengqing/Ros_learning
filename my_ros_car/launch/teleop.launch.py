#!/usr/bin/env python3
"""
Keyboard teleop launch file.
在单独的终端中运行此文件来控制小车运动：

    ros2 launch my_ros_car teleop.launch.py

控制方式:
    w/s: 前进/后退
    a/d: 左转/右转
    q/z: 加速/减速
    x:   停止
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="teleop_twist_keyboard",
            executable="teleop_twist_keyboard",
            output="screen",
            remappings=[
                ("/cmd_vel", "/diff_drive_controller/cmd_vel"),
            ],
        ),
    ])
