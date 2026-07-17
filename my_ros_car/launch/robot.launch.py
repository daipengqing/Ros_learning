#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("my_ros_car")

    # 1. 解析 URDF
    urdf_path = PathJoinSubstitution([pkg_share, "urdf", "robot.urdf.xacro"])
    robot_description = ParameterValue(Command(["xacro ", urdf_path]), value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
        }]
    )

    # 2. 生成机器人
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "my_robot",
            "-x", "0", "-y", "0", "-z", "0.08",
            "-allow_renaming", "true",
        ]
    )

    # 3. 控制器加载（参考官方 demo，顺序启动）
    controllers_yaml = PathJoinSubstitution([pkg_share, "config", "controllers.yaml"])

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen"
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "-c", "/controller_manager",
            "--param-file", controllers_yaml,
        ],
        output="screen"
    )

    # 事件驱动顺序启动: spawn → broadcaster → diff_drive
    # 等 robot spawn 完成后再加载控制器
    load_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # 等 broadcaster 完成后再加载 diff_drive
    load_diff_drive = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    # 键盘遥控请在单独终端运行:
    #   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel
    # 或者在终端运行测试脚本:
    #   python3 /home/daipegqin/dddmr_navigation/src/my_ros_car/scripts/cmd_vel_test.py

    return LaunchDescription([
        robot_state_publisher,
        gz_spawn_entity,
        load_broadcaster,
        load_diff_drive,
    ])
