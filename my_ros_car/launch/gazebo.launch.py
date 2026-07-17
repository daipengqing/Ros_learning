#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ####################################################################
    # Launch Arguments
    ####################################################################

    world = LaunchConfiguration("world")

    declare_world = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution([
            FindPackageShare("my_ros_car"),
            "worlds",
            "myHome.world"
        ])
    )

    ####################################################################
    # Gazebo Sim
    ####################################################################

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ),
        launch_arguments={
            "gz_args": ["-r ", world]
        }.items()
    )

    ####################################################################
    # Robot
    ####################################################################

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("my_ros_car"),
                "launch",
                "robot.launch.py"
            ])
        )
    )

    ####################################################################
    # ROS-GZ Bridge (桥接 Gazebo 数据到 ROS 2)
    ####################################################################

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            # GZ→ROS: 仿真时钟（所有 use_sim_time 节点依赖）
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",

            # GZ→ROS: 激光雷达数据（slam_toolbox / nav2 依赖）
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",

            # GZ→ROS: IMU 数据（可选，nav2 未使用但保留以备扩展）
            # "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",

            # ROS→GZ: 速度指令（备用，实际控制走 ros2_control）
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        ]
    )

    ####################################################################
    # Launch
    ####################################################################

    return LaunchDescription([
        declare_world,
        gazebo,
        robot,
        bridge
    ])
