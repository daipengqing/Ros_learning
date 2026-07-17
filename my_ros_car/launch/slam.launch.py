#!/usr/bin/env python3
# =============================================================================
# slam.launch.py — SLAM Toolbox 在线建图启动文件
# =============================================================================
# 运行位置: Docker 容器 (dddmr_x64_navigation, ROS 2 Humble)
# 前提条件: 宿主机已启动 gazebo.launch.py (仿真 + 机器人)
# 功能: 启动 async_slam_toolbox_node + RViz2 可视化
# =============================================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成 SLAM 启动描述"""

    pkg_share = FindPackageShare("my_ros_car")

    # =========================================================================
    # 启动参数
    # =========================================================================
    slam_params_file = LaunchConfiguration("slam_params_file")
    declare_slam_params_file = DeclareLaunchArgument(
        "slam_params_file",
        default_value=PathJoinSubstitution([pkg_share, "config", "slam_toolbox.yaml"]),
        description="SLAM Toolbox 参数文件的完整路径",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="使用仿真时钟",
    )

    use_rviz = LaunchConfiguration("use_rviz")
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="是否自动启动 RViz2 (true/false)",
    )

    # =========================================================================
    # SLAM Toolbox 节点 — 异步在线建图
    # =========================================================================
    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    # =========================================================================
    # RViz2 可视化 — 显示地图、激光、TF
    # =========================================================================
    rviz_config = PathJoinSubstitution([pkg_share, "rviz", "slam.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    # =========================================================================
    # 返回 LaunchDescription
    # =========================================================================
    return LaunchDescription([
        declare_slam_params_file,
        declare_use_sim_time,
        declare_use_rviz,
        slam_toolbox_node,
        rviz_node,
    ])
