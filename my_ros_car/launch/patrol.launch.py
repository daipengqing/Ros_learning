#!/usr/bin/env python3
# =============================================================================
# patrol.launch.py — 自主巡检系统一键启动
# =============================================================================
# 运行位置: Docker 容器 (dddmr_x64_navigation, ROS 2 Humble)
# 前提条件:
#   1. 宿主机已启动 gazebo.launch.py (仿真 + 机器人)
#   2. 已用 SLAM 建图并保存了地图
# 功能: 启动 Nav2 导航栈 + 巡检节点, 机器人自动按预设航点巡逻
# =============================================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成巡检系统启动描述"""

    pkg_share = FindPackageShare("my_ros_car")

    # =========================================================================
    # 启动参数
    # =========================================================================
    map_yaml_file = LaunchConfiguration("map")
    declare_map_yaml = DeclareLaunchArgument(
        "map",
        default_value=PathJoinSubstitution([pkg_share, "maps", "my_map.yaml"]),
        description="预建地图文件路径",
    )

    patrol_points_file = LaunchConfiguration("patrol_points_file")
    declare_patrol_points = DeclareLaunchArgument(
        "patrol_points_file",
        default_value=PathJoinSubstitution([pkg_share, "config", "patrol_points.yaml"]),
        description="巡检航点配置文件路径",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
    )

    # =========================================================================
    # 导航栈 — 复用 navigation.launch.py
    # =========================================================================
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "navigation.launch.py"])
        ),
        launch_arguments={
            "map": map_yaml_file,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # =========================================================================
    # 巡检节点 — 按预设航点自动巡逻
    # =========================================================================
    patrol_node = Node(
        package="my_ros_car",
        executable="patrol_node.py",
        name="patrol_node",
        output="screen",
        parameters=[
            patrol_points_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    # =========================================================================
    # 返回 LaunchDescription
    # =========================================================================
    return LaunchDescription([
        declare_map_yaml,
        declare_patrol_points,
        declare_use_sim_time,
        navigation_launch,
        patrol_node,
    ])
