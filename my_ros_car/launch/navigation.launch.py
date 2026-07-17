#!/usr/bin/env python3
# =============================================================================
# navigation.launch.py — Navigation2 完整导航栈启动文件
# =============================================================================
# 运行位置: Docker 容器 (dddmr_x64_navigation, ROS 2 Humble)
# 前提条件:
#   1. 宿主机已启动 gazebo.launch.py (仿真 + 机器人)
#   2. 已用 SLAM 建图并保存了 .pgm/.yaml 地图文件
# 功能: 加载地图 → AMCL 定位 → 路径规划 → 局部控制 → 行为恢复
# =============================================================================

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成 Nav2 完整导航栈启动描述"""

    pkg_share = FindPackageShare("my_ros_car")

    # =========================================================================
    # 启动参数
    # =========================================================================
    params_file = LaunchConfiguration("params_file")
    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([pkg_share, "config", "nav2_params.yaml"]),
        description="Nav2 参数文件的完整路径",
    )

    map_yaml_file = LaunchConfiguration("map")
    declare_map_yaml = DeclareLaunchArgument(
        "map",
        default_value=PathJoinSubstitution([pkg_share, "maps", "my_map.yaml"]),
        description="预建地图 .yaml 文件的完整路径",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="使用仿真时钟",
    )

    autostart = LaunchConfiguration("autostart")
    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="自动激活所有 Nav2 生命周期节点",
    )

    use_rviz = LaunchConfiguration("use_rviz")
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="是否自动启动 RViz2 (true/false)",
    )

    # =========================================================================
    # 设置日志级别环境变量（减少无关输出）
    # =========================================================================
    set_log_level = SetEnvironmentVariable("RCUTILS_LOGGING_SEVERITY_THRESHOLD", "INFO")

    # =========================================================================
    # Map Server — 加载预先保存的地图
    # =========================================================================
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            params_file,
            {"use_sim_time": use_sim_time},
            {"yaml_filename": map_yaml_file},
        ],
    )

    # =========================================================================
    # AMCL — 自适应蒙特卡洛定位
    # =========================================================================
    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            params_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    # =========================================================================
    # Planner Server — 全局路径规划 (Smac Hybrid-A*)
    # =========================================================================
    planner_server_node = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[
            params_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    # =========================================================================
    # Controller Server — 局部路径跟随 (Regulated Pure Pursuit)
    # =========================================================================
    controller_server_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[
            params_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    # =========================================================================
    # Behavior Server — 恢复行为 (spin / backup / wait)
    # =========================================================================
    behavior_server_node = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[
            params_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    # =========================================================================
    # Velocity Smoother — 速度平滑器
    # =========================================================================
    velocity_smoother_node = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[
            params_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    # =========================================================================
    # BT Navigator — 行为树导航器
    # =========================================================================
    bt_navigator_node = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            params_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    # =========================================================================
    # Waypoint Follower — 航点跟随器（巡检用）
    # =========================================================================
    waypoint_follower_node = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[
            params_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    # =========================================================================
    # Lifecycle Manager — 自动管理所有节点的生命周期
    # =========================================================================
    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": [
                "map_server",
                "amcl",
                "planner_server",
                "controller_server",
                "behavior_server",
                "velocity_smoother",
                "bt_navigator",
                "waypoint_follower",
            ]},
        ],
    )

    # =========================================================================
    # RViz2 可视化 — 显示地图、路径、代价地图、机器人模型
    # =========================================================================
    rviz_config = PathJoinSubstitution([pkg_share, "rviz", "navigation.rviz"])
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
        # 环境配置
        set_log_level,
        # 启动参数
        declare_params_file,
        declare_map_yaml,
        declare_use_sim_time,
        declare_autostart,
        declare_use_rviz,
        # 导航组件
        map_server_node,
        amcl_node,
        planner_server_node,
        controller_server_node,
        behavior_server_node,
        velocity_smoother_node,
        bt_navigator_node,
        waypoint_follower_node,
        # 生命周期管理（最后启动）
        lifecycle_manager_node,
        # RViz 可视化
        rviz_node,
    ])
