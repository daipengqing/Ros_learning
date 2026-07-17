#!/usr/bin/env python3
# =============================================================================
# patrol_node.py — 自主巡检节点
# =============================================================================
# 运行位置: Docker 容器 (dddmr_x64_navigation, ROS 2 Humble)
# 依赖: nav2_simple_commander (Python API)
# 功能: 循环遍历预设航点，自动导航并执行巡检任务
#
# 使用 Nav2 Simple Commander API 简化导航操作:
#   - navigator.goToPose(pose)  → 导航到单个目标点
#   - navigator.goThroughPoses(poses) → 按航点序列导航
#   - navigator.cancelTask()    → 取消当前任务
#
# 工作流程:
#   1. 等待 Nav2 启动完成
#   2. 设置初始位姿
#   3. 循环: 发送航点 → 等待到达 → 停留 → 下一个航点
# =============================================================================

import math
import time
import yaml
import os

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class PatrolNode(Node):
    """自主巡检节点 —— 按预设航点循环巡逻"""

    def __init__(self):
        super().__init__("patrol_node")

        # ---- 声明参数（从 patrol_points.yaml 加载）----
        self.declare_parameter("patrol_mode", "loop")
        self.declare_parameter("wait_at_point_sec", 3.0)
        self.declare_parameter("navigation_timeout_sec", 60.0)
        self.declare_parameter("max_retries", 3)

        # ---- 读取航点列表 ----
        # waypoints 参数是一个嵌套数组，格式: [[x, y, yaw], label]
        # 但 ROS 2 YAML 参数加载不支持嵌套数组，所以我们改用文件读取
        self.waypoints = self._load_waypoints()

        # ---- 初始化 Nav2 Simple Commander ----
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 导航栈已激活，巡检节点就绪")

        # ---- 状态变量 ----
        self.current_point_idx = 0
        self.direction = 1  # 1=正向, -1=反向 (往返模式用)

    def _load_waypoints(self):
        """从 patrol_points.yaml 加载航点列表"""
        # 尝试从参数服务器获取航点文件路径
        try:
            # 方式 1: 从参数获取
            pkg_share = os.environ.get("MY_ROS_CAR_SHARE", "")
            if not pkg_share:
                # 尝试从 install 路径推断
                from ament_index_python.packages import get_package_share_directory
                pkg_share = get_package_share_directory("my_ros_car")

            config_path = os.path.join(pkg_share, "config", "patrol_points.yaml")

            if os.path.exists(config_path):
                with open(config_path, "r") as f:
                    data = yaml.safe_load(f)
                    patrol_data = data.get("patrol", {}).get("ros__parameters", data.get("patrol", {}))

                    # 也尝试更新参数
                    self._update_params_from_dict(patrol_data)

                    waypoints_raw = patrol_data.get("waypoints", [])
                    waypoints = []
                    for wp in waypoints_raw:
                        if isinstance(wp, dict) and "pose" in wp:
                            waypoints.append({
                                "pose": wp["pose"],  # [x, y, yaw]
                                "label": wp.get("label", "未命名航点"),
                            })
                    return waypoints
        except Exception as e:
            self.get_logger().warn(f"从文件加载航点失败: {e}")

        # 方式 2: 使用默认航点
        self.get_logger().warn("使用默认航点（原点附近正方形巡逻）")
        return [
            {"pose": [0.0, 0.0, 0.0], "label": "起点"},
            {"pose": [2.0, 0.0, 0.0], "label": "东"},
            {"pose": [2.0, 2.0, 1.57], "label": "东北"},
            {"pose": [0.0, 2.0, 3.14], "label": "西北"},
        ]

    def _update_params_from_dict(self, data: dict):
        """从 YAML 加载的参数更新节点参数值"""
        key_map = {
            "patrol_mode": "patrol_mode",
            "wait_at_point_sec": "wait_at_point_sec",
            "navigation_timeout_sec": "navigation_timeout_sec",
            "max_retries": "max_retries",
        }
        for yaml_key, param_name in key_map.items():
            if yaml_key in data:
                try:
                    self.set_parameters([
                        Parameter(param_name, Parameter.Type.DOUBLE, float(data[yaml_key]))
                        if isinstance(data[yaml_key], (int, float))
                        else Parameter(param_name, Parameter.Type.STRING, str(data[yaml_key]))
                    ])
                except Exception:
                    pass

    def start_patrol(self):
        """开始巡检循环"""
        mode = self.get_parameter("patrol_mode").get_parameter_value().string_value
        wait_sec = self.get_parameter("wait_at_point_sec").get_parameter_value().double_value
        timeout_sec = self.get_parameter("navigation_timeout_sec").get_parameter_value().double_value
        max_retries = self.get_parameter("max_retries").get_parameter_value().integer_value

        wp_count = len(self.waypoints)
        if wp_count == 0:
            self.get_logger().error("没有定义航点！请在 patrol_points.yaml 中配置。")
            return

        self.get_logger().info(f"=" * 60)
        self.get_logger().info(f"开始巡检! 模式={mode}, 航点={wp_count}, 停留={wait_sec}s")
        self.get_logger().info(f"=" * 60)

        # ---- 首先设置初始位姿 ----
        self._set_initial_pose(self.waypoints[0])

        # ---- 主循环 ----
        while rclpy.ok():
            # 获取当前目标航点
            wp = self.waypoints[self.current_point_idx]
            label = wp.get("label", f"航点{self.current_point_idx}")
            pose_arr = wp["pose"]

            self.get_logger().info(
                f"→ 导航到 [{self.current_point_idx + 1}/{wp_count}]: "
                f"{label} (x={pose_arr[0]:.2f}, y={pose_arr[1]:.2f}, yaw={pose_arr[2]:.2f})"
            )

            # ---- 创建目标位姿 ----
            goal_pose = self._make_pose(pose_arr)

            # ---- 导航到目标点 ----
            success = self._navigate_with_retry(goal_pose, timeout_sec, max_retries)

            if success:
                self.get_logger().info(f"✓ 到达 {label}，停留 {wait_sec}s 执行巡检...")
                # 模拟巡检任务（拍照、测温、读数等）
                self._do_inspection(label)
                time.sleep(wait_sec)
            else:
                self.get_logger().warn(f"✗ 无法到达 {label}，跳过...")
                # 让行为树有时间清理状态
                time.sleep(2.0)

            # ---- 更新下一个航点索引 ----
            if mode == "round_trip":
                # 往返模式: 1→2→3→2→1→2→3...
                if self.direction == 1 and self.current_point_idx >= wp_count - 1:
                    self.direction = -1  # 到头了，返程
                elif self.direction == -1 and self.current_point_idx <= 0:
                    self.direction = 1   # 回到起点，重新正向
                self.current_point_idx += self.direction
            else:
                # 循环模式: 1→2→3→1→2→3...
                self.current_point_idx = (self.current_point_idx + 1) % wp_count

    def _set_initial_pose(self, wp: dict):
        """设置 AMCL 的初始位姿估计"""
        pose_arr = wp["pose"]
        init_pose = PoseWithCovarianceStamped()
        init_pose.header.frame_id = "map"
        init_pose.header.stamp = self.get_clock().now().to_msg()
        init_pose.pose.pose.position.x = float(pose_arr[0])
        init_pose.pose.pose.position.y = float(pose_arr[1])
        # yaw → quaternion
        yaw = float(pose_arr[2])
        init_pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
        init_pose.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # 设置协方差（初始估计的不确定性）
        init_pose.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,    # x 方差 0.25m²
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,     # y 方差 0.25m²
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.068,    # yaw 方差 ~0.068rad²
        ]

        self.navigator.setInitialPose(init_pose)
        self.get_logger().info("初始位姿已设置")

    def _navigate_with_retry(self, goal_pose: PoseStamped, timeout_sec: float, max_retries: int) -> bool:
        """导航到目标点，支持失败重试"""
        for attempt in range(1, max_retries + 1):
            if attempt > 1:
                self.get_logger().info(f"  重试 {attempt}/{max_retries}...")

            self.navigator.goToPose(goal_pose)

            # 等待导航完成
            start_time = time.time()
            while not self.navigator.isTaskComplete():
                # 检查超时
                if time.time() - start_time > timeout_sec:
                    self.get_logger().warn(f"  导航超时 ({timeout_sec}s)")
                    self.navigator.cancelTask()
                    break

                # 获取反馈
                feedback = self.navigator.getFeedback()
                if feedback:
                    # 每隔一段距离打印进度
                    remaining = feedback.distance_remaining
                    if int(remaining * 10) % 10 == 0:  # 每米打印一次（粗略）
                        pass  # 避免刷屏，可以取消注释下方日志
                        # self.get_logger().info(f"  剩余距离: {remaining:.2f}m")

                rclpy.spin_once(self, timeout_sec=0.1)

            # 检查结果
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                return True
            elif result == TaskResult.CANCELED:
                self.get_logger().warn("  导航被取消")
            elif result == TaskResult.FAILED:
                self.get_logger().warn("  导航失败")
            else:
                self.get_logger().warn(f"  未知结果: {result}")

        return False

    def _make_pose(self, pose_arr: list) -> PoseStamped:
        """将 [x, y, yaw] 数组转换为 PoseStamped 消息"""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(pose_arr[0])
        pose.pose.position.y = float(pose_arr[1])
        # yaw → quaternion
        yaw = float(pose_arr[2])
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _do_inspection(self, label: str):
        """执行巡检任务（占位函数，可扩展为拍照、测温、数据记录等）"""
        self.get_logger().info(f"  🔍 正在检查 [{label}] ...")
        # TODO: 在这里添加实际的巡检逻辑:
        #   - 调用摄像头拍照: self._capture_image()
        #   - 读取环境传感器: self._read_temperature()
        #   - 检查设备状态: self._check_equipment()
        #   - 记录巡检日志: self._log_inspection_result()
        self.get_logger().info(f"  ✅ [{label}] 巡检完成")


def main():
    rclpy.init()
    node = PatrolNode()

    try:
        node.start_patrol()
    except KeyboardInterrupt:
        node.get_logger().info("巡检被用户中断")
    except Exception as e:
        node.get_logger().error(f"巡检异常: {e}")
    finally:
        node.navigator.cancelTask()
        node.navigator.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
