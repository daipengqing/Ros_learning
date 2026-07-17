#!/usr/bin/env python3
# =============================================================================
# nav2_odom_relay.py — 里程计话题中继节点
# =============================================================================
# 运行位置: Docker 容器 (dddmr_x64_navigation, ROS 2 Humble)
# 功能: 将 /diff_drive_controller/odom 转发到 /odom
#
# 为什么需要: diff_drive_controller 默认发布里程计到
#   /diff_drive_controller/odom，而某些旧版 Nav2 组件可能硬编码了 /odom。
#   nav2_params.yaml 中已通过 odom_topic 参数配置了正确的 topic，
#   本节点作为备用方案，一般情况下不需要运行。
#
# 使用方法:
#   python3 nav2_odom_relay.py
#   或在 launch 中:
#     Node(package="my_ros_car", executable="nav2_odom_relay.py")
# =============================================================================

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry


class OdomRelay(Node):
    """里程计话题中继: /diff_drive_controller/odom → /odom"""

    def __init__(self):
        super().__init__("nav2_odom_relay")

        # 启用仿真时间
        self.set_parameters([
            Parameter("use_sim_time", Parameter.Type.BOOL, True)
        ])

        # 订阅 diff_drive_controller 发布的里程计
        self.sub = self.create_subscription(
            Odometry,
            "/diff_drive_controller/odom",
            self.odom_callback,
            10,
        )

        # 发布到 /odom（Nav2 传统默认话题名）
        self.pub = self.create_publisher(
            Odometry,
            "/odom",
            10,
        )

        self.get_logger().info(
            "OdomRelay ready: /diff_drive_controller/odom → /odom"
        )

    def odom_callback(self, msg: Odometry):
        """直接转发里程计消息"""
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = OdomRelay()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
