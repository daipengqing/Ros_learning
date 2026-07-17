#!/usr/bin/env python3
"""中继节点：将 Twist 消息转换为 TwistStamped，转发给 diff_drive_controller"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistRelay(Node):
    def __init__(self):
        super().__init__("twist_relay")
        # 订阅 teleop_twist_keyboard 发布的 Twist
        self.sub = self.create_subscription(Twist, "/cmd_vel", self.twist_cb, 10)
        # 发布 TwistStamped 给 diff_drive_controller
        self.pub = self.create_publisher(TwistStamped, "/diff_drive_controller/cmd_vel", 10)
        self.get_logger().info("TwistRelay ready: /cmd_vel (Twist) → /diff_drive_controller/cmd_vel (TwistStamped)")

    def twist_cb(self, msg: Twist):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = "base_link"
        stamped.twist = msg
        self.pub.publish(stamped)


def main():
    rclpy.init()
    node = TwistRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
