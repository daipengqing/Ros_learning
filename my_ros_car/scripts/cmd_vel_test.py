#!/usr/bin/env python3
"""持续发送 TwistStamped 速度指令，使用 sim time 测试小车运动"""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped


class CmdVelTest(Node):
    def __init__(self):
        super().__init__("cmd_vel_test")
        # 关键：必须启用 sim time，否则时间戳与仿真不匹配
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.pub = self.create_publisher(TwistStamped, "/diff_drive_controller/cmd_vel", 10)
        self.count = 0
        self.started = False

        # 等 sim time 可用后再开始发布
        self.timer = self.create_timer(0.05, self.timer_cb)

    def timer_cb(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            # sim time 还没同步，等待
            if self.count == 0:
                print("Waiting for sim time (/clock)...")
            self.count += 1
            return

        if not self.started:
            self.started = True
            print(f"Sim time ready: {now.nanoseconds/1e9:.1f}s, starting to publish...")

        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = 0.5
        msg.twist.angular.z = 0.0
        self.pub.publish(msg)
        self.count += 1

        if self.count % 100 == 0:
            print(f"Sent {self.count} msgs, stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}, vx=0.5")


def main():
    rclpy.init()
    node = CmdVelTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStopped.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
