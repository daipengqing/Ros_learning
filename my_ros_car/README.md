# 🤖 my_ros_car — 四轮差速机器人 ROS 2 + Gazebo + Nav2 仿真包

基于 **ROS 2 Lyrical** + **Gazebo Harmonic 10.x** 的四轮差速驱动机器人仿真项目，实现从 **键盘遥控 → SLAM 建图 → Nav2 自主导航 → 自动巡检** 的完整链路。

## 🏗️ 混合架构

```
宿主机 (Ubuntu, ROS 2 Lyrical)           Docker 容器 (ROS 2 Humble)
┌──────────────────────────┐            ┌──────────────────────────────┐
│ Gazebo Harmonic 仿真      │            │ slam_toolbox (SLAM 建图)      │
│  • 物理引擎 / 传感器       │   DDS      │ nav2 (AMCL + 规划 + 控制)    │
│  • ros2_control 控制器     │◄─────────►│ patrol_node (自主巡检)       │
│  • /scan /clock /odom /tf │  通信      │ RViz2 (可视化)               │
└──────────────────────────┘            └──────────────────────────────┘
         │                                        │
         └─── 数据卷映射: src/my_ros_car/ ─────────┘
```

**为什么这样设计**: Lyrical (2026.05) 是最新版 ROS 2，slam_toolbox 和 nav2 尚未发布适配版本；Docker 容器运行 Humble 提供完整的导航算法栈。两边通过 DDS 跨环境通信。

## 📁 项目结构

```
my_ros_car/
├── README.md                           # 本文件
├── CMakeLists.txt                      # CMake 构建配置
├── package.xml                         # ROS 2 包描述
├── CODE_GUIDE.md                       # 新手代码阅读手册（含调试记录）
├── differential_robot_workflow.html    # 开发全景图（可浏览器打开）
│
├── urdf/                               # 机器人 Xacro 模型
│   ├── robot.urdf.xacro                # 总装（全局参数 + include 所有模块）
│   ├── robot_base.urdf.xacro           # 底盘 link + base_footprint
│   ├── wheels.urdf.xacro               # 4 个驱动轮 link + joint
│   ├── sensors.urdf.xacro              # LiDAR / IMU / Camera link
│   ├── gazebo.xacro                    # Gazebo 物理/传感器插件
│   └── gazebo_control.xacro            # ros2_control 硬件接口
│
├── launch/                             # 启动文件
│   ├── gazebo.launch.py                # 【宿主机】启动 Gazebo + 机器人 + ros_gz_bridge
│   ├── robot.launch.py                 # 【宿主机】机器人 spawn + 控制器加载（嵌套）
│   ├── slam.launch.py                  # 【Docker】SLAM Toolbox + RViz2
│   ├── navigation.launch.py            # 【Docker】Nav2 完整导航栈 + RViz2
│   ├── patrol.launch.py                # 【Docker】导航 + 自主巡检
│   └── teleop.launch.py                # 已废弃（参考用）
│
├── config/                             # 配置文件
│   ├── controllers.yaml                # diff_drive + joint_state_broadcaster
│   ├── slam_toolbox.yaml               # 在线异步 SLAM 参数
│   ├── nav2_params.yaml                # Nav2 完整参数（~500行）
│   └── patrol_points.yaml              # 巡检航点列表
│
├── scripts/                            # Python 脚本
│   ├── twist_relay.py                  # Twist → TwistStamped 消息中继
│   ├── cmd_vel_test.py                 # 测试脚本（让小车持续前进）
│   ├── patrol_node.py                  # 自主巡检节点（Nav2 Simple Commander）
│   └── nav2_odom_relay.py              # 里程计话题中继（备用）
│
├── rviz/                               # RViz2 配置文件
│   ├── slam.rviz                       # SLAM 可视化配置
│   └── navigation.rviz                 # 导航可视化配置
│
├── worlds/                             # Gazebo 世界文件
│   ├── simple_office.world             # 简单办公室（4 面墙 + 障碍物）
│   └── myHome.world                    # 住宅环境
│
└── maps/                               # 保存的地图文件
    └── my_map.yaml / my_map.pgm        # （建图后生成）
```

## 🤖 机器人参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 底盘尺寸 | 0.40 × 0.32 × 0.10 m | L × W × H |
| 底盘质量 | 8.0 kg | |
| 轮子半径 | 0.05 m | |
| 轮宽 | 0.03 m | |
| 单轮质量 | 0.40 kg | |
| 左右轮距 | 0.34 m | |
| 前后轴距 | 0.28 m | |
| 驱动方式 | 四轮差速 (4WD) | 同侧两轮同速 |
| 传感器 | 360° LiDAR (10Hz) / IMU (100Hz) / RGB Camera (30fps) | |

## 🚀 快速开始

### 前置条件

- 宿主机: ROS 2 Lyrical + Gazebo Harmonic 10.x + `ros_gz_sim` + `ros_gz_bridge`
- Docker: 容器 `dddmr_x64_navigation`（ROS 2 Humble，已安装 slam_toolbox + nav2 完整栈）
- 宿主机源码目录已通过 `-v` 挂载到容器，两边实时同步

### 构建

```bash
# === 宿主机 ===
cd ~/dddmr_navigation
source /opt/ros/lyrical/setup.bash
colcon build --packages-select my_ros_car --symlink-install --install-base install_host
source install_host/setup.bash

# === Docker 容器 ===
docker exec -it dddmr_x64_navigation bash
cd ~/dddmr_navigation
source /opt/ros/humble/setup.bash
rm -rf build/my_ros_car                           # 清理宿主机残留
colcon build --packages-select my_ros_car --symlink-install --install-base install_docker
source install_docker/setup.bash
```

> **注意**: 宿主机用 `install_host/`，Docker 用 `install_docker/`，互不污染。

### 阶段 1: 启动仿真 + 键盘控制

```bash
# === 宿主机 终端1: 启动 Gazebo ===
cd ~/dddmr_navigation
source install_host/setup.bash
ros2 launch my_ros_car gazebo.launch.py

# === 宿主机 终端2: 消息中继（Twist → TwistStamped）===
python3 ~/dddmr_navigation/src/my_ros_car/scripts/twist_relay.py

# === 宿主机 终端3: 键盘控制 ===
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# 按键: i 前进, , 后退, j 左转, l 右转, k 停车
```

### 阶段 2: SLAM 建图

```bash
# === 宿主机: 仿真 + 键盘控制保持运行 ===

# === Docker 容器: 启动建图 ===
docker exec -it dddmr_x64_navigation bash
cd ~/dddmr_navigation
source install_docker/setup.bash
ros2 launch my_ros_car slam.launch.py
# RViz2 自动启动，显示实时地图

# 键盘控制机器人走遍环境后，保存地图（Docker 内）
ros2 run nav2_map_server map_saver_cli -f ~/dddmr_navigation/src/my_ros_car/maps/my_map
```

### 阶段 3: Nav2 自主导航

```bash
# === 宿主机: 重启仿真（Ctrl+C 后）===
ros2 launch my_ros_car gazebo.launch.py

# === Docker: 启动导航 ===
ros2 launch my_ros_car navigation.launch.py map:=/path/to/my_map.yaml
# 在 RViz 中用 "2D Goal Pose" 工具点击目标位置
```

### 阶段 4: 自动巡检

```bash
# === 宿主机: 仿真保持运行 ===

# === Docker: 一键启动巡检 ===
ros2 launch my_ros_car patrol.launch.py
# 机器人自动按 patrol_points.yaml 中定义的航点循环巡逻
```

## 🔧 常用诊断命令

```bash
# 查看所有话题
ros2 topic list

# 确认控制器状态（必须是 active）
ros2 control list_controllers

# 查看里程计
ros2 topic echo /diff_drive_controller/odom --once

# 确认激光雷达有数据
ros2 topic echo /scan --once

# 查看 TF 树
ros2 run tf2_tools view_frames

# 保存地图
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

# 跨环境通信诊断（Docker 内）
ros2 topic list          # 应能看到宿主机的 /scan /clock /tf 等
ros2 topic hz /scan      # 确认数据频率正常 (~10Hz)
```

## 🐛 调试记录

完整调试记录见 [CODE_GUIDE.md](CODE_GUIDE.md) 第 4 章。踩过的 8 个关键坑：

| # | 问题 | 根因 | 解决 |
|---|------|------|------|
| 1 | bridge 崩溃 | `@` 分隔符位置错误 | `/clock@ROS[gz` 格式 |
| 2 | 控制器找不到 type | `type` 在 `ros__parameters` 外部 | 移入 `ros__parameters` 内 |
| 3 | 不支持的参数 | Lyrical API 变更 | 删除 `use_stamped_vel` 等 |
| 4 | 仿真无响应 | 插件文件名不含 `lib` 前缀 | `gz_ros2_control-system` |
| 5 | 控制器不工作 | 缺 `controller_manager` 段 | 添加 `update_rate` 参数 |
| 6 | spawner 缺参数 | 没传 `--param-file` | 添加 `-p controllers.yaml` |
| 7 | 启动顺序竞态 | TimerAction 并发 | 改用 OnProcessExit 事件驱动 |
| 8 | 轮转车不动 | 底盘擦地 + 轴反向 | 修改 joint z 偏移和 axis |
| 9 | `/scan` 缺失 | 传感器需 bridge 桥接 | ros_gz_bridge 添加 LaserScan |
| 10 | Docker/宿主机 install 冲突 | 共享 build 目录 | 独立 `install_host/` `install_docker/` |

## 📚 参考

- [CODE_GUIDE.md](CODE_GUIDE.md) — 新手代码阅读手册
- [differential_robot_workflow.html](differential_robot_workflow.html) — 开发全景图（浏览器打开）
- [ROS 2 Control 文档](https://control.ros.org/)
- [Gazebo Harmonic 文档](https://gazebosim.org/docs)
- [Navigation2 文档](https://docs.nav2.org/)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)

## 📄 许可证

MIT License

## 👤 维护者

Langzi0805 — [GitHub: daipengqing](https://github.com/daipengqing)
