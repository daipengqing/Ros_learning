# 🚀 四轮差速机器人开发指南

基于 **ROS2 Lyrical (真机)** + **Gazebo 10.4.0** 的四轮差速机器人仿真项目

## 📋 项目架构

**混合架构设计**：
- **真机 (ROS2 Lyrical)**：运行 Gazebo 仿真
- **Docker容器 (ROS2 Humble)**：运行 SLAM 和 Navigation2 算法

这样设计的原因：
1. 真机性能更好，Gazebo仿真更流畅
2. Docker中Humble版本包更完整
3. 两个环境隔离，互不干扰

## 🗂️ 项目结构

```
my_ros_car/
├── CMakeLists.txt          # CMake构建配置
├── package.xml             # ROS2包描述文件
├── README.md               # 本文件
├── urdf/                   # URDF机器人模型
│   ├── robot_base.urdf.xacro   # 底盘定义
│   ├── wheels.urdf.xacro       # 轮子详细配置
│   ├── sensors.urdf.xacro      # 传感器定义
│   └── robot.urdf.xacro        # 完整机器人（包含所有子文件）
├── launch/                 # ROS2启动文件
│   ├── gazebo.launch.py        # 启动Gazebo仿真
│   └── robot.launch.py         # 完整机器人系统启动
├── config/                  # 配置文件
│   └── controllers.yaml        # ros2_control控制器配置
├── worlds/                  # Gazebo世界文件
│   └── simple_office.world     # 简单办公室环境
├── maps/                    # 地图目录（空）
└── src/                     # Python源代码（空）
```

## ✅ 步骤1：项目环境准备（已完成）

- [x] ROS2工作空间已存在：`~/dddmr_navigation/`
- [x] 创建功能包：`my_ros_car`
- [x] 配置package.xml依赖
- [x] 配置CMakeLists.txt
- [x] 创建目录结构（urdf, launch, config, worlds）

## 🔧 步骤2：Xacro机器人建模（进行中）

### 已创建的文件：

- [x] `urdf/robot_base.urdf.xacro` - 底盘基础模型
  - 底盘主体（box: 0.4m × 0.3m × 0.1m）
  - 四个连续旋转轮子
  - 轮距：0.34m
  - 轮半径：0.05m
  - 质量参数和惯性矩阵

- [x] `urdf/wheels.urdf.xacro` - 轮子装饰
  - 轮毂装饰（银色圆柱）
  - 四个轮子各一个

- [x] `urdf/sensors.urdf.xacro` - 传感器模型
  - 激光雷达（laser_link）
  - IMU惯性测量单元（imu_link）
  - 摄像头（camera_link，可选）

- [x] `urdf/robot.urdf.xacro` - 完整机器人
  - 包含所有子文件
  - Gazebo插件配置（差速驱动、IMU、激光雷达）
  - ros2_control配置
  - 材质定义

### 机器人参数总结：

| 参数 | 值 | 说明 |
|------|-----|------|
| 底盘长度 | 0.4m | X方向 |
| 底盘宽度 | 0.3m | Y方向 |
| 底盘高度 | 0.1m | Z方向 |
| 底盘质量 | 5.0kg | - |
| 轮半径 | 0.05m | 10cm直径 |
| 轮宽度 | 0.025m | - |
| 轮距 | 0.34m | 左右轮中心距 |
| 轮质量 | 1.0kg | 每个轮子 |

## 🗺️ 步骤3：Gazebo世界文件

- [x] `worlds/simple_office.world` - 简单办公室环境
  - 地面平面
  - 四面墙壁（8m × 8m房间）
  - 光照配置
  - 物理引擎配置（ODE）
  - 两个障碍物盒子

## ⚙️ 步骤4：机器人物理与插件配置

已在 `robot.urdf.xacro` 中完成：
- [x] Gazebo差速驱动插件
  - 左轮：left_front/left_rear_wheel_joint
  - 右轮：right_front/right_rear_wheel_joint
  - 发布odom话题和tf变换
- [x] IMU插件配置
- [x] 激光雷达插件配置（720个采样点，-π到π）

## 🎮 步骤5：ros2_control控制器配置

- [x] `config/controllers.yaml` - 控制器参数配置
  - 差速控制器（diff_cont）
    - 左右轮关节名称
    - 轮距：0.34m
    - 轮半径：0.05m
    - odom坐标系：odom → base_footprint
  - 关节状态广播器（joint_broad）

- [x] `launch/gazebo.launch.py` - Gazebo启动文件
  - 启动Gazebo服务器和GUI
  - 加载机器人URDF
  - 生成机器人实体
  - 自动加载控制器

- [x] `launch/robot.launch.py` - 完整机器人启动文件
  - 一键启动完整系统
  - 支持暂停启动
  - 环境变量配置

## 🗺️ 步骤6-8：SLAM与导航（待实现）

将在Docker容器（ROS2 Humble）中实现：
- [ ] SLAM建图（slam_toolbox）
- [ ] Navigation2导航栈
- [ ] 自主巡检系统

## 🚀 快速开始

### 1. 编译包

```bash
# 在真机上（ROS2 Lyrical）
source /opt/ros/lyrical/setup.bash
cd ~/dddmr_navigation
colcon build --packages-select my_ros_car
source install/setup.bash
```

### 2. 启动Gazebo仿真

```bash
# 方式1：启动Gazebo（包含机器人）
ros2 launch my_ros_car gazebo.launch.py

# 方式2：启动完整机器人系统
ros2 launch my_ros_car robot.launch.py

# 无GUI模式（服务器模式）
ros2 launch my_ros_car gazebo.launch.py gui:=false
```

### 3. 手动控制机器人

```bash
# 新终端，发送速度命令
ros2 topic pub /diff_cont/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.5}}" -1
```

### 4. 查看话题

```bash
# 查看odom里程计
ros2 topic echo /diff_cont/odom

# 查看激光雷达数据
ros2 topic echo /scan

# 查看IMU数据
ros2 topic echo /imu/data

# 查看关节状态
ros2 topic echo /joint_states
```

### 5. 查看TF树

```bash
# 安装tf2工具（如果还没有）
sudo apt install ros-${ROS_DISTRO}-tf2-tools

# 查看TF树
ros2 run tf2_tools view_frames
```

## 📝 后续工作

### Docker容器中的SLAM和导航（步骤6-8）

在Docker容器（ROS2 Humble）中：

1. **复制包到容器**
   ```bash
   docker cp ~/dddmr_navigation/src/my_ros_car dddmr_x64_gazebo:/ws_gz/src/
   ```

2. **在容器中编译**
   ```bash
   docker exec -it dddmr_x64_gazebo bash
   source /opt/ros/humble/setup.bash
   cd /ws_gz
   colcon build --packages-select my_ros_car
   source install/setup.bash
   ```

3. **配置DDS通信**（让真机和Docker能通信）

4. **实现步骤6-8**
   - SLAM建图
   - Navigation2配置
   - 自主巡检

## 🐛 调试技巧

### 检查控制器状态
```bash
ros2 control list_controllers
ros2 control list_hardware_components
```

### 查看Gazebo状态
```bash
gz stats  # 查看仿真统计信息
gz topic -l  # 查看Gazebo话题
```

### 重置仿真
```bash
gz reset  # 重置到初始状态
```

## 📚 参考资料

- [HTML指南](differential_robot_workflow.html) - 四轮差速机器人开发全景图
- [ROS2 Control文档](https://control.ros.org/)
- [Gazebo文档](https://gazebosim.org/docs)
- [Diff Drive Controller](https://github.com/ros-controls/ros2_controllers/blob/master/diff_drive_controller/README.md)

## 📄 许可证

MIT License

## 👤 维护者

Langzi0805 (1227059712@qq.com)
