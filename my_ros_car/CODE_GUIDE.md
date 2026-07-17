# my_ros_car 代码阅读手册

> **适用环境：** ROS 2 Lyrical（2026年5月发布）+ Gazebo Harmonic 10.x  
> **适用读者：** ROS 2 / Gazebo 仿真初学者  
> **项目目标：** 实现一个四轮差速驱动（4WD）机器人，在 Gazebo 仿真中受键盘控制运动

---

## 目录

- [1. 项目总览：我的小车是怎么跑起来的](#1-项目总览我的小车是怎么跑起来的)
- [2. 文件导航](#2-文件导航)
- [3. 核心文件详解](#3-核心文件详解)
  - [3.1 robot.urdf.xacro —— 机器人的"总装图"](#31-roboturdfxacro--机器人的总装图)
  - [3.2 robot_base.urdf.xacro —— 底盘](#32-robot_baseurdfxacro--底盘)
  - [3.3 wheels.urdf.xacro —— 四个驱动轮](#33-wheelsurdfxacro--四个驱动轮)
  - [3.4 sensors.urdf.xacro —— 激光雷达、IMU、摄像头](#34-sensorsurdfxacro--激光雷达imu摄像头)
  - [3.5 gazebo.xacro —— Gazebo 物理/摩擦/传感器插件](#35-gazeboxacro--gazebo-物理摩擦传感器插件)
  - [3.6 gazebo_control.xacro —— ros2_control 控制桥梁](#36-gazebo_controlxacro--ros2_control-控制桥梁)
  - [3.7 controllers.yaml —— 控制器参数配置](#37-controllersyaml--控制器参数配置)
  - [3.8 gazebo.launch.py —— 顶层启动文件](#38-gazebolaunchpy--顶层启动文件)
  - [3.9 robot.launch.py —— 机器人启动 + 控制器加载](#39-robotlaunchpy--机器人启动--控制器加载)
  - [3.10 teleop.launch.py —— 键盘遥控（已废弃，保留作参考）](#310-teleoplaunchpy--键盘遥控已废弃保留作参考)
  - [3.11 cmd_vel_test.py —— 速度指令测试脚本](#311-cmd_vel_testpy--速度指令测试脚本)
  - [3.12 twist_relay.py —— Twist→TwistStamped 消息中继](#312-twist_relaypy--twisttwiststamped-消息中继)
  - [3.13 辅助文件](#313-辅助文件)
- [4. 调试全记录：踩过的 8 个坑](#4-调试全记录踩过的-8-个坑)
- [5. 完整数据流图](#5-完整数据流图)
- [6. 操作速查表](#6-操作速查表)
- [7. SLAM 建图系统 (slam_toolbox)](#7-slam-建图系统-slam_toolbox)
  - [7.1 slam_toolbox.yaml —— 建图参数](#71-slam_toolboxyaml--建图参数)
  - [7.2 slam.launch.py —— 建图启动](#72-slamlaunchpy--建图启动)
- [8. Navigation2 导航系统](#8-navigation2-导航系统)
  - [8.1 nav2_params.yaml —— 导航核心参数](#81-nav2_paramsyaml--导航核心参数)
  - [8.2 navigation.launch.py —— 导航启动](#82-navigationlaunchpy--导航启动)
- [9. 自主巡检系统](#9-自主巡检系统)
  - [9.1 patrol_node.py —— 巡检逻辑](#91-patrol_nodepy--巡检逻辑)
  - [9.2 patrol_points.yaml —— 航点配置](#92-patrol_pointsyaml--航点配置)
  - [9.3 patrol.launch.py —— 巡检启动](#93-patrollaunchpy--巡检启动)
- [10. 辅助脚本](#10-辅助脚本)
  - [10.1 nav2_odom_relay.py —— 里程计中继](#101-nav2_odom_relaypy--里程计中继)
- [11. Docker 跨环境通信详解](#11-docker-跨环境通信详解)

---

## 1. 项目总览：我的小车是怎么跑起来的

整个系统分为三层，每层各司其职：

```
┌──────────────────────────────────────────────────┐
│  第 1 层：指令层（人类控制）                        │
│  键盘 → teleop_twist_keyboard → /cmd_vel (Twist) │
│         twist_relay 转换 → TwistStamped           │
├──────────────────────────────────────────────────┤
│  第 2 层：控制层（ros2_control）                    │
│  diff_drive_controller                            │
│    └─ 接收 TwistStamped 速度指令                   │
│    └─ 通过差速运动学，算出每个轮子的转速              │
│    └─ 写入关节速度指令接口                          │
│  joint_state_broadcaster                          │
│    └─ 读取关节位置/速度，发布 /joint_states         │
├──────────────────────────────────────────────────┤
│  第 3 层：物理仿真层（Gazebo Harmonic）              │
│  gz_ros2_control 插件                             │
│    └─ 把 ros2_control 的指令转成 Gazebo 物理关节力    │
│    └─ 物理引擎计算碰撞、摩擦、重力...                │
└──────────────────────────────────────────────────┘
```

**核心流程（从键盘到车轮转动）：**

```
键盘按键 → Twist 消息(无时间戳) → twist_relay 添加时间戳
→ TwistStamped 消息 → diff_drive_controller
→ 运动学公式: wheel_vel = (linear ± angular * separation/2) / radius
→ 4个关节速度指令 → gz_ros2_control → Gazebo 物理引擎 → 轮子转动 → 小车移动
```

---

## 2. 文件导航

| 文件路径 | 作用 | 重要程度 |
|----------|------|---------|
| `urdf/robot.urdf.xacro` | 总装文件，include 所有模块、定义全局参数 | ⭐⭐⭐ |
| `urdf/robot_base.urdf.xacro` | 底盘 link 定义（形状、质量、惯性） | ⭐⭐ |
| `urdf/wheels.urdf.xacro` | 四个轮子的 link + joint，含驱动轴定义 | ⭐⭐⭐ |
| `urdf/sensors.urdf.xacro` | LiDAR、IMU、Camera 的 link 和 joint | ⭐ |
| `urdf/gazebo.xacro` | Gazebo 物理属性（摩擦、接触刚度）、传感器插件 | ⭐⭐ |
| `urdf/gazebo_control.xacro` | ros2_control 硬件接口 + gz_ros2_control 插件 | ⭐⭐⭐ |
| `config/controllers.yaml` | diff_drive_controller 和 joint_state_broadcaster 参数 | ⭐⭐⭐ |
| `launch/gazebo.launch.py` | **顶层启动文件**：启动 Gazebo + 机器人 + ros_gz_bridge | ⭐⭐⭐ |
| `launch/robot.launch.py` | 机器人子启动：URDF 解析、spawn、控制器加载顺序 | ⭐⭐⭐ |
| `launch/teleop.launch.py` | 键盘遥控（不建议用，见下文） | ⭐ |
| `scripts/cmd_vel_test.py` | 测试脚本：让小车持续向前跑 | ⭐⭐ |
| `scripts/twist_relay.py` | Twist→TwistStamped 消息格式转换 | ⭐⭐ |
| `CMakeLists.txt` | 构建规则（安装哪些文件到哪些目录） | ⭐ |
| `package.xml` | 包元信息、依赖声明 | ⭐ |
| `worlds/simple_office.world` | Gazebo 世界文件（地面、墙壁、障碍物） | ⭐ |

---

## 3. 核心文件详解

### 3.1 robot.urdf.xacro —— 机器人的"总装图"

**作用：** 这是机器人描述文件的入口，类似 C 语言的 `main()`。它定义全局尺寸/质量参数，然后 `include` 各个模块。

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro"
       name="my_ros_car">

    <!-- ============ 全局参数 ============ -->
    <!-- 这些参数在整个 URDF 中可用 ${xxx} 引用 -->
    <xacro:property name="base_length" value="0.40"/>    <!-- 底盘长 40cm -->
    <xacro:property name="base_width"  value="0.32"/>    <!-- 底盘宽 32cm -->
    <xacro:property name="base_height" value="0.10"/>    <!-- 底盘高 10cm -->
    <xacro:property name="base_mass"   value="8.0"/>     <!-- 底盘质量 8kg -->

    <xacro:property name="wheel_radius"    value="0.05"/>  <!-- 轮子半径 5cm -->
    <xacro:property name="wheel_width"     value="0.03"/>  <!-- 轮子宽度 3cm -->
    <xacro:property name="wheel_mass"      value="0.40"/>  <!-- 单轮质量 0.4kg -->

    <xacro:property name="wheel_separation" value="0.34"/> <!-- 左右轮距 34cm -->
    <xacro:property name="wheel_base"       value="0.28"/> <!-- 前后轴距 28cm -->

    <!-- ============ 加载模块 ============ -->
    <!-- xacro:include 有点像 C 的 #include -->
    <xacro:include filename="$(find my_ros_car)/urdf/robot_base.urdf.xacro"/>
    <xacro:include filename="$(find my_ros_car)/urdf/wheels.urdf.xacro"/>
    <xacro:include filename="$(find my_ros_car)/urdf/sensors.urdf.xacro"/>
    <xacro:include filename="$(find my_ros_car)/urdf/gazebo.xacro"/>
    <xacro:include filename="$(find my_ros_car)/urdf/gazebo_control.xacro"/>

    <!-- ============ 创建机器人 ============ -->
    <!-- 每个 <xacro:xxx/> 调用对应模块中定义的宏 -->
    <xacro:robot_base/>   <!-- 生成底盘 link -->
    <xacro:wheels/>       <!-- 生成 4 个轮子 link + joint -->
    <xacro:sensors/>      <!-- 生成传感器 link + joint -->
    <xacro:gazebo_config/>     <!-- 添加 Gazebo 物理/传感器插件 -->
    <xacro:gazebo_control/>    <!-- 添加 ros2_control 硬件接口 -->
</robot>
```

**关键概念：**
- **Xacro 宏** 类似模板/函数。`<xacro:macro name="wheel" params="...">` 定义，`<xacro:wheel .../>` 调用。
- **`$(find my_ros_car)`** 是 ROS 的路径查找语法，返回包所在目录。
- **全局参数** 在开头统一定义，修改一处即可影响所有用到的地方。

---

### 3.2 robot_base.urdf.xacro —— 底盘

**作用：** 定义机器人主体和 `base_footprint`（地面投影点，用于导航）。

```xml
<xacro:macro name="robot_base">

    <!-- base_footprint：地面投影点（导航用，物理上不可见） -->
    <link name="base_footprint"/>

    <joint name="base_footprint_joint" type="fixed">
        <parent link="base_link"/>
        <child link="base_footprint"/>
        <!-- z = -wheel_radius：投影到地面高度 -->
        <origin xyz="0 0 -${wheel_radius}" rpy="0 0 0"/>
    </joint>

    <!-- base_link：机器人主体（所有其他 link 的父级） -->
    <link name="base_link">
        <visual>      <!-- Gazebo 中能看到的样子 -->
            <geometry>
                <box size="${base_length} ${base_width} ${base_height}"/>
            </geometry>
            <material name="Blue">
                <color rgba="0.20 0.45 0.85 1.0"/>
            </material>
        </visual>

        <collision>   <!-- 碰撞检测用的形状（通常和 visual 一致） -->
            <geometry>
                <box size="${base_length} ${base_width} ${base_height}"/>
            </geometry>
        </collision>

        <inertial>    <!-- 质量和转动惯量（物理仿真必须） -->
            <mass value="${base_mass}"/>
            <!-- 箱体转动惯量公式: I = m * (边长² + 另一边长²) / 12 -->
            <inertia
                ixx="${base_mass*(base_width²+base_height²)/12.0}"
                iyy="${base_mass*(base_length²+base_height²)/12.0}"
                izz="${base_mass*(base_length²+base_width²)/12.0}"
                ixy="0" ixz="0" iyz="0"/>
        </inertial>
    </link>
</xacro:macro>
```

**关键概念：**
- `visual`, `collision`, `inertial` 是 URDF link 的三个重要子元素：
  - `visual`：渲染用的（不影响物理）
  - `collision`：碰撞检测用的（影响物理但不一定和 visual 一模一样）
  - `inertial`：质量和惯性张量（缺了这个，Gazebo 会报错或行为诡异）
- `base_footprint` 不是实体，它是导航栈需要的一个坐标 frame，投影在地面

---

### 3.3 wheels.urdf.xacro —— 四个驱动轮

**作用：** 用 xacro 宏 `<xacro:macro name="wheel">` 定义单个轮子的模板（link + joint），然后实例化 4 个轮子。这是本项目**改动最多、坑也最多**的文件。

```xml
<!-- ============ 单个轮子的宏模板 ============ -->
<xacro:macro name="wheel" params="prefix x y axis_xyz">
    <!-- 参数说明:
         prefix:   轮子名称前缀，如 "left_front"
         x, y:     轮子在底盘上的安装位置（相对 base_link）
         axis_xyz: 旋转轴方向，如 "0 1 0"（绕 Y 轴） -->

    <!-- 轮子 Link（物理实体） -->
    <link name="${prefix}_wheel_link">
        <visual>
            <!-- rpy 旋转 90° 是因为 URDF cylinder 默认轴向是 Z，
                 我们要让轮子绕 Y 轴转，所以把圆柱侧过来 -->
            <origin xyz="0 0 0" rpy="1.57079632679 0 0"/>
            <geometry>
                <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
            </geometry>
            <material name="Black">
                <color rgba="0.15 0.15 0.15 1.0"/>
            </material>
        </visual>

        <collision>
            <origin xyz="0 0 0" rpy="1.57079632679 0 0"/>
            <geometry>
                <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
            </geometry>
        </collision>

        <inertial>
            <origin xyz="0 0 0" rpy="1.57079632679 0 0"/>
            <mass value="${wheel_mass}"/>
            <inertia
                ixx="${wheel_mass*(3*wheel_radius²+wheel_width²)/12.0}"
                iyy="${0.5*wheel_mass*wheel_radius²}"
                izz="${wheel_mass*(3*wheel_radius²+wheel_width²)/12.0}"
                ixy="0" ixz="0" iyz="0"/>
        </inertial>
    </link>

    <!-- 轮子 Joint（连接底盘和轮子，定义旋转轴） -->
    <joint name="${prefix}_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="${prefix}_wheel_link"/>

        <!-- ⚠️ 这行经过了多次修改，见下方"坑点"详解 -->
        <origin xyz="${x} ${y} -0.03" rpy="0 0 0"/>

        <!-- 旋转轴方向 -->
        <axis xyz="${axis_xyz}"/>

        <!-- 关节阻尼（模拟轴承阻力），设为 0.1 让轮子不会无限加速 -->
        <dynamics damping="0.1" friction="0.0"/>
    </joint>
</xacro:macro>
```

**四个轮子的实例化：**

```xml
<xacro:macro name="wheels">
    <!-- 左侧两个轮子，y=+0.17（车身左侧），axis 都是 +Y -->
    <xacro:wheel prefix="left_front"  x="0.14"  y="0.17"  axis_xyz="0 1 0"/>
    <xacro:wheel prefix="left_rear"   x="-0.14" y="0.17"  axis_xyz="0 1 0"/>

    <!-- 右侧两个轮子，y=-0.17（车身右侧） -->
    <!-- ⚠️ axis 原来是 0 -1 0，后改为 0 1 0（见坑点 #8） -->
    <xacro:wheel prefix="right_front" x="0.14"  y="-0.17" axis_xyz="0 1 0"/>
    <xacro:wheel prefix="right_rear"  x="-0.14" y="-0.17" axis_xyz="0 1 0"/>
</xacro:macro>
```

---

#### ⚠️ 坑点 #8：轮子 joint z 偏移与轴方向

这是调试过程中**最隐蔽的物理问题**，分两部分：

**A) 底盘擦地（joint z 偏移）**

原始代码中轮子 joint z = 0：
```
底盘底部 z = -0.05 (base_height/2)
轮子中心 z = 0（与 base_link 同一高度）
轮子底部 z = -0.05 (0 - wheel_radius)

结论：底盘底部 == 轮子底部 → 底盘擦地！轮子空转，小车不动。
```

修复：`z="0"` → `z="-0.03"`，同步调整 spawn 高度 `z="0.0"` → `z="0.08"`。

修复后的几何关系：
```
底盘底部 z = -0.05
轮子中心 z = -0.03
轮子底部 z = -0.03 - 0.05 = -0.08
离地间隙 = 0.03m（3cm）
```

**B) 右轮反转（axis 方向）**

原始代码中右轮 axis = `0 -1 0`，左轮 axis = `0 1 0`。

```python
# diff_drive_controller 给左右轮写相同的速度值：
left_vel  = (v - ω * sep/2) / r   # 例如 10 rad/s
right_vel = (v + ω * sep/2) / r   # 也是 10 rad/s（直行时）

# 左轮 axis = +Y，正速度 → 向前转 ✓
# 右轮 axis = -Y，正速度 → 向后转 ✗  （原地打转！）
```

修复：右轮 axis 全部改为 `0 1 0`（与左轮一致）。

---

### 3.4 sensors.urdf.xacro —— 激光雷达、IMU、摄像头

**作用：** 定义 3 个传感器及其安装位置。目前只定义了物理结构（link + joint），Gazebo 传感器插件在 `gazebo.xacro` 中配置。

```xml
<xacro:macro name="sensors">
    <!-- LiDAR：装在底盘前上方 -->
    <joint name="laser_joint" type="fixed">
        <origin xyz="0.15 0 0.08"/>   <!-- 前 15cm，上 8cm -->
        ...
    </joint>

    <!-- IMU：装在底盘中心偏上 -->
    <joint name="imu_joint" type="fixed">
        <origin xyz="0 0 0.03"/>      <!-- 中心，上 3cm -->
        ...
    </joint>

    <!-- Camera：装在前上方 -->
    <joint name="camera_joint" type="fixed">
        <origin xyz="0.17 0 0.07"/>   <!-- 前 17cm，上 7cm -->
        ...
    </joint>

    <!-- Camera Optical Frame：ROS 相机数据标准帧 -->
    <joint name="camera_optical_joint" type="fixed">
        <origin rpy="-1.57 0 -1.57"/>  <!-- REP-103 相机光轴朝前 -->
        ...
    </joint>
</xacro:macro>
```

**关键概念：**
- 传感器的 link 都是 `fixed` joint 连接到 `base_link`，没有独立的运动
- `camera_optical_frame` 是 ROS 标准：相机数据用 `X→右, Y→下, Z→前`，而机器人用 `X→前, Y→左, Z→上`

---

### 3.5 gazebo.xacro —— Gazebo 物理/摩擦/传感器插件

**作用：** 提供 Gazebo 特有的配置（URDF 标准中没有的部分）：轮子摩擦、接触刚度、LiDAR/IMU/Camera 传感器插件。

```xml
<!-- 轮子的 Gazebo 物理配置 -->
<xacro:macro name="wheel_gazebo" params="wheel">
    <gazebo reference="${wheel}">
        <material>Gazebo/Black</material>
        <mu1>1.0</mu1>     <!-- 主摩擦系数（沿 fdir1 方向） -->
        <mu2>1.0</mu2>     <!-- 次摩擦系数（垂直 fdir1 方向） -->
        <kp>50000.0</kp>   <!-- 接触刚度（N/m），值越大越"硬" -->
        <kd>100.0</kd>     <!-- 接触阻尼，值越大越"黏" -->
        <fdir1>1 0 0</fdir1>  <!-- 主摩擦方向 = X 轴（前进方向） -->
    </gazebo>
</xacro:macro>

<!-- LiDAR 传感器插件 -->
<gazebo reference="laser_link">
    <sensor name="lidar_sensor" type="gpu_lidar">
        <update_rate>10</update_rate>   <!-- 10Hz 扫描频率 -->
        <topic>scan</topic>             <!-- 发布的 ROS 话题名 -->
        <lidar>
            <scan>
                <horizontal>
                    <samples>360</samples>        <!-- 每圈 360 个采样点 -->
                    <min_angle>-3.14159</min_angle>  <!-- -180° -->
                    <max_angle>3.14159</max_angle>   <!-- +180°（360° 扫描）-->
                </horizontal>
            </scan>
            <range>
                <min>0.1</min>   <!-- 最近探测距离 10cm -->
                <max>10.0</max>  <!-- 最远探测距离 10m -->
            </range>
        </lidar>
    </sensor>
</gazebo>

<!-- IMU 传感器插件 -->
<gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
        <update_rate>100</update_rate>    <!-- 100Hz -->
        <topic>imu/data</topic>           <!-- 发布话题 -->
    </sensor>
</gazebo>

<!-- Camera 传感器插件 -->
<gazebo reference="camera_link">
    <sensor name="camera_sensor" type="camera">
        <update_rate>30</update_rate>     <!-- 30fps -->
        <topic>camera/image_raw</topic>   <!-- 原始图像 -->
        <camera>
            <horizontal_fov>1.0472</horizontal_fov>  <!-- 60° 水平视场角 -->
            <image>
                <width>640</width>
                <height>480</height>
                <format>R8G8B8</format>    <!-- RGB 24bit -->
            </image>
            <clip>
                <near>0.05</near>    <!-- 近裁剪面 5cm -->
                <far>100.0</far>     <!-- 远裁剪面 100m -->
            </clip>
        </camera>
    </sensor>
</gazebo>
```

**关键概念：**
- `<gazebo reference="xxx">` 是 URDF 扩展，把 Gazebo 特有配置附加到某个 link 上
- `mu1`/`mu2` 决定轮子和地面的摩擦力，值太小轮子会打滑，值太大不真实
- `kp`/`kd` 是 Gazebo ODE 物理引擎的接触参数，影响"软硬度"——太小则像海绵，太大则像石头

---

### 3.6 gazebo_control.xacro —— ros2_control 控制桥梁

**作用：** 这是让 ros2_control 和 Gazebo 物理引擎对话的"翻译官"。包含两部分配置：

```xml
<xacro:macro name="gazebo_control">

    <!-- 第 1 部分：硬件接口定义 -->
    <!-- 告诉 ros2_control：机器人有哪些关节、每个关节提供什么接口 -->
    <ros2_control name="GazeboSystem" type="system">
        <hardware>
            <!-- 使用 gz_ros2_control 提供的仿真硬件插件 -->
            <plugin>gz_ros2_control/GazeboSimSystem</plugin>
        </hardware>

        <!-- 每个轮子关节：提供 velocity 指令接口 + position/velocity 状态接口 -->
        <joint name="left_front_wheel_joint">
            <command_interface name="velocity">
                <param name="min">-1</param>   <!-- 最小速度限制 (rad/s) -->
                <param name="max">1</param>     <!-- 最大速度限制 (rad/s) -->
            </command_interface>
            <state_interface name="position"/>   <!-- 关节角度反馈 -->
            <state_interface name="velocity"/>   <!-- 关节速度反馈 -->
        </joint>
        <!-- ... 其余 3 个轮子同理 ... -->
    </ros2_control>

    <!-- 第 2 部分：Gazebo 系统插件 -->
    <!-- ⚠️ 文件名经过了修改（见坑点 #4） -->
    <gazebo>
        <plugin filename="gz_ros2_control-system"
                name="gz_ros2_control::GazeboSimROS2ControlPlugin">
            <!-- 指向控制器参数文件 -->
            <parameters>$(find my_ros_car)/config/controllers.yaml</parameters>
        </plugin>
    </gazebo>
</xacro:macro>
```

**关键概念：**
- `command_interface` = 控制器**写入**的接口（告诉关节该转多快）
- `state_interface` = 控制器**读取**的接口（从仿真获取实际角度/速度）
- 关节名称必须与 URDF 中定义的完全一致，否则控制器无法 claim 接口

---

#### ⚠️ 坑点 #4：Gazebo 插件文件名

原始代码用的是 `libgz_ros2_control-system.so`。参考官方 demo 后改为 `gz_ros2_control-system`（Gazebo Harmonic 的插件命名规范：不需要 `lib` 前缀和 `.so` 后缀）。

---

### 3.7 controllers.yaml —— 控制器参数配置

**作用：** 配置 ros2_control 的两个控制器：`diff_drive_controller`（差速驱动，核心）和 `joint_state_broadcaster`（关节状态发布）。

这个文件经历了**最多次修改**（见坑点 #2, #3, #5, #6）。

```yaml
# ============================================================
# controller_manager 顶层配置
# ============================================================
# ⚠️ 这个段是后来加的（坑点 #5），来自官方 demo 的对比分析
controller_manager:
  ros__parameters:
    update_rate: 100  # 控制循环频率 100Hz（每 10ms 更新一次）

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

# ============================================================
# 差速驱动控制器 —— 系统的核心
# ============================================================
diff_drive_controller:
  ros__parameters:
    # type 必须在这里（ros__parameters 内部）！否则 spawner 找不到
    type: diff_drive_controller/DiffDriveController

    # 坐标帧名称
    base_frame_id: base_link    # 机器人本体帧
    odom_frame_id: odom          # 里程计帧（世界坐标系中机器人位置的估计）

    # 指令超时：如果超过 0.5 秒没收到新指令，自动停车
    cmd_vel_timeout: 0.5

    # 是否发布 odom → base_link 的 TF 变换
    enable_odom_tf: true

    # ======== 运动学参数 ========
    # ⚠️ 这两个值必须与 URDF 中的物理尺寸精确一致！
    wheel_separation: 0.34    # 左右轮中心距 (m)
    wheel_radius: 0.05        # 轮子半径 (m)

    # 4WD：四个轮子的关节名称列表
    # 同一侧的轮子会收到相同的速度指令
    left_wheel_names:
      - left_front_wheel_joint
      - left_rear_wheel_joint
    right_wheel_names:
      - right_front_wheel_joint
      - right_rear_wheel_joint

    # ======== 速度/加速度限制 ========
    # 设置限制可以防止小车跑太快或突然加速
    linear.x.has_velocity_limits: true
    linear.x.has_acceleration_limits: true
    linear.x.max_velocity: 1.0      # 最大线速度 1 m/s
    linear.x.min_velocity: -1.0     # 最大后退速度 -1 m/s
    linear.x.max_acceleration: 1.0   # 最大线加速度 1 m/s²

    angular.z.has_velocity_limits: true
    angular.z.has_acceleration_limits: true
    angular.z.max_velocity: 1.0      # 最大角速度 1 rad/s
    angular.z.min_velocity: -1.0
    angular.z.max_acceleration: 1.0
    angular.z.min_acceleration: -1.0

    # jerk（加加速度）设为 NaN 表示"不限制"
    linear.x.max_jerk: .NAN
    linear.x.min_jerk: .NAN
    angular.z.max_jerk: .NAN
    angular.z.min_jerk: .NAN

# ============================================================
# 关节状态广播器 —— 发布 /joint_states 话题
# ============================================================
joint_state_broadcaster:
  ros__parameters:
    type: joint_state_broadcaster/JointStateBroadcaster
    use_sim_time: true
```

**差速驱动运动学公式（控制器内部计算）：**

```
wheel_velocity_left  = (linear_velocity - angular_velocity * wheel_separation / 2) / wheel_radius
wheel_velocity_right = (linear_velocity + angular_velocity * wheel_separation / 2) / wheel_radius
```

#### ⚠️ 坑点 #2：type 放在 ros__parameters 内部还是外部？

```yaml
# ❌ 错误（rclcpp 会报错 "Cannot have a value before ros__parameters"）：
diff_drive_controller:
  type: diff_drive_controller/DiffDriveController   # 在外面！
  ros__parameters:
    base_frame_id: base_link
    ...

# ✅ 正确（type 必须在 ros__parameters 内部）：
diff_drive_controller:
  ros__parameters:
    type: diff_drive_controller/DiffDriveController   # 在里面！
    base_frame_id: base_link
    ...
```

#### ⚠️ 坑点 #3：不要写 Lyrical 不支持的参数

- ~~`use_stamped_vel`~~ — Lyrical 已固定使用 TwistStamped，删掉
- ~~`publish_rate`~~ — Lyrical 已移除该参数，删掉
- ~~`wheels_per_side`~~ — 现在自动从 `left_wheel_names` 的长度推算，删掉

#### ⚠️ 坑点 #5：缺少 controller_manager 顶层配置段

参考官方 `gz_ros2_control_demos` 后发现需要 `controller_manager` 段指定 `update_rate`。没有这段的话，控制器虽然能加载但不一定正常运行。

---

### 3.8 gazebo.launch.py —— 顶层启动文件

**作用：** 用户唯一需要 `ros2 launch` 的文件。启动 Gazebo + 机器人 + ros_gz_bridge。

```python
#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # ---- 世界文件参数（可通过命令行 --world xxx 覆盖）----
    world = LaunchConfiguration("world")
    declare_world = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution([
            FindPackageShare("my_ros_car"), "worlds", "simple_office.world"
        ])
    )

    # ---- 1. 启动 Gazebo Sim ----
    # IncludeLaunchDescription：嵌套另一个 launch 文件
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"
            ])
        ),
        launch_arguments={
            "gz_args": ["-r ", world]   # -r: 启动后自动开始仿真
        }.items()
    )

    # ---- 2. 启动机器人 ----
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("my_ros_car"), "launch", "robot.launch.py"
            ])
        )
    )

    # ---- 3. ROS ↔ Gazebo 消息桥接 ----
    # ⚠️ 格式经过了修改（见坑点 #1）
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            # GZ→ROS：仿真时钟（必须！否则控制器不知道仿真时间）
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # ROS→GZ：速度指令（本项目中不经过这里，
            # 实际控制走 ros2_control 路径）
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        ]
    )

    return LaunchDescription([
        declare_world,
        gazebo,
        robot,
        bridge
    ])
```

**关键概念：**
- `LaunchDescription` = 启动任务的清单，包含一系列 `Action`
- `IncludeLaunchDescription` = 嵌套另一个 launch 文件（模块化设计）
- `Node` = 启动一个 ROS 节点
- `LaunchConfiguration` + `DeclareLaunchArgument` = 可配置的启动参数

#### ⚠️ 坑点 #1：parameter_bridge 语法

```
格式：topic@ROS_type[direction]GZ_type
     @ 必须作为第一个分隔符（不是 [ 或 ]）！
     [ = GZ→ROS（Gazebo 发布，ROS 订阅）
     ] = ROS→GZ（ROS 发布，Gazebo 订阅）

✅ 正确：/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
❌ 错误：/clock[rosgraph_msgs/msg/Clock@...     # @ 不是第一个
❌ 错误：/clock@...@...                           # 两个 @
```

---

### 3.9 robot.launch.py —— 机器人启动 + 控制器加载

**作用：** 解析 URDF（Xacro→URDF 转换）→ 启动 robot_state_publisher → 在 Gazebo 中 spawn 机器人 → **顺序加载**控制器。

```python
def generate_launch_description():
    pkg_share = FindPackageShare("my_ros_car")

    # ---- 1. 解析 Xacro → URDF ----
    # Command 执行 shell 命令，相当于在终端运行: xacro robot.urdf.xacro
    urdf_path = PathJoinSubstitution([pkg_share, "urdf", "robot.urdf.xacro"])
    robot_description = ParameterValue(
        Command(["xacro ", urdf_path]), value_type=str
    )

    # ---- 2. Robot State Publisher（发布 TF 变换） ----
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}]
    )

    # ---- 3. Spawn 机器人到 Gazebo ----
    # ⚠️ spawn z 从 0.0 改为 0.08（配合轮子 joint z=-0.03 的修改）
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",   # 用话题（不只是参数）获取 URDF
            "-name", "my_robot",
            "-x", "0", "-y", "0", "-z", "0.08",   # spawn 位置
            "-allow_renaming", "true",
        ]
    )

    # ---- 4. 控制器加载（事件驱动的顺序启动） ----
    # ⚠️ 从 TimerAction 改为 RegisterEventHandler（见坑点 #6, #7）
    controllers_yaml = PathJoinSubstitution([pkg_share, "config", "controllers.yaml"])

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen"
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "-c", "/controller_manager",
            "--param-file", controllers_yaml,
        ],
        output="screen"
    )

    # 事件驱动顺序: spawn 完成 → 加载 broadcaster → 加载 diff_drive
    load_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    load_diff_drive = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    return LaunchDescription([
        robot_state_publisher,
        gz_spawn_entity,
        load_broadcaster,
        load_diff_drive,
    ])
```

**关键概念：**
- `ParameterValue(Command(...))` = 执行 shell 命令并将结果作为 ROS 参数
- `ros_gz_sim create` = Gazebo 的工具，通过话题获取机器人描述并创建模型
- `RegisterEventHandler(OnProcessExit(...))` = "当 A 完成后，再执行 B"（事件驱动）
- `spawner` = controller_manager 的工具，加载/配置/激活控制器

#### ⚠️ 坑点 #6：spawner 必须传 -p 或 --param-file

原始代码中 spawner 没有 `-p` 参数：
```python
# ❌ 错误（spawner 找不到 type 参数）
arguments=["diff_drive_controller", "-c", "/controller_manager"]

# ✅ 正确（spawner 从 YAML 文件读取 type 和其余参数）
arguments=["diff_drive_controller", "-c", "/controller_manager",
           "--param-file", controllers_yaml]
```

#### ⚠️ 坑点 #7：控制器加载顺序

原始代码用 `TimerAction(period=3.0)` 同时启动两个 spawner：
```python
# ❌ 可能有竞态条件
load_controllers = TimerAction(period=3.0,
    actions=[joint_state_broadcaster_spawner, diff_drive_spawner])

# ✅ 事件驱动顺序启动（参考官方 demo）
load_broadcaster = RegisterEventHandler(
    event_handler=OnProcessExit(
        target_action=gz_spawn_entity,
        on_exit=[joint_state_broadcaster_spawner]))
load_diff_drive = RegisterEventHandler(
    event_handler=OnProcessExit(
        target_action=joint_state_broadcaster_spawner,
        on_exit=[diff_drive_spawner]))
```

---

### 3.10 teleop.launch.py —— 键盘遥控（已废弃，保留作参考）

**作用：** 尝试在 launch 中运行 `teleop_twist_keyboard`。  
**为什么不工作：** `ros2 launch` 启动的子进程没有 TTY（终端输入），`teleop_twist_keyboard` 需要 stdin 读取按键，所以会 crash（`termios.error`）。

**结论：** 键盘控制必须用 `ros2 run` 直接在终端运行，不能包在 launch 里。

---

### 3.11 cmd_vel_test.py —— 速度指令测试脚本

**作用：** 让小车持续向前跑，用来验证控制系统是否正常。调试阶段的核心工具。

```python
#!/usr/bin/env python3
"""持续发送 TwistStamped 速度指令，使用 sim time 测试小车运动"""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped

class CmdVelTest(Node):
    def __init__(self):
        super().__init__("cmd_vel_test")

        # ⚠️ 坑点核心：必须手动启用 use_sim_time！
        # 没有这个，get_clock().now() 返回的是系统时间（Unix 时间戳），
        # 而控制器用的是仿真时间（从 /clock 桥接过来）。
        # 系统时间 ≈ 17 亿秒，仿真时间 ≈ 几十秒 → 时间戳差距巨大
        # → cmd_vel_timeout(0.5s) 判定指令超时 → 全部丢弃！
        self.set_parameters([
            Parameter("use_sim_time", Parameter.Type.BOOL, True)
        ])

        # 发布到 diff_drive_controller 订阅的话题
        # ⚠️ 消息类型必须是 TwistStamped，不是 Twist！
        self.pub = self.create_publisher(
            TwistStamped, "/diff_drive_controller/cmd_vel", 10
        )
        self.count = 0
        self.started = False

        # 每 50ms 触发一次回调（20Hz 发布频率）
        self.timer = self.create_timer(0.05, self.timer_cb)

    def timer_cb(self):
        now = self.get_clock().now()

        # 等待仿真时钟同步（nanoseconds == 0 表示还没连上 /clock）
        if now.nanoseconds == 0:
            if self.count == 0:
                print("Waiting for sim time (/clock)...")
            self.count += 1
            return

        if not self.started:
            self.started = True
            print(f"Sim time ready: {now.nanoseconds/1e9:.1f}s, "
                  f"starting to publish...")

        # 构造 TwistStamped 消息
        msg = TwistStamped()
        msg.header.stamp = now.to_msg()    # ⚠️ 时间戳必须是当前仿真时间
        msg.header.frame_id = "base_link"   # 速度参考坐标系
        msg.twist.linear.x = 0.5            # 0.5 m/s 向前
        msg.twist.angular.z = 0.0           # 不旋转
        self.pub.publish(msg)

        self.count += 1
        if self.count % 100 == 0:  # 每 100 条消息打印一次
            print(f"Sent {self.count} msgs, vx=0.5")

def main():
    rclpy.init()
    node = CmdVelTest()
    try:
        rclpy.spin(node)    # 阻塞，直到 Ctrl+C
    except KeyboardInterrupt:
        print("\nStopped.")
    node.destroy_node()
    rclpy.shutdown()
```

**使用方式：**
```bash
# 必须 source 环境！
source /opt/ros/lyrical/setup.bash
source ~/dddmr_navigation/install/setup.bash
python3 /path/to/cmd_vel_test.py
```

---

### 3.12 twist_relay.py —— Twist→TwistStamped 消息中继

**作用：** `teleop_twist_keyboard` 发布的是 `Twist`（无时间戳），而 `diff_drive_controller` 需要 `TwistStamped`（带时间戳）。这个节点做格式转换。

```python
class TwistRelay(Node):
    def __init__(self):
        super().__init__("twist_relay")
        # 输入：teleop_twist_keyboard 的 Twist
        self.sub = self.create_subscription(
            Twist, "/cmd_vel", self.twist_cb, 10
        )
        # 输出：diff_drive_controller 需要的 TwistStamped
        self.pub = self.create_publisher(
            TwistStamped, "/diff_drive_controller/cmd_vel", 10
        )

    def twist_cb(self, msg: Twist):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()  # 加上时间戳
        stamped.header.frame_id = "base_link"                    # 加上坐标系
        stamped.twist = msg      # 复制速度值（linear.x, angular.z）
        self.pub.publish(stamped)
```

**为什么需要这个？**

| | teleop_twist_keyboard 输出 | diff_drive_controller 输入 |
|---|---|---|
| 消息类型 | `geometry_msgs/msg/Twist` | `geometry_msgs/msg/TwistStamped` |
| 话题名 | `/cmd_vel` | `/diff_drive_controller/cmd_vel` |
| 时间戳 | 无 | 有（`header.stamp`） |
| 坐标系 | 无 | 有（`header.frame_id`） |

两个维度都不匹配，需要中继桥接。没有时间戳的消息会被 `cmd_vel_timeout: 0.5` 当作超时丢弃。

---

### 3.13 辅助文件

#### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.20)
project(my_ros_car)

find_package(ament_cmake REQUIRED)

# 把这些目录安装到 install/share/my_ros_car/ 下
install(DIRECTORY urdf/   DESTINATION share/${PROJECT_NAME}/urdf/)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch/)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config/)
install(DIRECTORY maps/   DESTINATION share/${PROJECT_NAME}/maps/)
install(DIRECTORY worlds/ DESTINATION share/${PROJECT_NAME}/worlds/)

ament_package()
```

这个项目没有 C++/Python 源码需要编译（纯 URDF + Launch + 脚本），所以 CMakeLists.txt 只负责安装文件到正确位置。

#### package.xml

声明包名、版本、依赖。`rosdep` 工具会根据这个文件自动安装缺失的依赖。

```xml
<package format="3">
    <name>my_ros_car</name>
    <version>0.0.1</version>
    <description>四轮差速机器人 Gazebo 仿真包</description>

    <!-- 关键依赖 -->
    <depend>gz_ros2_control</depend>        <!-- Gazebo + ros2_control 桥 -->
    <depend>diff_drive_controller</depend>   <!-- 差速驱动 -->
    <depend>joint_state_broadcaster</depend> <!-- 关节状态 -->
    <depend>ros_gz_sim</depend>             <!-- Gazebo 仿真 -->
    <depend>robot_state_publisher</depend>   <!-- TF 发布 -->
    ...
</package>
```

---

## 4. 调试全记录：踩过的 8 个坑

这些坑按照发现顺序排列，覆盖了配置、ROS API 版本差异、Gazebo 插件规范、物理几何等多个层面。

| # | 症状 | 根因 | 修复 | 关键教训 |
|---|------|------|------|---------|
| **1** | `ros_gz_bridge` 启动 crash | parameter_bridge 语法 `@` 必须是第一个分隔符 | `/clock[` → `/clock@`, `/cmd_vel]` → `/cmd_vel@` | 认真读官方文档，不要凭经验猜测 |
| **2** | 控制器报 "type param not defined" | spawner 没有 `-p` 传参数文件，且 `type` 在 rclcpp 不允许的位置 | 加 `-p controllers.yaml`，把 `type` 放进 `ros__parameters` 内部 | rclcpp 的参数文件格式：`ros__parameters` 是唯一的顶级键 |
| **3** | 控制器加载了但不工作 | YAML 里有 Lyrical 不支持的参数（`use_stamped_vel`, `publish_rate`, `wheels_per_side`） | 删除无效参数，只保留当前版本 Parameters 结构体中存在的 | 每个 ROS 版本 API 不同，以实际安装的头文件为准 |
| **4** | 小车一动不动 | 1. `libgz_ros2_control-system.so` 文件名不规范 2. 缺少 `controller_manager` 段 3. 不是事件驱动的顺序启动 | 参考官方 `gz_ros2_control_demos`，逐项对齐 | 当东西不工作时，找官方 demo 对比，而不是自己猜 |
| **5** | 轮子转了但车不动 | 底盘底部和轮子底部在同一高度，底盘擦地 | 轮子 joint z 从 0 → -0.03，spawn z 从 0 → 0.08 | 仿真不只是代码问题，物理几何参数同样关键 |
| **6** | （潜在问题）右轮反转导致打转 | 左右轮 axis 方向相反，diff_drive_controller 不知道这件事 | 右轮 axis 从 `0 -1 0` 改为 `0 1 0` | diff_drive_controller 默认左右轮同方向旋转 |
| **7** | `ros2 topic pub Twist` 没有匹配的订阅者 | Lyrical 的 diff_drive_controller 订阅 TwistStamped | 改用 TwistStamped 发布 | ROS 不同版本间 API 会变，Lyrical 是最新版 |
| **8** | `teleop_twist_keyboard` 不能直接控制 | 发布 Twist（无时间戳），而控制器要 TwistStamped | 写 `twist_relay.py` 做消息转换 | 时间戳在仿真中很重要（`cmd_vel_timeout` 检查依赖它） |

---

## 5. 完整数据流图

```
                         ┌──────────────────────┐
                         │     键盘 (终端3)      │
                         │ teleop_twist_keyboard │
                         └──────────┬───────────┘
                                    │ Twist
                                    ▼
                         ┌──────────────────────┐
                         │   twist_relay (终端2) │
                         │   加时间戳+坐标系      │
                         └──────────┬───────────┘
                                    │ TwistStamped
                                    ▼
┌──────────────────────────────────────────────────────────────┐
│                      ROS 2 仿真层                             │
│                                                              │
│  /clock ◄── ros_gz_bridge ◄── Gazebo 仿真时钟               │
│                                                              │
│  /diff_drive_controller/cmd_vel                              │
│       │                                                      │
│       ▼                                                      │
│  ┌──────────────────────┐     ┌─────────────────────┐       │
│  │ diff_drive_controller │────▶│ /diff_drive_controller│     │
│  │   差速运动学计算       │     │   /odom (里程计)      │     │
│  │   v→ω 转换           │     └─────────────────────┘       │
│  └──────────┬───────────┘                                    │
│             │ 4× joint velocity                              │
│             ▼                                                │
│  ┌──────────────────────┐     ┌─────────────────────┐       │
│  │ gz_ros2_control       │────▶│ /joint_states        │       │
│  │ GazeboSimSystem       │     │ (关节角度/速度)       │       │
│  │ 硬件接口层             │     └─────────────────────┘       │
│  └──────────┬───────────┘                                    │
│             │ 关节力/速度                                     │
├─────────────┼────────────────────────────────────────────────┤
│             ▼              Gazebo Harmonic 物理引擎           │
│  ┌──────────────────────────────────────────────────────┐    │
│  │  ODE 物理引擎                                         │    │
│  │  • 碰撞检测 (chassis ↔ ground)                        │    │
│  │  • 摩擦力 (mu1/mu2 + fdir1)                          │    │
│  │  • 重力 (9.81 m/s²)                                  │    │
│  │  • 关节动力学 (damping=0.1)                           │    │
│  └──────────────────────────────────────────────────────┘    │
│                                                              │
└──────────────────────────────────────────────────────────────┘
```

---

## 6. 操作速查表

### 构建

```bash
cd ~/dddmr_navigation
source /opt/ros/lyrical/setup.bash
colcon build --packages-select my_ros_car --symlink-install
source install/setup.bash
```

### 启动仿真

```bash
# 终端 1
ros2 launch my_ros_car gazebo.launch.py
```

### 测试小车运动

```bash
# 终端 2：让小车持续 0.5m/s 向前跑
python3 ~/dddmr_navigation/src/my_ros_car/scripts/cmd_vel_test.py
```

### 键盘控制

```bash
# 终端 2：启动 Twist→TwistStamped 中继
python3 ~/dddmr_navigation/src/my_ros_car/scripts/twist_relay.py

# 终端 3：键盘遥控
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# 按键说明：i 前进, , 后退, j 左转, l 右转, k 停车, q/z 调速
```

### 诊断命令

```bash
ros2 node list                    # 查看所有节点
ros2 topic list                   # 查看所有话题
ros2 control list_controllers     # 查看控制器状态（必须是 active）
ros2 control list_hardware_interfaces  # 查看硬件接口是否 claimed
ros2 topic info /diff_drive_controller/cmd_vel  # 查看订阅者数量
ros2 topic echo /diff_drive_controller/odom --once  # 查看里程计
```

---

> **版本历史：**
> - 2026-07-16 v1：初始版本，覆盖项目建立到键盘控制成功的完整过程（步骤 1-5）
> - 2026-07-16 v2：新增 SLAM、Nav2、自主巡检系统（步骤 6-8），Docker 跨环境通信
> - 调试过程中参考了 `/opt/ros/lyrical/include/diff_drive_controller/` 下的头文件和
>   `/tmp/demo_extract/opt/ros/lyrical/share/gz_ros2_control_demos/` 下的官方 demo


---

## 7. SLAM 建图系统 (slam_toolbox)

**架构变化：** 从本章开始，slam_toolbox 和 nav2 运行在 **Docker 容器 (Humble)** 中，而不是宿主机 (Lyrical)。因为 Lyrical 是 2026 年 5 月发布的最新版，slam_toolbox 和 nav2 尚未发布适配版本。Docker 容器安装了完整的 ROS 2 Humble + 所有导航相关包。

### 跨环境通信原理

```
宿主机 (Lyrical)                     Docker 容器 (Humble)
┌────────────────────┐               ┌────────────────────────┐
│ Gazebo 仿真         │   DDS 发现    │ slam_toolbox / nav2    │
│ 发布: /scan, /clock,│◄────────────►│ 订阅宿主机发布的 topic  │
│ /tf, /odom          │   (网络/UDP)  │ 发布: /map, /cmd_vel   │
└────────────────────┘               └────────────────────────┘
         │                                      │
         └──── 数据卷映射: src/my_ros_car/ ─────┘
              (文件实时同步，无需重新构建)
```

关键配置：
- 宿主机和 Docker 使用**相同的 ROS_DOMAIN_ID**（默认 0）
- Docker 使用 `--net=host` 网络模式（和宿主机共享网络栈）
- 宿主机源码目录通过 `-v` 挂载到容器内
- 所有节点使用 `use_sim_time:=true`，从宿主机的 `/clock` 获取仿真时间

---

### 7.1 slam_toolbox.yaml —— 建图参数

**作用：** 配置 SLAM Toolbox 的在线异步建图模式。

```yaml
slam_toolbox:
  ros__parameters:
    use_sim_time: true

    # ---- 工作模式 ----
    # mapping: 建图+定位（本文件使用的模式）
    # localization: 仅用已有地图定位（建图完成后切换）
    mode: mapping

    # ---- 坐标系（必须与 URDF 一致）----
    odom_frame: odom                      # 里程计系
    map_frame: map                        # 地图系（slam_toolbox 发布 map→odom）
    base_frame: base_footprint            # 机器人基座投影
    scan_topic: /scan                     # 订阅 LiDAR 数据

    # ---- 地图参数 ----
    resolution: 0.05                      # 5cm/格
    map_size: [20.0, 20.0, 0.5]          # 初始尺寸 20m×20m

    # ---- 扫描匹配 ----
    minimum_travel_distance: 0.1          # 最少移动 10cm 才添加关键帧
    scan_queue_size: 10                   # 缓冲 10 帧激光

    # ---- 回环检测 ----
    enable_loop_closure: true             # 开启回环检测
    loop_search_maximum_distance: 5.0     # 回环搜索半径 5m

    # ---- 距离过滤 ----
    max_laser_range: 8.0                  # 8m 以外忽略
    minimum_laser_range: 0.15             # 15cm 以内忽略
```

**关键概念：**
- **异步建图 (async)**：扫描匹配和地图更新在后台线程进行，不阻塞传感器数据接收
- **回环检测**：当机器人回到之前经过的地方，自动修正累积误差
- **关键帧**：不是每一帧激光都用来更新地图，只有移动一定距离/角度后才添加

**建图过程中的坐标系变换链：**
```
map → (slam_toolbox 发布) → odom → (diff_drive_controller 发布) → base_footprint → base_link → ...
```

---

### 7.2 slam.launch.py —— 建图启动

**作用：** 启动 `async_slam_toolbox_node`，加载建图参数。

```python
slam_toolbox_node = Node(
    package="slam_toolbox",
    executable="async_slam_toolbox_node",  # 异步在线 SLAM
    name="slam_toolbox",
    parameters=[slam_params_file, {"use_sim_time": True}],
)
```

**使用流程：**
```bash
# === 终端 1（宿主机）：启动仿真 ===
ros2 launch my_ros_car gazebo.launch.py

# === 终端 2（Docker 容器）：启动建图 ===
docker exec -it dddmr_x64_navigation bash
source /opt/ros/humble/setup.bash
source /home/daipegqin/dddmr_navigation/install/setup.bash
ros2 launch my_ros_car slam.launch.py

# === 终端 3（宿主机）：键盘控制机器人建图 ===
python3 ~/dddmr_navigation/src/my_ros_car/scripts/twist_relay.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# === 建图完成后保存地图（在 Docker 容器内）===
ros2 run nav2_map_server map_saver_cli -f ~/dddmr_navigation/src/my_ros_car/maps/my_map
```

---

## 8. Navigation2 导航系统

### 8.1 nav2_params.yaml —— 导航核心参数

**作用：** Nav2 完整参数配置（约 500 行），包含 8 个核心组件。

**Nav2 节点关系图：**
```
┌─────────────────────────────────────────────────────────────┐
│                   bt_navigator (行为树导航器)                 │
│                   总指挥，协调各模块                           │
├──────────┬──────────┬──────────┬──────────┬─────────────────┤
│ planner  │controller│ behavior │ waypoint │ velocity_smoother│
│ _server  │ _server  │ _server  │_follower │                  │
│ (全局规划)│ (局部控制)│ (恢复行为)│ (航点跟随)│ (速度平滑)       │
├──────────┴──────────┴──────────┴──────────┴─────────────────┤
│              amcl (定位)          │    map_server (地图服务)  │
├──────────────────────────────────────────────────────────────┤
│     global_costmap (全局代价地图)  │ local_costmap (局部代价地图)│
└──────────────────────────────────────────────────────────────┘
```

#### 关键组件配置

**A) AMCL 定位**
```yaml
amcl:
  ros__parameters:
    odom_topic: /diff_drive_controller/odom  # ⚠️ 指向宿主机发布的里程计
    scan_topic: /scan
    min_particles: 500                       # 最少 500 个粒子
    max_particles: 2000                      # 最多 2000 个（自适应）
    alpha1~alpha5: 0.2                       # 里程计噪声参数
    laser_model_type: likelihood_field       # 似然场模型
```

**B) 全局规划器 (SmacPlannerHybrid)**
```yaml
planner_server:
  ros__parameters:
    GridBased:
      plugin: "nav2_smac_planner/SmacPlannerHybrid"
      tolerance: 0.25                       # 目标容差 25cm
      minimum_turning_radius: 0.20          # 最小转弯半径（差速设较小值）
      angle_quantization_bins: 72           # 5° 角度分辨率
```

**C) 局部控制器 (Regulated Pure Pursuit)**
```yaml
controller_server:
  ros__parameters:
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.3               # 目标速度 0.3 m/s
      max_linear_vel: 0.5
      lookahead_dist: 0.4                   # 前瞻距离 40cm
      use_collision_detection: true          # 启用碰撞检测
      goal_dist_tol: 0.15                   # 到达容差 15cm
```

**D) 代价地图**
```yaml
# 全局代价地图：20×20m，静态地图+膨胀层
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "inflation_layer"]
      inflation_layer.inflation_radius: 0.30  # 障碍物周围 30cm 安全区

# 局部代价地图：5×5m 滚动窗口，障碍物+体素+膨胀层
local_costmap:
  local_costmap:
    ros__parameters:
      width: 5, height: 5                    # 5×5m 窗口
      rolling_window: true                   # 随机器人移动
      plugins: ["obstacle_layer", "voxel_layer", "inflation_layer"]
```

**E) 速度平滑器**
```yaml
velocity_smoother:
  ros__parameters:
    input_topic: /cmd_vel_nav                         # 从 controller 接收
    output_topic: /diff_drive_controller/cmd_vel      # 发布到机器人
    odom_topic: /diff_drive_controller/odom           # 获取当前速度
    max_linear_accel: 0.5                             # 平滑加减速
```

**关键概念：**
- **代价地图 (Costmap)**：用代价值表示环境通行难度，0=自由，254=障碍
- **膨胀层 (Inflation)**：在障碍物周围扩展安全 margin
- **前瞻距离 (Lookahead)**：Pure Pursuit 算法"看前方多远"，影响跟踪平滑度
- **AMCL 粒子滤波**：用一堆加权粒子表示机器人可能的位姿，激光匹配好的粒子权重高

---

### 8.2 navigation.launch.py —— 导航启动

**作用：** 启动完整 Nav2 栈，包含 9 个节点 + 生命周期管理器。

```python
# 核心节点（按启动顺序）
map_server_node        # 1. 加载预建地图
amcl_node              # 2. 定位
planner_server_node    # 3. 全局规划
controller_server_node # 4. 局部控制
behavior_server_node   # 5. 恢复行为
velocity_smoother_node # 6. 速度平滑
bt_navigator_node      # 7. 行为树导航器
waypoint_follower_node # 8. 航点跟随
lifecycle_manager_node # 9. 自动激活所有节点
```

**生命周期管理器**：Nav2 节点遵循 ROS 2 生命周期（Unconfigured → Inactive → Active）。`lifecycle_manager` 自动按依赖顺序激活所有节点，无需手动操作。

**使用流程：**
```bash
# === 在 Docker 容器内启动导航 ===
ros2 launch my_ros_car navigation.launch.py map:=/path/to/map.yaml

# === 在 RViz 中测试（Docker 容器内） ===
rviz2
# 设置 Fixed Frame = map
# 添加 TF, Map, Costmap, Path 等可视化
# 使用 "2D Goal Pose" 工具点击目标点
```

---

## 9. 自主巡检系统

### 9.1 patrol_node.py —— 巡检逻辑

**作用：** 使用 Nav2 Simple Commander Python API 实现自动巡航。

```python
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class PatrolNode(Node):
    def __init__(self):
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()  # 等待 Nav2 就绪

    def start_patrol(self):
        """主循环: 遍历航点 → 导航 → 停留 → 下一个"""
        while rclpy.ok():
            wp = self.waypoints[self.current_point_idx]  # 取当前航点
            goal_pose = self._make_pose(wp["pose"])       # 构造 PoseStamped

            self.navigator.goToPose(goal_pose)            # 发送导航任务

            # 等待到达或超时
            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                rclpy.spin_once(self, timeout_sec=0.1)

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self._do_inspection(wp["label"])  # 执行巡检任务
                time.sleep(wait_sec)              # 停留 N 秒

            # 移动到下一个航点（循环或往返模式）
            self._update_index()
```

**两种巡逻模式：**
- **循环模式 (loop)**：1→2→3→1→2→3→...（永远循环）
- **往返模式 (round_trip)**：1→2→3→2→1→2→...（到终点后反向）

**巡检任务扩展点**（`_do_inspection()` 函数）：
- 拍摄照片保存：调用 camera topic 订阅截帧
- 读取温度/湿度传感器数据
- 检测设备状态指示灯
- 记录巡检日志到文件

---

### 9.2 patrol_points.yaml —— 航点配置

**作用：** 定义巡逻路径上的所有目标点。

```yaml
patrol:
  ros__parameters:
    patrol_mode: "loop"          # 循环模式
    wait_at_point_sec: 3.0       # 每点停留 3 秒
    navigation_timeout_sec: 60.0 # 导航超时 60 秒
    max_retries: 3               # 失败重试 3 次

    waypoints:
      - pose: [0.0, 0.0, 0.0]
        label: "起点/充电桩"
      - pose: [3.0, 0.0, 0.0]
        label: "走廊东端"
      # ... 按需添加
```

**如何获取航点坐标：**
```bash
# 方法 1: 在 RViz 中查看机器人当前位置
# 方法 2: 在终端查看 AMCL 估计位姿
ros2 topic echo /amcl_pose --once
# 方法 3: 使用 "2D Pose Estimate" 在 RViz 中点击获取坐标
```

---

### 9.3 patrol.launch.py —— 巡检启动

**作用：** 一键启动 Nav2 导航栈 + 巡检节点。

```python
# 复用 navigation.launch.py 的导航栈
navigation_launch = IncludeLaunchDescription(...)

# 额外启动巡检节点
patrol_node = Node(
    package="my_ros_car",
    executable="patrol_node.py",
    parameters=[patrol_points_file, {"use_sim_time": True}],
)
```

**使用流程：**
```bash
# === 终端 1（宿主机）：启动仿真 ===
ros2 launch my_ros_car gazebo.launch.py

# === 终端 2（Docker 容器）：一键启动巡检 ===
docker exec -it dddmr_x64_navigation bash
source /opt/ros/humble/setup.bash
source /home/daipegqin/dddmr_navigation/install/setup.bash
ros2 launch my_ros_car patrol.launch.py map:=/home/daipegqin/dddmr_navigation/src/my_ros_car/maps/my_map.yaml
```

---

## 10. 辅助脚本

### 10.1 nav2_odom_relay.py —— 里程计中继

**作用：** 将 `/diff_drive_controller/odom` 转发到 `/odom`（备用方案）。

正常情况下不需要这个脚本，因为 nav2_params.yaml 中已经通过 `odom_topic` 参数指向了正确的里程计话题。但如果某些 Nav2 版本或第三方插件硬编码了 `/odom` 话题，可以运行此中继。

```python
class OdomRelay(Node):
    """里程计话题中继: /diff_drive_controller/odom → /odom"""
    def __init__(self):
        self.sub = self.create_subscription(Odometry,
            "/diff_drive_controller/odom", self.odom_callback, 10)
        self.pub = self.create_publisher(Odometry, "/odom", 10)

    def odom_callback(self, msg: Odometry):
        self.pub.publish(msg)  # 直接转发
```

---

## 11. Docker 跨环境通信详解

### 为什么需要 Docker？

| 组件 | 宿主机 (Lyrical) | Docker (Humble) | 原因 |
|------|:-:|:-:|------|
| Gazebo + ros2_control | ✅ | ❌ | 宿主机 GPU 渲染性能更好 |
| slam_toolbox | ❌ 未发布 | ✅ 已安装 | Lyrical 太新，包还没发布 |
| nav2 | ❌ 未发布 | ✅ 已安装 | 同上 |
| robot_state_publisher | ✅ | ❌ | 和 URDF 绑定，跑在宿主机 |
| patrol_node | ❌ | ✅ | 依赖 nav2_simple_commander |

### DDS 发现机制

ROS 2 使用 DDS (Data Distribution Service) 进行节点间通信。DDS 默认使用 UDP 多播进行节点发现。

**关键设置：**
```bash
# 宿主机和 Docker 必须使用相同的 ROS_DOMAIN_ID
export ROS_DOMAIN_ID=0  # 默认值，确保两边一致

# Docker 运行时建议使用 host 网络模式
docker run --net=host ...
# 如果不用 host 模式，需要配置 DDS 使用特定网段
```

### 验证跨环境通信

```bash
# 宿主机启动仿真后，在 Docker 容器内执行：
docker exec -it dddmr_x64_navigation bash
source /opt/ros/humble/setup.bash

# 1. 检查 topic 发现
ros2 topic list
# 应该能看到宿主机的话题: /scan, /clock, /diff_drive_controller/odom, /tf 等

# 2. 检查具体消息
ros2 topic echo /scan --once    # 应该能看到激光数据
ros2 topic echo /clock --once   # 应该能看到仿真时钟

# 3. 如果看不到，检查:
#    - ROS_DOMAIN_ID 是否一致
#    - Docker 网络模式 (需要 --net=host)
#    - 防火墙是否阻止 UDP 多播 (sudo ufw status)
```

### 文件同步

宿主机目录 `/home/daipegqin/dddmr_navigation/` 通过 `docker run -v` 挂载到容器内。修改 launch 文件或配置后：
- 在 Docker 内**不需要重新 build**（Python 文件直接生效）
- 修改 `.yaml` 配置后**重启对应的 launch 即可**
- 只有 C++ 代码需要 `colcon build`（本项目没有 C++ 源码）

### 完整操作流程总结

```bash
# ═══════════════════════════════════════════════════════════
# 阶段 1: 建图
# ═══════════════════════════════════════════════════════════

# 宿主机终端 1: 启动仿真
ros2 launch my_ros_car gazebo.launch.py

# 宿主机终端 2: 键盘控制
python3 ~/dddmr_navigation/src/my_ros_car/scripts/twist_relay.py

# 宿主机终端 3: 键盘
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Docker 终端: 启动 SLAM
docker exec -it dddmr_x64_navigation bash
ros2 launch my_ros_car slam.launch.py

# 控制机器人走遍环境 → 建图完成 → 保存地图
# (在 Docker 容器内执行)
ros2 run nav2_map_server map_saver_cli -f ~/dddmr_navigation/src/my_ros_car/maps/my_map

# ═══════════════════════════════════════════════════════════
# 阶段 2: 导航
# ═══════════════════════════════════════════════════════════

# 宿主机: 重启仿真 (Ctrl+C 后重新启动)
ros2 launch my_ros_car gazebo.launch.py

# Docker: 启动导航
ros2 launch my_ros_car navigation.launch.py

# 在 RViz 中用 "2D Goal Pose" 测试导航

# ═══════════════════════════════════════════════════════════
# 阶段 3: 自动巡检
# ═══════════════════════════════════════════════════════════

# 宿主机: 仿真保持运行

# Docker: 一键启动巡检
ros2 launch my_ros_car patrol.launch.py
```
