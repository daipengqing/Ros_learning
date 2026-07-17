# DDDMR Navigation 项目结构说明

## 项目概述

**DDDMR Navigation** 是一个专为 3D 移动机器人设计的导航栈，支持在三维环境中进行建图、定位和自主导航。该项目基于 ROS2 Humble 开发，相比传统的 Nav2 导航栈，它能够处理多楼层建图、立体结构路径规划以及在三维点云地图中的障碍物感知与清除。

**核心优势：**
- 🗺️ **3D 环境建图**：支持多楼层地图、斜坡等复杂地形
- 📍 **高效定位**：基于位姿图的 MCL 定位，支持大范围地图
- 🧭 **3D 路径规划**：全局和局部规划器均在三维空间工作
- 🤖 **多机器人支持**：轮式机器人、四足机器人、人形机器人等
- 🚀 **实时感知**：障碍物标记/清除、语义分割、速度限制区域

---

## 目录结构详解

### 📦 根目录文件

#### [README.md](README.md)
项目的主入口文档，包含：
- 项目简介和核心特性
- 与 Nav2 的对比说明
- 功能演示动画
- 各模块的快速链接
- 引用信息

---

### 🐳 dddmr_docker/

**功能**：Docker 容器化配置，提供统一的开发和部署环境

**支持的系统镜像：**
- Ubuntu 22.04 (无 GPU)
- Ubuntu 22.04 with CUDA 12.6 + TensorRT 10.7 + cuDNN 9 + PyTorch 2.5.1
- Ubuntu 22.04 on NVIDIA L4T JetPack 5.x/6.x

**包含的软件栈：**
- ROS2 Humble
- PCL 1.15
- gtsam 4.2a9

**关键脚本：**
- `docker_file/build.bash` - 构建 Docker 镜像
- `run_demo.bash` - 运行演示容器

**使用场景：** 为所有子包提供一致的开发和运行环境，避免本地环境配置问题。

---

### 📚 src/ 目录 - 核心功能包

#### 1️⃣ dddmr_beginner_guide/

**功能**：新手入门指南，提供完整的上手教程

**包含内容：**
- 🌍 **Gazebo 仿真教程**（四足机器人 Unitree Go2）
  - 双容器配置（Gazebo + 导航）
  - 完整演示步骤
  - 已知问题说明

- 🤖 **真实机器人部署指南**
  - 硬件配置要求（LiDAR、IMU、计算平台）
  - TF 树配置说明
  - 三种工作流程：
    1. 在线建图 + 导航
    2. 离线建图 + 导航
    3. 仅导航（使用已有地图）


**关键配置：**
- `airy_tilt45_mapping.launch` - RoboSense Airy 45°倾斜 LiDAR 建图
- `airy_tilt45_navigation.launch` - 定位与导航

**硬件支持：** 轮式机器人、四足机器人、人形机器人等任何满足以下要求的平台：
- `/cmd_vel` - 速度控制命令
- `/lidar_point_cloud` - 3D LiDAR 点云
- `/odom` - 里程计
- TF: `odom → base_link → lidar_link`

---

#### 2️⃣ dddmr_lego_loam/

**功能**：基于 LeGO-LOAM-BOR 的 3D SLAM 建图系统（ROS2 移植版）

**核心特性：**
- 🗺️ **交互式建图**：支持暂停/恢复建图，实时调整参数
- 🔄 **回环检测**：优化的 ICP 算法实现闭环
- 📊 **位姿图编辑器**：
  - 可视化位姿图
  - 手动添加回环边
  - 合并多个位姿图
- 💾 **数据保存**：位姿图格式存储，支持大场景

**新增功能（相比原版）：**
- 支持倾斜 LiDAR（10°、45°、90° 等）
- Ground FOV 配置优化
- base_footprint 支持（非地面 baselink）
- 条件回环机制，更稳定的建图结果
- 在线和离线（bag 文件）建图模式
- IMU/里程计可选

**关键技术：**
- 基于 LeGO-LOAM-BOR 修改
- 使用优化版 ICP 进行回环检测
- 位姿图输出与 MCL 定位器兼容

**使用场景：** 3D 环境建图，生成用于定位的位姿图地图。

---

#### 3️⃣ dddmr_mcl_3dl/

**功能**：基于位姿图的 3D 蒙特卡洛定位系统（MCL）

**核心特性：**
- 🎯 **子图定位**：Submap 概念大幅降低计算量
  - 可在 Jetson Orin Nano 上定位 500m×500m 的地图
  - 定位范围：`2×激光探测距离 + 2×子图搜索半径`

- ⚡ **自适应粒子更新**：
  - 根据行驶距离/旋转角度更新粒子
  - 机器人静止时减少计算开销

- 🔍 **特征选择**：基于 LeGO-LOAM 的特征点
- 📊 **粒子评分机制优化**：
  - 欧几里得聚类提取（替代点级评分）
  - 特征法向量跟踪（防止虚拟滑动）

**关键创新：**
- 地面点云约束，确保机器人位于地面
- 聚类评分有效利用远处物体（点少的区域）
- 法向量跟踪补偿长墙间的虚拟滑动

**使用场景：** 在大规模 3D 地图中进行实时定位，与 LeGO-LOAM 生成的位姿图配合使用。

---

#### 4️⃣ dddmr_perception_3d/

**功能**：3D 感知框架，支持多种传感器和区域管理

**传感器支持：**
- ✅ 多层旋转 LiDAR（Velodyne/Ouster/Leishen）
- ✅ 深度相机（Realsense/OAK）
- ✅ 扫描型 LiDAR（Livox Mid-360/Unitree 4D LiDAR L1）

**核心功能：**
- 🚧 **障碍物动态管理**：
  - Marking：检测到新障碍物时标记
  - Tracking：跟踪移动障碍物
  - Clearing：确认障碍物消失后清除

- 🗺️ **多层地图支持**：
  - 静态层（Static Layer）
  - 速度限制层（Speed Limit Layer）
  - 禁止进入层（No-Enter Layer）

- 🛠️ **实用工具**：
  - 区域编辑器（创建速度限制/禁止进入区域）
  - 点云删除工具（编辑 .pcd 文件）

**演示案例：**
1. 多层 LiDAR（Leishen C16）障碍物标记/清除
2. 多深度相机（Realsense D455×2）点云融合
3. 扫描型 LiDAR（Unitree G4）动态感知

**性能指标：**
- Jetson Orin Nano：实时处理，CPU 58%，GPU 75%

**使用场景：** 动态环境感知、障碍物管理、自定义导航区域。

---

#### 5️⃣ dddmr_global_planner/

**功能**：基于图搜索的 3D 全局路径规划器

**核心特性：**
- 🔍 **A* 算法**：在点云地图中寻找最短路径
- 📊 **双层图结构**：
  - 静态图：地面约束
  - 动态图：传感器实时数据
- 🎯 **边界检测**：自动识别地图边缘，确保路径远离边界

**工作原理：**
1. 从点云中提取可行走地面
2. 构建可通行图的邻接图
3. A* 搜索找到最优路径
4. 考虑动态障碍物实时更新

**演示：** 轮椅坡道、zigzag 通道等复杂地形

**使用场景：** 在 3D 点云地图中规划从起点到目标点的全局路径。

---

#### 6️⃣ dddmr_local_planner/

**功能**：3D 局部路径规划器，基于 DWB 算法改进

**核心特性：**
- 🎯 **轨迹生成器**：生成候选轨迹
- ⚖️ **评分器（Critics）**：多维度评估轨迹质量
- 🔄 **恢复行为**：处理局部困境
- 🎨 **独立配置**：每个生成器可独立设置评分器和权重

**3D 扩展特性：**
- **3D 碰撞检测**：检查点云是否在立方体内（而非 2D 多边形）
- **电机约束考虑**：`dd_simple_trajectory_generator_theory`
  - 考虑电机最大转速限制
  - 防止超速轨迹生成

**与 Nav2 DWB 的区别：**
- 三维空间中的碰撞检查和轨迹评分
- 每个轨迹生成器独立的评分器配置
- 电机约束集成到轨迹生成

**演示：** 使用 "Play Ground" 可视化测试不同场景下的局部规划

**使用场景：** 实时避障、轨迹优化、局部路径跟随。

---

#### 7️⃣ dddmr_p2p_move_base/

**功能**：点对点移动基座控制器（有限状态机）

**核心职责：**
- 协调全局规划器、局部规划器和恢复行为
- 管理导航状态机
- 处理多机器人运动学模型

**支持的机器人运动学：**
- 🚗 **全向轮**（Omni-directional）
- 🚙 **阿克曼转向**（Ackermann Steering）
- 🚲 **三轮车**（Tricycle）
- 🚛 **铰接式车辆**（Articulated Vehicle）
- ⚙️ **差速驱动**（Differential Drive）

**工作流程：**
1. 接收目标位姿
2. 调用全局规划器生成路径
3. 局部规划器跟踪路径
4. 异常时触发恢复行为
5. 到达目标或失败

**演示：** 与 MCL 定位配合，在 bag 文件上重放导航

**使用场景：** 统一接口控制不同类型移动机器人从 A 点导航到 B 点。

---

#### 8️⃣ dddmr_odom_3d/

**功能**：3D 里程计示例（教学用途）

**核心特性：**
- 📐 **欧拉角融合**：将 2D 里程计与 IMU 融合生成 3D 里程计
- 🎓 **教育目的**：帮助理解 3D 里程计计算原理

**数学原理：**
- X 轴：前进方向（来自里程计）
- Y 轴：侧向（假设无侧滑）
- Z 轴：俯仰角积分（来自 IMU）

**注意事项：**
- ⚠️ 仅用于学习理解，生产环境需要更完善的考虑
- 假设无轮滑、无离地
- IMU 姿态必须经过良好滤波

**使用场景：** 教学演示，理解差速驱动机器人的 3D 里程计原理。

---

#### 9️⃣ dddmr_semantic_segmentation/

**功能**：语义分割与点云着色

**核心技术：**
- 🎨 **语义分割**：使用 DDRNet 模型进行实时语义分割
- 🚀 **TensorRT 加速**：显著提升推理速度
- 🔗 **深度对齐**：将分割结果与深度图像对齐并着色到点云

**性能指标：**

| 平台 | FPS | CPU | GPU |
|------|-----|-----|-----|
| Jetson Orin Nano | 15 | 58% | 75% |
| Jetson Orin AGX 32GB | 19 | 24% | 30% |

**支持模式：**
- RGB-D 相机实时推理
- 点云分割（排除特定类别）

**工作流程：**
1. RGB-D 图像输入
2. DDRNet 语义分割（TensorRT 引擎）
3. 深度图与分割结果对齐
4. 生成语义点云

**可排除类别：**  sidewalk(0), parking(1) 等（可配置）

**使用场景：** 为点云添加语义信息，辅助导航决策。

---

#### 🔟 dddmr_trt/

**功能**：TensorRT 模型转换工具库

**职责：**
- 为所有需要深度学习的包生成 TensorRT 库
- 统一管理模型转换流程

**使用场景：** 内部工具包，为语义分割和 YOLO 检测提供加速支持。

---

#### 1️⃣1️⃣ dddmr_pcl/

**功能**：点云库（PCL）封装工具包

**职责：**
- 提供通用的点云处理函数
- 为其他模块共享的点云算法

**使用场景：** 基础库，被其他模块依赖使用。

---

#### 1️⃣2️⃣ dddmr_pg_map_server/

**功能**：位姿图地图服务器

**职责：**
- 加载和提供位姿图地图
- 地图数据服务

**README 内容较少**，可能是一个较新的或内部使用的包。

**使用场景：** 为定位和导航提供地图数据服务。

---

#### 1️⃣3️⃣ dddmr_rviz_tools/

**功能**：RViz2 可视化工具集

**职责：**
- 提供自定义 RViz2 插件
- 可视化辅助工具

**README 内容较少**，可能包含：
- 自定义显示面板
- 交互工具
- 数据可视化插件

**使用场景：** 增强 RViz2 可视化体验，调试和监控。

---

#### 1️⃣4️⃣ dddmr_explore_and_search/

**功能**：自主探索与搜索演示

**核心特性：**
- 🗺️ **集成 SLAM**：使用 LeGO-LOAM
- 🧭 **导航与避障**：完整的导航能力
- 🔍 **探索策略**：基于已探索边缘的随机探索

**系统组成：**
1. SLAM 建图
2. 导航与避障
3. 随机探索策略

**工作流程：**
1. 启动建图和导航
2. 探索节点随机选择边缘点作为目标
3. 发送给 p2p_move_base 执行

**状态：** 智能探索策略仍在开发中，当前版本展示与 LeGO-LOAM 实时接口的关键结构。

**使用场景：** 未知环境自主探索、地图构建。

---

#### 1️⃣5️⃣ dddmr_sys_core/

**功能**：系统核心基础组件

**包含内容：**
- 基类定义
- 枚举状态定义
- 公共工具函数

**使用场景：** 为所有其他包提供共享的基础类和常量定义。

---

## 依赖关系图

```
dddmr_beginner_guide (新手入门)
    ├── dddmr_lego_loam (建图)
    │   └── dddmr_pcl (点云工具)
    ├── dddmr_mcl_3dl (定位)
    │   ├── dddmr_lego_loam (位姿图)
    │   └── dddmr_pcl
    ├── dddmr_perception_3d (感知)
    │   └── dddmr_pcl
    ├── dddmr_global_planner (全局规划)
    │   └── dddmr_perception_3d
    ├── dddmr_local_planner (局部规划)
    ├── dddmr_p2p_move_base (导航协调)
    │   ├── dddmr_global_planner
    │   └── dddmr_local_planner
    ├── dddmr_odom_3d (3D里程计)
    ├── dddmr_semantic_segmentation (语义分割)
    │   └── dddmr_trt (TensorRT加速)
    └── dddmr_explore_and_search (自主探索)
        └── 以上多个模块
```

---

## 技术栈

### 核心框架
- **操作系统**：Ubuntu 22.04
- **中间件**：ROS2 Humble
- **点云处理**：PCL 1.15
- **图优化**：gtsam 4.2a9

### 深度学习
- **推理引擎**：TensorRT 10.7
- **深度学习框架**：PyTorch 2.5.1
- **模型**：
  - DDRNet（语义分割）
  - YOLO11（目标检测，用于 SLAM 中的人体过滤）

### 硬件支持
- **计算平台**：
  - Intel NUC（x64）
  - NVIDIA Jetson Orin Nano/AGX（arm64/L4T）

- **传感器**：
  - LiDAR：Velodyne/Ouster/Leishen/RoboSense Airy/Livox/Unitree 4D
  - IMU：6轴/9轴
  - 深度相机：Intel Realsense/OAK

---

## 典型工作流程

### 1. 建图阶段
```
启动 LeGO-LOAM → 移动机器人采集数据 → 实时建图
                                    ↓
                           保存位姿图地图
```

### 2. 定位阶段
```
加载位姿图地图 → 启动 MCL → 提供初始位姿
                                    ↓
                           实时定位（粒子滤波）
```

### 3. 导航阶段
```
发送目标位姿 → P2P Move Base 协调
                      ↓
        全局规划器生成路径 → 局部规划器跟踪
                      ↓
             动态避障 → 到达目标
```

### 4. 感知增强（可选）
```
实时点云输入 → 障碍物标记/清除
                          ↓
               动态更新导航地图
```

---

## 快速开始

### 环境准备
1. 安装 Docker
2. 克隆仓库
3. 构建 Docker 镜像：
   ```bash
   cd dddmr_docker/docker_file
   ./build.bash  # 选择适合的镜像：x64 / l4t / x64_gz
   ```

### 新手入门
请参考 **[dddmr_beginner_guide](src/dddmr_beginner_guide/README.md)**：
- Gazebo 仿真演示
- 真实机器人部署教程

---

## 文档资源

- **GitHub 仓库**：https://github.com/dfl-rlab/dddmr_navigation
- **文档材料**：https://github.com/dfl-rlab/dddmr_documentation_materials
- **YOLO 训练**：https://github.com/dddmobilerobot/dddmr_yolo

---

## 引用

```bibtex
@software{dddmr_navigation_dfl-rlab,
  author = {CM, PS, Tarek Taha},
  title = {dddmr_navigation: 3D Mobile Robot Navigation},
  url = {https://github.com/dfl-rlab/dddmr_navigation},
  year = {2025}
}
```

---

## 贡献者

- **LeGO-LOAM 原始作者**：Tixiao Shan, Brendan Englot
- **MCL-3DL 原始作者**：@at-wat
- **DDDMR 改进**：CM, PS, Tarek Taha 及 DFL-RLab 团队

---

**生成日期**：2026-07-12
**文档版本**：1.0
