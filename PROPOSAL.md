# 视觉SLAM自主导航与AI目标跟踪机器人

## 项目概述

基于 Orange Pi 5B + 4路EMM_V5闭环步进电机 + USB摄像头的自主移动机器人。具备视觉SLAM建图定位、NPU加速AI目标检测、自主导航避障、人物跟随等能力。采用 ROS2 Humble 架构，适用于简历展示 / 毕业设计 / 机器人竞赛。

---

## 1. 硬件架构

```
┌──────────────────────────────────────────────────────────┐
│                    Orange Pi 5B (RK3588S)                │
│  ┌─────────────────────────────────────────────────────┐ │
│  │  Ubuntu 22.04 / 24.04 + ROS2 Humble                 │ │
│  │                                                     │ │
│  │  CPU: 4×A76 + 4×A55  │  NPU: 6 TOPS  │  GPU: G610  │ │
│  └─────────────────────────────────────────────────────┘ │
│         │           │            │            │          │
│    USB 3.0     UART×2      I2C/SPI     WiFi/Ethernet     │
│         │           │            │            │          │
│    ┌────┴────┐ ┌───┴────┐ ┌───┴────┐ ┌─────┴──────┐    │
│    │ USB     │ │EMM_V5  │ │ IMU    │ │ Remote     │    │
│    │ Camera  │ │Motor×4 │ │ Sensor │ │ Control    │    │
│    └─────────┘ └───┬────┘ └────────┘ └────────────┘    │
│                    │                                     │
│            ┌───────┼───────┐                             │
│            ▼       ▼       ▼       ▼                     │
│         ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐               │
│         │Motor│ │Motor│ │Motor│ │Motor│               │
│         │  1  │ │  2  │ │  3  │ │  4  │               │
│         └──┬──┘ └──┬──┘ └──┬──┘ └──┬──┘               │
│            │       │       │       │                      │
│         ┌──┴───────┴───────┴───────┴──┐                 │
│         │      4-Wheel Chassis        │                 │
│         └─────────────────────────────┘                 │
└──────────────────────────────────────────────────────────┘
```

### 1.1 现有器件

| 器件 | 型号/规格 | 数量 | 用途 |
|------|-----------|------|------|
| 主控 | Orange Pi 5B (RK3588S) | 1 | 核心计算、AI推理、运行ROS2 |
| 电机驱动 | EMM_V5.0 闭环步进驱动 | 4 | 驱动4个步进电机，UART通信 |
| 电机 | 闭环步进电机（配EMM_V5） | 4 | 底盘动力 |
| 摄像头 | USB摄像头 | 1 | 视觉SLAM输入 + AI目标检测 |
| 底盘 | 4轮小车底盘 | 1 | 承载所有器件 |

### 1.2 建议新增器件

| 器件 | 型号建议 | 数量 | 预估价格 | 用途 | 优先级 |
|------|----------|------|----------|------|--------|
| IMU惯性测量单元 | MPU6050 / ICM-20948 / GY-85 | 1 | ¥10-30 | 视觉-惯导融合SLAM，提升定位精度 | **[必选]** |
| USB转TTL模块 | CH340 / CP2102 | 1-2 | ¥5-10 | 扩展UART接口，连接EMM_V5 | **[必选]** 若板载UART不够 |
| 深度相机 | Intel RealSense D435i / OAK-D Lite | 1 | ¥500-1200 | 直接获取深度，大幅提升SLAM和避障 | [可选/进阶] |
| 2D激光雷达 | LD19 / LD06 / YDLIDAR X2 | 1 | ¥100-300 | 鲁棒的障碍物检测 | [可选/进阶] |
| 锂电池 | 12V/24V 锂电池组（带保护板） | 1 | ¥50-150 | 移动供电 | **[必选]** |
| 降压模块 | DC-DC 降压模块 (→5V 3A) | 1 | ¥10-20 | 电池降压给香橙派供电 | **[必选]** |
| 超声波模块 | HC-SR04 | 2-4 | ¥3-5/个 | 近距离辅助避障 | [可选] |
| OLED显示屏 | 0.96" SSD1306 (I2C) | 1 | ¥8-15 | 显示运行状态/日志 | [可选] |
| 电压检测模块 | 电阻分压 + ADC / INA219 | 1 | ¥5-15 | 电池电量监测 | [可选] |
| 蜂鸣器 | 有源蜂鸣器模块 | 1 | ¥2-5 | 状态提示/报警 | [可选] |

### 1.3 EMM_V5 电机驱动通信协议

```
帧格式: [地址(1B)] [命令(1-2B)] [参数(NB)] [校验=0x6B]

示例命令:
  - 使能电机:    [addr] 0xF3 0xAB [state] [snF] 0x6B
  - 速度模式:    [addr] 0xF6 [dir] [vel_H] [vel_L] [acc] [snF] 0x6B
  - 位置模式:    [addr] 0xFD [dir] [vel_H] [vel_L] [acc] [clk×4] [raF] [snF] 0x6B
  - 同步触发:    [addr] 0xFF 0x66 0x6B
  - 读取当前位置: [addr] 0x36 0x6B
  - 读取实时速度: [addr] 0x35 0x6B
  - 立即停止:    [addr] 0xFE 0x98 [snF] 0x6B
  - 回零操作:    [addr] 0x9A [o_mode] [snF] 0x6B

关键特性:
  - 闭环控制: 编码器反馈，不丢步
  - 速度范围: 0-5000 RPM
  - 位置范围: 0 - 2^32-1 脉冲
  - 同步运动: 多电机同时触发
  - 状态读取: 位置/速度/误差/电压/电流/温度
```

---

## 2. 软件架构

### 2.1 分层架构

```
┌─────────────────────────────────────────────────────────────┐
│  Layer 5: 应用层 (Application)                              │
│  ┌───────────┐ ┌──────────────┐ ┌─────────────┐           │
│  │ Web 仪表盘 │ │ 语音/手势控制 │ │ 远程遥控    │           │
│  └───────────┘ └──────────────┘ └─────────────┘           │
├─────────────────────────────────────────────────────────────┤
│  Layer 4: 决策层 (Decision)                                 │
│  ┌────────────────┐ ┌──────────────┐ ┌──────────────────┐  │
│  │ 行为状态机(FSM) │ │ 任务编排器    │ │ 跟随/巡线/探索模式│  │
│  └────────────────┘ └──────────────┘ └──────────────────┘  │
├─────────────────────────────────────────────────────────────┤
│  Layer 3: 感知层 (Perception)                               │
│  ┌──────────┐ ┌──────────┐ ┌───────────┐ ┌─────────────┐ │
│  │ ORB-SLAM3│ │ YOLOv8   │ │ 深度估计   │ │ 传感器融合  │ │
│  │ 视觉SLAM │ │ 目标检测  │ │ (ZoeDepth)│ │ (EKF/UKF)  │ │
│  └──────────┘ └──────────┘ └───────────┘ └─────────────┘ │
├─────────────────────────────────────────────────────────────┤
│  Layer 2: 控制层 (Control)                                  │
│  ┌──────────┐ ┌───────────┐ ┌──────────┐ ┌─────────────┐ │
│  │ 运动学模型│ │ PID控制器  │ │ 里程计    │ │ 电机指令分发 │ │
│  │ (差速/全向)│ │ (速度环)  │ │ 计算      │ │ (同步/异步) │ │
│  └──────────┘ └───────────┘ └──────────┘ └─────────────┘ │
├─────────────────────────────────────────────────────────────┤
│  Layer 1: 硬件抽象层 (HAL)                                  │
│  ┌──────────┐ ┌───────────┐ ┌──────────┐ ┌─────────────┐ │
│  │ EMM_V5   │ │ Camera    │ │ IMU      │ │ GPIO/Buzzer │ │
│  │ Driver   │ │ Driver    │ │ Driver   │ │ Manager     │ │
│  └──────────┘ └───────────┘ └──────────┘ └─────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 ROS2 节点架构

```
                    ┌─────────────────┐
                    │   /web_node     │  Flask/FastAPI + rosbridge
                    │  (Web仪表盘)    │
                    └────────┬────────┘
                             │
        ┌────────────────────┼────────────────────┐
        │                    │                    │
        ▼                    ▼                    ▼
┌───────────────┐   ┌───────────────┐   ┌───────────────┐
│ /camera_node  │   │ /imu_node     │   │ /motor_driver  │
│ 发布:          │   │ 发布:          │   │ 发布: /odom    │
│  /image_raw   │──▶│  /imu/data    │──▶│       /motor_status
│  /camera_info │   │               │   │ 订阅: /cmd_vel │
└───────┬───────┘   └───────┬───────┘   └───────┬───────┘
        │                   │                   │
        ▼                   ▼                   ▼
┌──────────────┐   ┌───────────────┐   ┌───────────────┐
│ /slam_node   │   │ /sensor_fusion │  │ /slam_node    │
│ ORB-SLAM3    │   │ robot_        │   │ 发布:          │
│ 订阅:/image  │   │ localization  │   │  /map          │
│ 发布:/tf,     │   │ 订阅:/odom    │   │  /tf (map→odom)│
│  /pose       │   │      /imu     │   │  /trajectory  │
└──────┬───────┘   │ 发布:/filtered│   └───────┬───────┘
       │           │      _odom,   │           │
       │           │      /tf      │           │
       │           └───────┬───────┘           │
       │                   │                   │
       ▼                   ▼                   ▼
┌──────────────────────────────────────────────────┐
│              /object_detection_node               │
│  YOLOv8 on RKNN NPU                              │
│  订阅: /image_raw                                 │
│  发布: /detections, /detection_image              │
└──────────────────────┬───────────────────────────┘
                       │
                       ▼
┌──────────────────────────────────────────────────┐
│              /behavior_node                       │
│  行为决策 (FSM)                                   │
│  订阅: /map, /detections, /filtered_odom, /tf    │
│  发布: /cmd_vel, /goal_pose                      │
└──────────────────────┬───────────────────────────┘
                       │
                       ▼
┌──────────────────────────────────────────────────┐
│              /nav2 (Navigation2)                  │
│  全局规划器 (A*/Smac) + 局部规划器 (TEB/DWB)        │
│  订阅: /map, /scan, /tf                          │
│  发布: /cmd_vel, /plan                           │
└──────────────────────────────────────────────────┘
```

### 2.3 核心数据流

```
[Camera] ──USB──▶ V4L2 ──▶ /camera_node ──▶ /image_raw (sensor_msgs/Image)
                                               │
                        ┌──────────────────────┼──────────────────────┐
                        ▼                      ▼                      ▼
                  /slam_node             /object_detection     /depth_estimation
                  ORB-SLAM3              YOLOv8 on NPU         ZoeDepth/MiDaS
                  ──▶ /tf               ──▶ /detections       ──▶ /depth
                  ──▶ /pose             ──▶ /tracking         ──▶ /pointcloud

[Motors] ──UART──▶ /motor_driver_node ──▶ /odom (encoder feedback)
                                            │
[IMU] ───I2C───▶ /imu_node ──▶ /imu/data ──┤
                                            ▼
                                    /sensor_fusion_node
                                    (robot_localization EKF)
                                    ──▶ /odometry/filtered
                                    ──▶ /tf (odom→base_footprint)

                               ┌──────────┐
                               │ /map     │ (from SLAM)
                               │ /tf      │ (map→odom)
                               │ /detections│
                               └────┬─────┘
                                    ▼
                             /behavior_node
                             ┌────────────────┐
                             │  行为决策 FSM   │
                             │                │
                             │  IDLE          │
                             │  EXPLORE       │
                             │  NAVIGATE      │
                             │  FOLLOW        │
                             │  RETURN_HOME   │
                             └───────┬────────┘
                                     │
                                     ▼
                              /cmd_vel (geometry_msgs/Twist)
                                     │
                                     ▼
                             /motor_driver_node
                             ┌────────────────┐
                             │ 逆运动学解算    │
                             │ 速度→电机坐标   │
                             │ UART 发送指令   │
                             └────────────────┘
```

### 2.4 TF 坐标树

```
map ──▶ odom ──▶ base_footprint ──▶ base_link ──▶ camera_link
                                           │
                                           ├──▶ imu_link
                                           ├──▶ wheel_front_left
                                           ├──▶ wheel_front_right
                                           ├──▶ wheel_rear_left
                                           └──▶ wheel_rear_right
```

---

## 3. 运动学模型

### 3.1 差速驱动模型 (Skid-Steer) — 普通4轮

```
        前
    ┌─────────┐
    │ FL   FR │      v = (vL + vR) / 2
    │         │      ω = (vR - vL) / L
    │ RL   RR │
    └─────────┘      vL = v - ω·L/2
        后            vR = v + ω·L/2

其中 L = 左右轮间距 (base_width)
```

### 3.2 麦克纳姆轮模型 (Mecanum) — 如安装全向轮

```
  v_fl = v_x - v_y - (Lx+Ly)·ω
  v_fr = v_x + v_y + (Lx+Ly)·ω
  v_rl = v_x + v_y - (Lx+Ly)·ω
  v_rr = v_x - v_y + (Lx+Ly)·ω

其中 Lx = 前后半轴距, Ly = 左右半轴距
```

### 3.3 编码器里程计

```
EMM_V5 闭环驱动可实时读取编码器位置值 (S_ENCL)
  → 计算 Δ脉冲 → 转换为轮子位移 → 推算机器人位姿增量

轮速 = Δ脉冲 × (2π / 编码器分辨率) × (1/车轮周长)
```

---

## 4. 核心算法模块

### 4.1 视觉SLAM — ORB-SLAM3

```
输入: 单目摄像头图像流
输出: 相机位姿 (Tcw), 稀疏地图点, 关键帧

工作流程:
  1. ORB特征提取 (GPU加速)
  2. 特征匹配 + 运动估计
  3. 局部BA优化 (滑动窗口)
  4. 回环检测 (DBoW2词袋模型)
  5. 全局图优化 (g2o)

关键参数:
  - 帧率: 15-30 FPS
  - 特征点数: 1000-2000
  - 地图点: 动态管理
  - 尺度恢复: 需要已知运动或IMU初始化
```

### 4.2 AI目标检测 — YOLOv8 on NPU

```
输入: RGB图像 (640×480)
输出: 检测框 [class_id, x, y, w, h, confidence]

部署流程:
  1. 训练/下载 YOLOv8s 预训练模型 (.pt)
  2. 转换为 ONNX (.onnx)
  3. 转换为 RKNN (RKNN-Toolkit2)
  4. 在香橙派上部署 RKNN Runtime
  5. NPU 推理 → 提取检测结果

目标类别 (COCO 80类):
  - 人物(person)     ← 人物跟随核心
  - 自行车/摩托车     ← 障碍物
  - 汽车/卡车         ← 障碍物
  - 椅子/桌子         ← 场景理解
  - 狗/猫             ← 宠物跟随扩展

性能预期: NPU 推理 < 50ms (≥20 FPS)
```

### 4.3 目标跟踪 — DeepSORT / ByteTrack

```
输入: 连续帧检测结果
输出: 目标ID + 轨迹

DeepSORT流程:
  1. 卡尔曼滤波预测目标下一帧位置
  2. 匈牙利算法匹配检测-跟踪 (级联匹配)
  3. 外观特征(ReID)作为关联度量
  4. 轨迹管理 (创建/更新/删除)

跟踪ID用于人物跟随: 锁定特定ID的目标
```

### 4.4 传感器融合 — EKF

```
输入:
  - 轮式里程计 /odom (100Hz)
  - IMU /imu/data  (200Hz, 可选)
  - 视觉里程计 /vo (30Hz, 从SLAM)

融合策略 (robot_localization):
  State: [x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]
  
  流程:
    1. IMU提供角速度+加速度 → 姿态估计
    2. 轮式里程计提供线速度 → 速度估计
    3. 视觉里程计提供位置 → 位置漂移修正
    4. EKF融合 → 高频(100Hz)平滑位姿输出
```

### 4.5 路径规划 — Nav2

```
全局规划器 (Smac/A*):
  输入: /map (占据栅格地图), start, goal
  输出: 全局路径 (waypoints)

局部规划器 (TEB/DWB):
  输入: 全局路径, /costmap, /scan, /odom
  输出: /cmd_vel (实时速度指令)
  约束: 运动学约束、障碍物距离、时间最优

代价地图 (Costmap):
  - 静态层: SLAM地图膨胀
  - 障碍物层: 摄像头深度/雷达点云投影
  - 膨胀层: 安全距离 (机器人半径 + margin)
```

### 4.6 人物跟随控制

```
控制策略 (PID + 视线法):
  1. 从 /detections + /tracking 获取跟踪目标 ID 的边界框
  2. 计算目标在图像中的位置 (cx, cy) 和相对角度
  3. PID控制器:
     - 角度环: 保持目标在图像中心 → 调整角速度 ω
     - 距离环: 保持目标边界框大小 → 调整线速度 v
  4. 发布 /cmd_vel

  期望距离 = Kd / bbox_height  (Kd为标定参数)
  角度误差 = (image_center_x - target_center_x) / image_width
```

---

## 5. 行为状态机 (FSM)

```
                    ┌─────────┐
         上电 ────▶ │  INIT   │
                    └────┬────┘
                         │ 初始化完成
                         ▼
                    ┌─────────┐
            ┌──────▶│  IDLE   │◀──────────┐
            │       └────┬────┘           │
            │            │                │
     ┌──────┴──────┐    │    ┌────────────┴────────┐
     │   EXPLORE   │◀───┼───▶│     NAVIGATE        │
     │  (自主探索)  │    │    │   (定点导航)         │
     └──────┬──────┘    │    └──────────┬──────────┘
            │           │               │
            │    ┌──────┴──────┐        │
            └───▶│   FOLLOW    │◀───────┘
                 │ (人物跟随)   │
                 └──────┬──────┘
                        │
                 ┌──────┴──────┐
                 │ RETURN_HOME │
                 │ (自主回充)   │
                 └─────────────┘

模式切换: Web指令 / 语音指令 / 自动避障触发
```

---

## 6. 目录结构

```
smart_car/
├── PROPOSAL.md                    # 本文档 - 项目方案
├── README.md                      # 项目说明 + 快速开始
├── LICENSE                        # 许可证
│
├── docs/                          # 文档
│   ├── architecture.md            # 详细架构文档
│   ├── hardware_setup.md          # 硬件接线指南
│   ├── api_reference.md           # API文档
│   └── images/                    # 架构图/接线图
│
├── firmware/                      # 底层固件（如需要额外MCU）
│
├── scripts/                       # 辅助脚本
│   ├── setup_ros2.sh              # ROS2环境安装
│   ├── build_all.sh               # 编译所有包
│   ├── launch_robot.sh            # 启动机器人
│   └── calibrate_camera.sh        # 摄像头标定
│
├── src/                           # 源码
│   ├── robot_bringup/             # 启动配置包
│   │   ├── launch/
│   │   │   ├── robot_base.launch.py       # 基础硬件启动
│   │   │   ├── robot_navigation.launch.py # 导航启动
│   │   │   └── robot_full.launch.py       # 全功能启动
│   │   └── config/
│   │       ├── robot.yaml                # 机器人参数
│   │       ├── ekf.yaml                  # EKF参数
│   │       └── nav2_params.yaml          # Nav2参数
│   │
│   ├── robot_hal/                  # 硬件抽象层
│   │   ├── include/
│   │   │   ├── emm_v5_driver.hpp          # EMM_V5 驱动封装
│   │   │   ├── serial_port.hpp            # 串口操作封装
│   │   │   ├── camera_driver.hpp          # 摄像头驱动
│   │   │   └── imu_driver.hpp             # IMU驱动
│   │   ├── src/
│   │   │   ├── emm_v5_driver.cpp
│   │   │   ├── serial_port.cpp
│   │   │   ├── camera_driver.cpp
│   │   │   └── imu_driver.cpp
│   │   ├── launch/
│   │   │   └── hal.launch.py
│   │   └── CMakeLists.txt
│   │
│   ├── robot_motor/                # 电机控制节点
│   │   ├── include/
│   │   │   ├── motor_node.hpp
│   │   │   ├── kinematics.hpp           # 运动学模型
│   │   │   └── odometry.hpp             # 里程计计算
│   │   ├── src/
│   │   │   ├── motor_node.cpp
│   │   │   ├── kinematics.cpp
│   │   │   └── odometry.cpp
│   │   ├── launch/
│   │   │   └── motor.launch.py
│   │   └── CMakeLists.txt
│   │
│   ├── robot_perception/           # 感知包
│   │   ├── nodes/
│   │   │   ├── slam_node.cpp             # SLAM节点
│   │   │   ├── object_detection_node.cpp # 目标检测节点
│   │   │   ├── tracking_node.cpp         # 目标跟踪节点
│   │   │   └── depth_node.cpp            # 深度估计节点
│   │   ├── rknn_models/                  # RKNN模型文件
│   │   │   └── yolov8s.rknn
│   │   ├── launch/
│   │   │   └── perception.launch.py
│   │   └── CMakeLists.txt
│   │
│   ├── robot_control/              # 控制层
│   │   ├── nodes/
│   │   │   ├── behavior_node.cpp         # 行为决策
│   │   │   ├── follower_node.cpp         # 人物跟随
│   │   │   └── pid_controller.cpp        # PID控制
│   │   ├── include/
│   │   │   ├── fsm.hpp                   # 有限状态机
│   │   │   └── pid.hpp                   # PID实现
│   │   ├── launch/
│   │   │   └── control.launch.py
│   │   └── CMakeLists.txt
│   │
│   └── robot_web/                  # Web仪表盘
│       ├── static/
│       │   ├── css/
│       │   ├── js/
│       │   └── index.html
│       ├── app.py                        # Flask/FastAPI后端
│       └── launch/
│           └── web.launch.py
│
├── config/                         # 全局配置文件
│   ├── motor_params.yaml           # 电机参数 (轮径、轴距等)
│   ├── camera_params.yaml          # 摄像头标定参数
│   └── robot_env.yaml              # 机器人环境配置
│
├── tests/                          # 测试代码
│   ├── test_serial.cpp
│   ├── test_motor.cpp
│   ├── test_kinematics.cpp
│   ├── test_odometry.cpp
│   └── test_pid.cpp
│
├── reference data/                 # 参考资料 (现有)
│   ├── Emm_V5/
│   └── OrangePi_5B_RK3588S_用户手册_v1.5.1.pdf
│
├── .gitignore
├── CMakeLists.txt                  # 顶层CMake
└── package.xml                     # ROS2 包元信息
```

---

## 7. 开发阶段与里程碑

| 阶段 | 内容 | 耗时 | 关键产出 |
|------|------|------|----------|
| **Phase 1** 硬件调试 | 香橙派装系统、USB转TTL接线、EMM_V5单电机通信测试、摄像头图像采集 | 3-5天 | 单电机可以响应UART指令，摄像头出图 |
| **Phase 2** 底盘驱动 | ROS2环境搭建、EMM_V5驱动封装(motor_driver_node)、运动学解算、里程计发布 | 5-7天 | 可以通过 /cmd_vel 控制小车运动，/odom 有数据 |
| **Phase 3** 视觉感知 | ORB-SLAM3编译适配、YOLOv8转RKNN部署到NPU、目标检测节点运行 | 7-10天 | SLAM可以建图定位，NPU推理可检测目标 |
| **Phase 4** 导航系统 | IMU集成(如有)、EKF传感器融合、Nav2配置与调试、自主导航测试 | 5-7天 | 小车可以在建好图的环境里自主导航到目标点 |
| **Phase 5** 智能行为 | 目标跟踪(DeepSORT)、人物跟随PID控制、行为状态机 | 5-7天 | 小车可以跟随指定人物移动 |
| **Phase 6** 系统集成 | Web仪表盘开发、系统联调、异常处理、性能优化 | 3-5天 | 完整Demo可运行 |
| **Phase 7** 文档收尾 | 架构图、API文档、Demo视频录制、README | 3-5天 | 简历可投 |

**总预估**: 30-45天（兼职）/ 2-3周（全职）

---

## 8. 关键技术难点与解决思路

| 难点 | 风险 | 解决思路 |
|------|------|----------|
| ORB-SLAM3 在 ARM64 编译 | 依赖库多 (Pangolin, Eigen, g2o, DBoW2) | 使用预编译的 ARM 版本或 Docker 交叉编译 |
| 单目 SLAM 尺度模糊 | 无法确定真实尺度 | 用轮式里程计恢复尺度，或加 IMU 做 VI-SLAM |
| NPU 部署 YOLOv8 | RKNN 模型转换失败 | 先跑 CPU 版 OpenCV DNN 作为 fallback |
| 实时性 | 多节点 + 大模型同时跑 | 使用 ROS2 的 QoS 配置，NPU 推理与 CPU 并行 |
| 电机同步 | 4电机异步指令导致运动不平顺 | 利用 EMM_V5 的同步运动指令 (0xFF 0x66)，一次触发4个电机 |
| 地面不平引起打滑 | 编码器里程计累计误差大 | EKF 融合视觉里程计 + IMU，减少发散 |

---

## 9. 技术栈

| 类别 | 技术 | 用途 |
|------|------|------|
| **框架** | ROS2 Humble | 节点通信、包管理、工具链 |
| **语言** | C++17 / Python 3.10+ | C++ 用于核心节点，Python 用于配置/脚本/Web |
| **视觉SLAM** | ORB-SLAM3 / VINS-Mono | 建图、定位、回环检测 |
| **AI推理** | RKNN-Toolkit2 + YOLOv8 | NPU 加速目标检测 |
| **目标跟踪** | DeepSORT / ByteTrack | 多目标跟踪与ID分配 |
| **传感器融合** | robot_localization (EKF) | IMU + 轮速 + 视觉融合 |
| **导航** | Nav2 (Navigation2) | 全局/局部路径规划 |
| **计算机视觉** | OpenCV 4.x | 图像处理、特征提取 |
| **串口通信** | `libserial` / `boost::asio` | UART 与 EMM_V5 通信 |
| **线性代数** | Eigen3 | 矩阵运算、位姿变换 |
| **图优化** | g2o / GTSAM | SLAM 后端优化 |
| **Web后端** | Flask + rosbridge | 远程控制仪表盘 |
| **可视化** | RViz2 + Foxglove | ROS2 调试可视化 |
| **IMU** | I2C (linux/i2c-dev) | 读取IMU数据 |
| **构建** | colcon + CMake | ROS2 包编译 |
| **测试** | GoogleTest / pytest | 单元测试 |
| **版本控制** | Git | 代码管理 |

---

## 10. 简历展示建议

### 项目名称
**"基于 RK3588 与 ROS2 的自主移动机器人平台"**

### 简历描述（中文）
> - 设计并实现了一款基于 Orange Pi 5B (RK3588S) 的自主移动机器人，采用 ROS2 Humble 分布式架构
> - 封装 EMM_V5 闭环步进驱动器的 UART 通信协议，实现四轮差速运动学控制与编码器里程计
> - 集成 ORB-SLAM3 视觉 SLAM 系统，结合 IMU 与轮式里程计实现多传感器 EKF 融合定位
> - 利用 RK3588 的 6TOPS NPU 部署 YOLOv8 目标检测模型，实现 >20FPS 的实时推理
> - 实现 DeepSORT 多目标跟踪 + PID 视觉伺服的人物跟随功能
> - 基于 Nav2 栈构建自主导航系统，支持 Slam 建图、全局路径规划与动态避障

### 简历描述（English）
> - Built a ROS2-based autonomous mobile robot on Orange Pi 5B (RK3588S), utilizing a distributed node architecture
> - Developed UART communication driver for EMM_V5 closed-loop stepper motors with 4-wheel differential kinematics and encoder-based odometry
> - Integrated ORB-SLAM3 visual SLAM with multi-sensor EKF fusion (IMU + wheel odometry + visual)
> - Deployed YOLOv8 on RK3588's 6-TOPS NPU for real-time object detection at 20+ FPS
> - Implemented person-following with DeepSORT tracking and PID-based visual servoing
> - Achieved autonomous navigation using Nav2 stack with global path planning and dynamic obstacle avoidance

### 建议录制的 Demo 视频内容
1. 小车在房间内自主建图（SLAM 过程 + RViz2 可视化）
2. 建图完成后，点击地图目标点，小车自主导航到目标
3. 人物走进画面，小车锁定并跟随行走
4. 遇到障碍物自动避障
5. Web 仪表盘远程监控和控制

---

## 11. 待定问题 (需确认)

- [ ] 底盘轮子布局是普通轮（差速）还是麦克纳姆轮？
- [ ] EMM_V5 驱动器的 UART 是 TTL 电平还是 RS232？ (极大概率是 TTL 3.3V/5V)
- [ ] 4个 EMM_V5 是否接在同一个 UART 总线上（通过地址区分）还是各自独立 UART？
- [ ] USB 摄像头的分辨率和帧率？
- [ ] 是否有电池？电压规格？
- [ ] 是否需要 IMU？如需，建议 MPU6050 还是 ICM-20948？
- [ ] 目标场景是室内还是室外？
