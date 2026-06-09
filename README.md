# Humanoid Robot 系统总体框架

## 项目概述

本项目是一个基于 ROS 2 Humble 的人形机器人上半身控制系统，涵盖从底层硬件驱动、运动规划、遥操作控制到模仿学习数据采集与策略训练的完整链路。

机器人本体为双臂 + 头部结构：
- 左臂 6 自由度 + 夹爪
- 右臂 6 自由度 + 夹爪
- 头部 2 自由度（pitch + yaw）

---

## 系统架构图

```
┌─────────────────────────────────────────────────────────────────┐
│                        上层应用                                   │
│  ┌──────────────┐  ┌──────────────┐  ┌────────────────────────┐ │
│  │ 键盘遥操作    │  │ Servo 伺服   │  │ 模仿学习 Pipeline      │ │
│  │ keyboard_ctrl│  │ servo_control│  │ imitation_pipeline     │ │
│  └──────┬───────┘  └──────┬───────┘  └───────────┬────────────┘ │
├─────────┼──────────────────┼──────────────────────┼─────────────┤
│         │        运动规划层  │                      │             │
│         │  ┌───────────────┴───────────────┐      │             │
│         │  │  MoveIt 2 (move_group)        │      │             │
│         │  │  robot_moveit_config          │      │             │
│         │  │  robot_commander              │      │             │
│         │  └───────────────┬───────────────┘      │             │
├─────────┼──────────────────┼──────────────────────┼─────────────┤
│         │       控制器层     │                      │             │
│         ▼                  ▼                      ▼             │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │  ros2_control (controller_manager)                      │    │
│  │  - joint_state_broadcaster                              │    │
│  │  - right_arm_controller / left_arm_controller           │    │
│  │  - neck_controller                                      │    │
│  │  - left_gripper_controller / right_gripper_controller   │    │
│  └────────────────────────────┬────────────────────────────┘    │
├───────────────────────────────┼─────────────────────────────────┤
│                    硬件接口层   │                                  │
│  ┌────────────────────────────┴────────────────────────────┐    │
│  │  robot_hardware (ros2_control hardware_interface 插件)   │    │
│  │  通过串口/CAN 与电机驱动通信                               │    │
│  └─────────────────────────────────────────────────────────┘    │
├─────────────────────────────────────────────────────────────────┤
│                    仿真环境 (可选)                                │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │  Gazebo + gz_ros2_control / RViz (fake_hw)              │    │
│  │  robot_simulation                                       │    │
│  └─────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────┘
```

---

## 功能包说明

### 核心层

| 包名 | 功能 |
|------|------|
| `robot_description` | URDF/Xacro 模型、mesh 文件、ros2_controllers 配置 |
| `robot_hardware` | ros2_control 硬件接口插件，负责与真实电机通信 |
| `robot_interfaces` | 自定义 ROS 2 消息/服务定义 |
| `robot_moveit_config` | MoveIt 2 配置（SRDF、运动学、规划器、控制器映射） |
| `robot_commander` | MoveIt launch 入口 + commander 节点，统一启动真机/仿真栈 |

### 控制层

| 包名 | 功能 |
|------|------|
| `robot_keyboard_control` | 键盘遥操作，支持关节模式和笛卡尔模式 |
| `robot_servo_control` | 基于 MoveIt Servo 的实时伺服控制（键盘关节 jog） |
| `servo_control` | 早期 servo 测试包（单组 servo 节点 + 测试命令） |

### 数据与学习

| 包名 | 功能 |
|------|------|
| `robot_imitation_pipeline` | 模仿学习数据录制、验证、回放、策略推理节点 |
| `robot_rl_training` | Isaac Sim PPO 训练的 ROS 2 接口包 |

### 仿真

| 包名 | 功能 |
|------|------|
| `robot_simulation` | Gazebo 仿真 launch 文件 |
| `gz_ros2_control` | Gazebo-ROS 2 control 桥接插件 |

### 其他

| 包名 | 功能 |
|------|------|
| `camera_calibration` | 手眼标定工具 |
| `ROS-TCP-Endpoint` | Unity 仿真通信桥接 |

---

## ROS 2 Topic 通信结构

```
/joint_states                    ← joint_state_broadcaster 发布（所有关节位置/速度）
/right_joint_command             ← 键盘控制/策略推理 发布 → 右臂 6 关节目标
/left_joint_command              ← 键盘控制 发布 → 左臂 6 关节目标
/neck_joint_command              ← 键盘控制 发布 → 头部 2 关节目标
/open_right_gripper              ← Bool 消息，true=张开 false=闭合
/open_left_gripper               ← Bool 消息，true=张开 false=闭合
/right_wrist_camera/image_raw    ← 腕部 USB 相机图像（可选）
/head_camera/color/image_raw     ← RealSense 头部相机图像（可选）
```

Servo 模式下额外的 topic：
```
/left_servo/moveit_servo/delta_joint_cmds   ← JointJog 消息
/right_servo/moveit_servo/delta_joint_cmds  ← JointJog 消息
/head_servo/moveit_servo/delta_joint_cmds   ← JointJog 消息
```

---

## 启动流程

### 仿真环境

```bash
# 方式一：Gazebo 全仿真
ros2 launch robot_simulation visual_servoing.launch.py

# 方式二：仅 RViz（fake hardware）
ros2 launch robot_commander robot_moveit.launch.xml
```

### 真机环境

```bash
# 终端 1：启动真机栈（硬件接口 + 控制器 + MoveIt + RViz）
ros2 launch robot_commander robot_moveit.launch.xml use_simulation:=false
```

### 键盘遥操作

```bash
# 终端 2：启动键盘控制
ros2 run robot_keyboard_control keyboard_control
```

### Servo 伺服控制（真机）

```bash
# 终端 2：启动 servo 节点
ros2 launch robot_servo_control servo_control.launch.py

# 终端 3：启动键盘 servo 控制
ros2 run robot_servo_control joint_servo_control
```

### 模仿学习数据录制

```bash
# 终端 1：启动真机栈
ros2 launch robot_commander robot_moveit.launch.xml use_simulation:=false

# 终端 2：启动 recorder（当前配置仅录制关节数据，不需要摄像头）
ros2 launch robot_imitation_pipeline demo_recorder.launch.py

# 终端 3：开始录制
ros2 run robot_imitation_pipeline demo_control start

# 终端 3：停止录制（成功）
ros2 run robot_imitation_pipeline demo_control stop --success
```

录制数据保存在 `data/imitation_raw/episode_XXXXXX/` 下，包含：
- `robot_state.npy` — 右臂关节位置(6) + 速度(6) + 夹爪位置(1) = 13 维
- `action.npy` — 右臂关节命令(6) + 夹爪命令(1) = 7 维
- `joint_pos.npy` / `joint_vel.npy` — 全身关节位置/速度
- `timestamps.npy` — 采样时间戳
- `meta.json` — episode 元信息

---

## 目录结构

```
humanoid/
├── src/                          # ROS 2 功能包源码
│   ├── robot_description/        # URDF 模型
│   ├── robot_hardware/           # 硬件接口
│   ├── robot_interfaces/         # 自定义消息
│   ├── robot_moveit_config/      # MoveIt 配置
│   ├── robot_commander/          # 统一 launch 入口
│   ├── robot_keyboard_control/   # 键盘遥操作
│   ├── robot_servo_control/      # MoveIt Servo 伺服控制
│   ├── servo_control/            # 早期 servo 测试
│   ├── robot_imitation_pipeline/ # 模仿学习数据管线
│   ├── robot_rl_training/        # RL 训练 ROS 接口
│   ├── robot_simulation/         # Gazebo 仿真
│   ├── gz_ros2_control/          # Gazebo 控制桥接
│   ├── camera_calibration/       # 手眼标定
│   └── ROS-TCP-Endpoint/         # Unity 通信
├── RL_training/                  # 离线训练代码（不依赖 ROS）
│   ├── Imitation_Learning/       # Diffusion Policy 低维 baseline
│   ├── Isaac_RL/                 # Isaac Sim PPO 训练
│   └── Mujoco_RL/               # MuJoCo PPO baseline
├── diffusion_policy_reference/   # 原版 Diffusion Policy 参考实现
├── data/                         # 录制的演示数据
├── doc/                         # 项目文档
└── unity/                        # Unity 仿真相关
```

---

## 编译

```bash
cd ~/humanoid
colcon build
source install/setup.bash
```

按需编译：
```bash
# 只编译控制相关
colcon build --packages-select robot_description robot_hardware robot_moveit_config robot_commander

# 只编译数据录制
colcon build --packages-select robot_imitation_pipeline robot_keyboard_control
```

---

## 依赖

- ROS 2 Humble
- MoveIt 2
- ros2_control
- moveit_servo（`sudo apt install ros-humble-moveit-servo`）
- Python 3.10+
- NumPy

可选：
- Gazebo Fortress（仿真）
- OpenCV + cv_bridge（图像录制）
- realsense2_camera（RealSense 头部相机）
- PyTorch（策略训练与推理）

---

## 文档索引

| 主题 | 文件 |
|------|------|
| 仿真环境配置 | [doc/仿真环境配置.md](doc/仿真环境配置.md) |
| MoveIt Commander | [doc/robot_commander_README.md](doc/robot_commander_README.md) |
| 键盘遥操作 | [doc/keyboard_control_README.md](doc/keyboard_control_README.md) |
| 伺服控制 | [doc/伺服控制说明.md](doc/伺服控制说明.md) |
| 模仿学习数据录制 | [doc/imitation_data_recording.md](doc/imitation_data_recording.md) |
| 手眼标定 | [doc/手眼标定说明.md](doc/手眼标定说明.md) |
| RL 训练 | [doc/RL_training.md](doc/RL_training.md) |
| Diffusion Policy 迁移 | [doc/Diffusion report.md](doc/Diffusion%20report.md) |
| 已知问题 | [doc/Problem.md](doc/Problem.md) |

---

## Contributors

- Zhangyu Fan — 硬件
- Tongbin Hu — 软件
- Zhe Wang — 软件
- Wu Jie — 软件
