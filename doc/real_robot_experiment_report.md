# 真机实验报告

## 1. 实验概述

在人形机器人真机上完成了右臂关节数据的采集、处理与 Diffusion Policy 训练验证。本次实验目标是验证从真机数据到策略训练的完整链路可行性。

- 实验日期：2026-05-19
- 机器人：Humanoid 上半身（右臂 6DOF + 夹爪）
- 控制方式：键盘遥操作 / Servo 伺服控制
- 数据格式：ROS 2 rosbag (sqlite3)
- 训练框架：PyTorch + 自研 Diffusion Policy baseline

---

## 2. 数据采集

### 采集配置

| 项目 | 配置 |
|------|------|
| 录制 topic | `/right_arm_controller/state`, `/left_arm_controller/state` |
| 消息类型 | `control_msgs/msg/JointTrajectoryControllerState` |
| 采样率 | ~48 Hz |
| 控制频率 | 48 Hz (ros2_control) |
| 右臂关节 | base_pitch, shoulder_roll, shoulder_yaw, elbow_pitch, wrist_pitch, wrist_yaw |

### 数据统计

| 指标 | 数值 |
|------|------|
| 总 episode 数 | 10 |
| 总帧数 | 35,893 |
| 总时长 | 12.5 min |
| 单条时长 | 40s ~ 148s |
| 采样间隔 | 20.87ms ± 0.33ms (jitter < 2%) |
| 数据质量 | 无 NaN/Inf，全部 clean |

### 右臂关节运动范围

| 关节 | 最小值 (rad) | 最大值 (rad) | 范围 (rad) |
|------|-------------|-------------|-----------|
| base_pitch | -0.054 | +0.486 | 0.540 |
| shoulder_roll | -0.684 | -0.086 | 0.598 |
| shoulder_yaw | -0.683 | -0.134 | 0.548 |
| elbow_pitch | -1.354 | -0.487 | 0.867 |
| wrist_pitch | -0.507 | -0.001 | 0.506 |
| wrist_yaw | -0.444 | -0.242 | 0.202 |

左臂全程静止（range < 0.001 rad），本次实验仅操作右臂。

---

## 3. 控制器跟踪性能

从 rosbag 中的 desired vs actual 数据分析控制器跟踪质量：

| 关节 | 平均误差 (rad) | 标准差 (rad) | 最大瞬时误差 (rad) |
|------|---------------|-------------|-------------------|
| base_pitch | +0.000272 | 0.002083 | 0.177 |
| shoulder_roll | +0.000050 | 0.002447 | 0.230 |
| shoulder_yaw | +0.000109 | 0.001452 | 0.098 |
| elbow_pitch | +0.000390 | 0.002729 | 0.161 |
| wrist_pitch | -0.000050 | 0.001363 | 0.186 |
| wrist_yaw | +0.000004 | 0.000478 | 0.057 |

结论：控制器跟踪精度良好，稳态误差 < 0.003 rad，最大瞬时误差出现在快速运动过渡阶段。

---

## 4. 训练配置与结果

### 训练配置

| 参数 | 值 |
|------|-----|
| 模型 | MLP Diffusion Policy (3-layer, hidden=256) |
| state 维度 | 12 (6 pos + 6 vel) |
| action 维度 | 6 (desired joint position) |
| obs_horizon | 2 |
| pred_horizon | 16 |
| diffusion steps | 100 |
| batch size | 128 |
| learning rate | 1e-4 |
| epochs | 200 |
| optimizer | Adam |
| GPU | RTX 3070 Laptop |
| 可用训练样本 | 35,733 |

### 训练曲线

```
epoch 001: loss = 0.963
epoch 010: loss = 0.233
epoch 050: loss = 0.141
epoch 100: loss = 0.098
epoch 150: loss = 0.090
epoch 200: loss = 0.083
```

Loss 持续下降，200 epoch 后趋于收敛。

### 离线评估结果

在 500 个随机采样上进行 DDPM 完整采样评估：

| 指标 | 数值 |
|------|------|
| MSE (rad) | 0.000103 |
| RMSE (rad) | 0.0102 |
| MAE (rad) | 0.0073 |
| MAE (deg) | 0.42° |

### 每关节预测误差

| 关节 | MAE (rad) | MAE (deg) |
|------|-----------|-----------|
| base_pitch | 0.0069 | 0.40° |
| shoulder_roll | 0.0083 | 0.48° |
| shoulder_yaw | 0.0074 | 0.42° |
| elbow_pitch | 0.0115 | 0.66° |
| wrist_pitch | 0.0077 | 0.44° |
| wrist_yaw | 0.0018 | 0.11° |

### 预测误差随 horizon 变化

误差从 step 0 的 0.0068 rad 缓慢增长到 step 15 的 0.0078 rad，增幅仅 15%，说明模型对 16 步预测保持稳定。

---

## 5. 文件结构

```
data/rosbag/                              # 原始 rosbag 数据 (10 bags)
data/processed/real_robot/
├── figures/
│   ├── all_episodes_trajectories.pdf      # 关节轨迹 (actual vs desired)
│   ├── right_arm_tracking_error.png       # 跟踪误差时间序列
│   ├── right_arm_error_distribution.png   # 跟踪误差分布
│   ├── right_arm_joint_ranges.png         # 关节范围箱线图
│   ├── right_arm_velocity.png             # 速度曲线
│   ├── sampling_analysis.png             # 采样率分析
│   ├── eval_pred_vs_true_sample.png      # 模型预测 vs 真实
│   ├── eval_error_distribution.png       # 预测误差分布
│   └── eval_error_vs_horizon.png         # 误差 vs horizon
├── training_episodes/                     # 导出的训练数据
│   ├── episode_000000/ ~ episode_000009/
│   │   ├── robot_state.npy               # (N, 12) float32
│   │   ├── action.npy                    # (N, 6) float32
│   │   ├── timestamps.npy
│   │   └── meta.json
│   └── dataset_summary.json
└── processed/
    └── stats.json                         # 归一化统计量

data/checkpoints/diffusion_policy/
└── latest.pt                              # 训练好的模型权重

scripts/
├── process_rosbag_data.py                 # 数据提取与可视化
├── data_quality_report.py                 # 数据质量报告
└── eval_diffusion_policy.py              # 模型离线评估
```

---

## 6. 结论与下一步

### 本次验证结论

1. 真机数据采集链路通畅，rosbag 录制的 48Hz 关节数据质量高、无丢帧
2. 控制器跟踪精度满足模仿学习需求（稳态误差 < 0.003 rad）
3. Diffusion Policy 在低维关节空间上训练收敛良好，离线预测精度 < 0.5°
4. 完整的 rosbag → npy → 训练 → 评估 pipeline 已跑通

### 下一步计划

1. **真机 dry-run 推理**：加载 checkpoint，发布预测命令但不执行，验证输出合理性
2. **增加数据量**：当前 10 条 episode 偏少，建议采集 30-50 条覆盖更多姿态变化
3. **加入夹爪**：当前 action 只有 6 维关节位置，后续加入夹爪命令变为 7 维
4. **加入相机**：接入腕部相机后训练视觉 diffusion policy
5. **限幅安全机制**：真机部署前加入关节变化量限制和工作空间检查
