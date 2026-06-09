# Humanoid Imitation Learning Pipeline

本文档用于向导师汇报当前模仿学习系统的完整流程：从真机数据录制、数据校验、数据集转换、行为克隆训练，到 ROS 2 中加载策略并在真机上推理执行。

## 1. 目标与总体方案

本项目的模仿学习目标是让机器人右臂学习人工遥操作示范中的抓取动作。当前实现采用状态到动作的行为克隆基线模型：

- 输入：右臂 6 个关节位置、右臂 6 个关节速度、右夹爪状态，共 13 维。
- 输出：右臂 6 个关节目标位置、右夹爪开合命令，共 7 维。
- 模型：多层感知机 MLP，默认 3 层隐藏层，每层 256 维，ReLU 激活。
- 训练方式：监督学习，最小化模型输出动作与示范动作之间的 MSE。
- 部署方式：ROS 2 节点订阅 `/joint_states`，加载 `best.pt`，周期性推理并发布到 `/right_joint_command` 和 `/open_right_gripper`。

整体链路如下：

```text
人工遥操作
  -> demo_recorder 录制 raw episode
  -> validate_demo 校验数据完整性
  -> convert_to_hdf5 转换为 dataset.npz / dataset.hdf5
  -> train_bc 训练 BC MLP
  -> policy_inference.launch.py 在 ROS 中加载 checkpoint 推理
```

## 2. 目录结构

模仿学习相关内容分成两类：

```text
src/robot_imitation_pipeline/
  config/
    recording.yaml              # 录制参数
    training.yaml               # 训练参数
    replay.yaml                 # demo 回放参数
    policy_inference.yaml       # 策略推理参数
  launch/
    demo_recorder.launch.py
    start_recording_cameras.launch.py
    policy_inference.launch.py  # 新增：ROS policy 推理 launch
  robot_imitation_pipeline/
    nodes/demo_recorder_node.py
    nodes/policy_inference_node.py
    convert_to_hdf5.py
    train_bc.py
    validate_demo.py
    replay_demo.py

RL_training/Imitation_Learning/
  config/cameras.yaml
  tools/
    check_camera_topics.py
    inspect_demo.py
    make_fake_demo.py
    validate_demo.py
  imitation_learning_report.md
```

`src/robot_imitation_pipeline` 是 ROS 2 package，负责运行时节点和可执行入口；`RL_training/Imitation_Learning` 存放训练辅助脚本、检查工具和汇报文档。

## 3. 环境准备

在机器人或开发机上构建 ROS 包：

```bash
cd ~/humanoid
colcon build --packages-select robot_imitation_pipeline robot_keyboard_control robot_commander
source install/setup.bash
```

训练需要 Python 包：

```bash
pip install numpy pyyaml torch h5py
```

如果录制相机图像，需要：

```bash
sudo apt install ros-${ROS_DISTRO}-cv-bridge python3-opencv
```

## 4. 数据录制流程

### 4.1 启动相机

右腕相机是必需输入：

```bash
ros2 launch robot_imitation_pipeline start_recording_cameras.launch.py \
  use_wrist_camera:=true \
  wrist_video_device:=/dev/video0 \
  use_head_camera:=false
```

检查相机 topic：

```bash
ros2 topic hz /right_wrist_camera/image_raw
python3 RL_training/Imitation_Learning/tools/check_camera_topics.py
```

### 4.2 启动真机控制与遥操作

```bash
ros2 launch robot_commander robot_moveit.launch.xml use_simulation:=false
ros2 run robot_commander commander
ros2 run robot_keyboard_control joint_control
```

### 4.3 启动 recorder

```bash
ros2 launch robot_imitation_pipeline demo_recorder.launch.py
```

录制一条成功示范：

```bash
ros2 run robot_imitation_pipeline demo_control start
# 人工遥操作完成一次抓取
ros2 run robot_imitation_pipeline demo_control stop --success
```

失败示范保留但不进入训练：

```bash
ros2 run robot_imitation_pipeline demo_control stop --failure
```

默认输出目录由 `recording.yaml` 决定，通常为：

```text
data/imitation_raw/episode_000001/
```

## 5. Raw Episode 数据格式

每条 episode 的核心文件：

```text
episode_000001/
  meta.json
  success.json
  timestamps.npy
  robot_state.npy              # shape: [T, 13]
  action.npy                   # shape: [T, 7]
  joint_pos.npy                # legacy, shape: [T, 16]
  joint_vel.npy                # legacy, shape: [T, 16]
  actions.npy                  # legacy, shape: [T, 16]
  action_valid.npy
  obs/right_wrist_camera/
    000000.jpg
    000001.jpg
```

当前训练使用低维状态，不使用图像训练：

```text
robot_state[i] = [
  right_arm_joint_pos(6),
  right_arm_joint_vel(6),
  right_gripper_pos(1)
]

action[i] = [
  right_arm_joint_target(6),
  right_gripper_command(1)
]
```

`meta.json` 中的 `diagnostics` 会记录录制期间哪些输入缺失。如果出现 0 帧数据，通常需要检查：

- `/joint_states` 是否发布。
- `/right_joint_command` 是否发布。
- `/open_right_gripper` 是否发布，或是否能从 joint state 初始化夹爪状态。
- `/right_wrist_camera/image_raw` 是否发布图像。

## 6. 数据验证

验证单条 episode：

```bash
ros2 run robot_imitation_pipeline validate_demo data/imitation_raw/episode_000001
```

或使用训练目录中的工具：

```bash
python3 RL_training/Imitation_Learning/tools/validate_demo.py data/imitation_raw/episode_000001
```

查看内容摘要：

```bash
python3 RL_training/Imitation_Learning/tools/inspect_demo.py \
  --episode data/imitation_raw/episode_000001
```

合格数据应满足：

- `frames > 0`
- `robot_state_dim = 13`
- `action_dim = 7`
- 图像数量等于帧数
- `success.json` 中 `valid_for_training: true`
- 不含 NaN/Inf

## 7. 数据集转换

将多条 raw episode 转为训练用数据集：

```bash
ros2 run robot_imitation_pipeline convert_to_hdf5 data/imitation_raw \
  --output-dir data/imitation_converted \
  --val-fraction 0.15 \
  --seed 7
```

输出：

```text
data/imitation_converted/
  dataset.npz
  dataset.hdf5
  splits.json
  manifest.json
```

`dataset.npz` 中关键字段：

- `obs_state`: `[N, 13]`
- `actions`: `[N, 7]`
- `episode_index`: 每个样本来自哪条 episode
- `episode_ends`: 每条 episode 的结束位置

`manifest.json` 记录每条 episode 的样本数、训练/验证划分和成功标记，便于汇报数据规模。

## 8. 行为克隆模型

训练脚本为：

```bash
ros2 run robot_imitation_pipeline train_bc \
  --config src/robot_imitation_pipeline/config/training.yaml \
  --dataset data/imitation_converted/dataset.npz \
  --output-dir data/imitation_runs/bc_state
```

默认模型结构：

```text
input_dim = 13
hidden_dim = 256
num_layers = 3
activation = ReLU
action_dim = 7
loss = MSE(pred_action, demo_action)
optimizer = AdamW
```

训练输出：

```text
data/imitation_runs/bc_state/
  best.pt
  last.pt
```

checkpoint 中保存：

- `model`: PyTorch state_dict
- `input_dim`
- `action_dim`
- `config`
- `epoch`
- `val_loss`

汇报时建议记录：

- episode 数量
- 总样本数
- train/val 划分
- best validation loss
- 是否完成真机 dry-run
- 是否完成真机闭环推理

## 9. ROS 真机策略推理

新增 launch：

```bash
ros2 launch robot_imitation_pipeline policy_inference.launch.py \
  checkpoint_path:=data/imitation_runs/bc_state/best.pt
```

默认 `publish_commands:=false`，只打印模型预测，不向机器人发控制命令。先 dry-run 确认模型输出范围合理。

真机执行时必须显式开启：

```bash
ros2 launch robot_imitation_pipeline policy_inference.launch.py \
  checkpoint_path:=data/imitation_runs/bc_state/best.pt \
  publish_commands:=true
```

推理节点参数位于：

```text
src/robot_imitation_pipeline/config/policy_inference.yaml
```

关键参数：

- `checkpoint_path`: 训练得到的 `best.pt`
- `publish_commands`: 是否真的向机器人发布命令
- `inference_rate_hz`: 推理频率，默认 10 Hz
- `max_joint_step_rad`: 每次推理允许的最大关节变化，用于限制突变
- `stale_joint_state_sec`: joint state 超时保护
- `right_joint_command_topic`: 默认 `/right_joint_command`
- `right_gripper_topic`: 默认 `/open_right_gripper`

推理节点执行逻辑：

```text
订阅 /joint_states
  -> 提取右臂位置、右臂速度、右夹爪位置
  -> 拼成 13 维 robot_state
  -> MLP 输出 7 维 action
  -> 限制单步关节变化
  -> 发布右臂 Float64MultiArray 和右夹爪 Bool
```

## 10. 真机运行安全流程

建议按以下顺序执行：

```bash
# 1. 真机控制栈
ros2 launch robot_commander robot_moveit.launch.xml use_simulation:=false
ros2 run robot_commander commander

# 2. 确认 joint state 正常
ros2 topic hz /joint_states

# 3. dry-run 策略推理
ros2 launch robot_imitation_pipeline policy_inference.launch.py \
  checkpoint_path:=data/imitation_runs/bc_state/best.pt

# 4. 确认输出范围正常后再执行
ros2 launch robot_imitation_pipeline policy_inference.launch.py \
  checkpoint_path:=data/imitation_runs/bc_state/best.pt \
  publish_commands:=true
```

执行前检查：

- 机器人处于示范数据覆盖过的初始姿态附近。
- 急停可用。
- 周围无人员和障碍物。
- `max_joint_step_rad` 设置较小，例如 0.03 到 0.08 rad。
- 第一次执行时低速、短时间观察。

## 11. 当前局限与后续改进

当前模型是状态行为克隆基线，优点是部署简单、调试快；局限是没有图像输入，泛化能力依赖初始位姿和目标物位置一致性。

后续可以改进：

- 增加图像编码器，将右腕相机图像作为视觉输入。
- 使用时序模型，例如 LSTM、Transformer 或 Diffusion Policy。
- 引入动作平滑和速度/加速度限制。
- 增加 episode 级质量筛选和失败示范过滤。
- 在仿真中先进行 policy rollout，再转真机。
- 记录更多传感器数据，例如末端位姿、力/接触状态。

## 12. 汇报摘要

本阶段已经完成一条完整的模仿学习工程链路：ROS 真机录制示范数据，保存标准化 episode；使用校验工具过滤空数据和无效数据；转换为训练数据集；使用 MLP 行为克隆模型训练状态到动作策略；最后通过 ROS 2 policy inference launch 加载模型，在真机控制话题上进行实时推理发布。该链路为后续加入视觉输入和更复杂策略模型提供了基础。
