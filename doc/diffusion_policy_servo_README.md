# Diffusion Policy Servo 推理节点

本文档说明如何用训练好的 Diffusion Policy checkpoint 替代人工
`joint_servo_control`，通过 MoveIt Servo 的 JointJog 接口驱动真机右臂。

## 当前实现

新增节点：

```bash
ros2 run robot_servo_control diffusion_policy_servo_node
```

新增 launch：

```bash
ros2 launch robot_servo_control diffusion_policy_servo.launch.py
```

节点订阅：

- `/joint_states`：`sensor_msgs/msg/JointState`
- `/diffusion_policy_servo/emergency_stop`：`std_msgs/msg/Bool`

节点发布：

- `/right_servo/moveit_servo/delta_joint_cmds`：`control_msgs/msg/JointJog`

当前训练数据对应的观测和动作：

- observation：右臂 6 个关节位置 + 6 个关节速度，共 12 维。
- action：右臂 6 个关节 desired position，共 6 维。
- obs_horizon：默认从 checkpoint/config 读取，当前训练为 2。
- pred_horizon：默认从 checkpoint/config 读取，当前训练为 16。

关节顺序：

```text
right_base_pitch_joint
right_shoulder_roll_joint
right_shoulder_yaw_joint
right_elbow_pitch_joint
right_wrist_pitch_joint
right_wrist_yaw_joint
```

由于模型输出是 desired position，而 Servo 接收 JointJog 速度命令，节点会在每个控制周期做转换：

```text
joint_delta = (policy_target_position - current_joint_position) * action_scale
joint_delta = clip(joint_delta, +/- max_joint_delta)
joint_velocity = clip(joint_delta * control_rate, +/- max_velocity)
```

## 训练命令

示例：

```bash
conda activate train-gpu
cd ~/humanoid/RL_training/Imitation_Learning
python train_diffusion_policy.py \
  --dataset ../../data/processed/real_robot/training_episodes \
  --checkpoint-dir ../../data/checkpoints/diffusion_policy \
  --epochs 500
```

如果训练脚本保存的是 `latest.pt`，请把 launch 参数指向 `latest.pt`；如果你已经生成 `best.pt`，指向 `best.pt`。

## 环境检查

方案 A 是让 ROS 2 Python 环境直接 import torch 和 rclpy：

```bash
python3 -c "import rclpy; print('rclpy ok')"
python3 -c "import torch; print(torch.__version__, torch.cuda.is_available())"
```

如果 `rclpy` 正常但 `torch` 失败，需要在 ROS 2 使用的 Python 环境里安装 PyTorch。例如 CPU 版可参考：

```bash
python3 -m pip install torch --index-url https://download.pytorch.org/whl/cpu
```

CUDA 版请按当前 CUDA 驱动选择 PyTorch 官方对应 wheel。不要只装在 `train-gpu` conda 环境里，除非该环境也能正常 `import rclpy`。

## 编译

```bash
cd ~/humanoid
colcon build --symlink-install --packages-select robot_servo_control
source install/setup.bash
ros2 pkg executables robot_servo_control | grep diffusion_policy_servo_node
```

## 真机 dry run

先启动真机控制栈：

```bash
ros2 launch robot_commander robot_moveit.launch.xml use_simulation:=false
```

再启动 MoveIt Servo：

```bash
ros2 launch robot_servo_control servo_control.launch.py
```

如果你的本地文件名是 `servo_control.py`，按本地命令启动即可；当前仓库文件名是 `servo_control.launch.py`。

启动推理 dry run：

```bash
ros2 launch robot_servo_control diffusion_policy_servo.launch.py \
  checkpoint_path:=/home/arthur/humanoid/data/checkpoints/diffusion_policy/best.pt \
  dry_run:=true
```

dry run 会加载模型、订阅 `/joint_states`、持续打印 obs/action summary，但不会发布 JointJog，也不会调用 Servo start 服务。

## 真机低速执行

确认 dry run 输出稳定、无 NaN、obs/action 维度正确后，再显式关闭 dry run：

```bash
ros2 launch robot_servo_control diffusion_policy_servo.launch.py \
  checkpoint_path:=/home/arthur/humanoid/data/checkpoints/diffusion_policy/best.pt \
  dry_run:=false \
  action_scale:=0.2 \
  max_joint_delta:=0.03 \
  max_velocity:=0.3
```

建议第一次真机执行使用较小的 `action_scale`、`max_joint_delta` 和 `max_velocity`。

## 常用参数

```text
checkpoint_path        checkpoint 绝对路径
device                 cuda 或 cpu，默认 cuda
control_rate           控制频率，默认 10 Hz
obs_horizon            0 表示从 checkpoint 读取
action_horizon         每次预测后按顺序执行几个 action，默认 1
action_scale           动作缩放，默认 1.0
dry_run                默认 true，false 才发布真机命令
max_joint_delta        单周期关节目标差值限幅，默认 0.05 rad
max_velocity           JointJog 速度限幅，默认 0.5 rad/s
use_normalizer         默认 true
action_type            默认 target_position，可选 joint_delta/joint_velocity
joint_state_timeout    /joint_states 超时阈值，默认 0.5 s
startup_delay_sec      启动后延迟发布，默认 3 s
emergency_stop         参数级急停，默认 false
```

## 安全机制

- `dry_run` 默认 true。
- `dry_run:=false` 时才会调用 `/right_servo/moveit_servo/start_servo` 并发布命令。
- 节点启动后前 3 秒不发布。
- `/joint_states` 超过 0.5 秒未更新会停止发布并发送一次零速度。
- obs buffer 不完整时停止发布。
- 模型输出 NaN/Inf 时停止发布。
- 动作超过阈值会 clip，并打印 warning。
- 支持 `/diffusion_policy_servo/emergency_stop` 话题急停。

## 常见错误

找不到 checkpoint：

```text
checkpoint_path not found
```

确认传入的是绝对路径，且 ROS 终端能访问该文件。

checkpoint key 不匹配：

```text
Unsupported checkpoint format
```

节点兼容 `model_state_dict`、`state_dict`、`model` 和直接 state_dict。其他格式需要补适配。

obs_dim 不匹配：

```text
state_dim mismatch
```

当前节点只实现右臂 joint position + velocity 的 12 维观测。如果 checkpoint 使用图像、末端位姿或物体位姿，需要扩展 observation builder。

action_dim 不匹配：

```text
action_dim mismatch
```

当前默认只控制右臂 6 关节。请确认 checkpoint 的 action_dim 是 6，且关节顺序一致。

import torch 失败：

```bash
python3 -c "import torch; print(torch.__version__)"
```

在 ROS 2 Python 环境安装 torch。

import rclpy 失败：

```bash
python3 -c "import rclpy; print('rclpy ok')"
```

确认已经 source ROS 2 Humble，例如：

```bash
source /opt/ros/humble/setup.bash
source ~/humanoid/install/setup.bash
```

`/joint_states` 没数据：

```bash
ros2 topic hz /joint_states
ros2 topic echo /joint_states --once
```

确认 `robot_moveit.launch.xml use_simulation:=false` 已启动并且控制器 active。

servo topic 没订阅者：

```bash
ros2 topic info /right_servo/moveit_servo/delta_joint_cmds
```

确认 `robot_servo_control` 的 Servo launch 已启动。

模型输出 NaN：

降低 `action_scale`，检查 checkpoint、normalizer 和输入 obs 是否来自同一训练数据格式。

机器人没有动作：

确认 `dry_run:=false`、Servo start 成功、topic 有订阅者、`/joint_states` 新鲜，并检查 `max_joint_delta` / `max_velocity` 是否过小。

## 以后扩展

当前 checkpoint 使用低维关节状态，不使用 end-effector pose、object pose 或 image observation。后续如果训练图像策略，应在节点里增加 camera topic 订阅、图像预处理、时间同步，并保证 shape、归一化和训练完全一致。
