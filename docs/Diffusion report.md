# Humanoid Project

This repository contains the codebase for a humanoid robotics project.

It includes the core system implementation as well as supporting resources developed during the project, such as documentation, tools, and testing code.

## Repository Structure

- `src/`  
  Core ROS 2 source code and packages intended for direct build and execution (robot_description, robot_commander, moveit config, hardware interface, simulation, etc.).

- `RL_training/`  
  Reinforcement learning training code for the humanoid brick-picking task. Contains two independent sub-projects:
  - `Isaac_RL/` — Isaac Sim + Stable-Baselines3 PPO training pipeline (standalone, no ROS dependency).
  - `Mujoco_RL/` — MuJoCo + Stable-Baselines3 PPO baseline using a right-arm subset.

- `docs/`  
  Project documentation, including environment setup, control instructions, calibration guides, and RL training notes.

- `resource/`  
  Auxiliary tools and experimental code used during development (e.g. motor driver testing).

## Quick Links

| Topic | Doc |
|---|---|
| Simulation environment setup | [docs/仿真环境配置.md](docs/仿真环境配置.md) |
| Robot commander & MoveIt | [docs/robot_commander_README.md](docs/robot_commander_README.md) |
| Keyboard teleoperation | [docs/keyboard_control_README.md](docs/keyboard_control_README.md) |
| Imitation data logging | [docs/imitation_data_recording.md](docs/imitation_data_recording.md) |
| Camera startup for recording | [docs/imitation_data_recording.md](docs/imitation_data_recording.md) |
| Servo control | [docs/伺服控制说明.md](docs/伺服控制说明.md) |
| Hand-eye calibration | [docs/手眼标定说明.md](docs/手眼标定说明.md) |
| RL training overview | [docs/RL_training.md](docs/RL_training.md) |
| Known issues | [docs/Problem.md](docs/Problem.md) |

## Logging Imitation Data

Build and source the recorder packages:

```bash
cd ~/humanoid
colcon build --packages-select robot_imitation_pipeline robot_keyboard_control robot_commander
source install/setup.bash
```

Start the wrist camera first. This topic is required by the recorder:

```bash
ros2 launch robot_imitation_pipeline start_recording_cameras.launch.py \
  use_wrist_camera:=true \
  wrist_video_device:=/dev/video0 \
  use_head_camera:=false
```

If the RealSense head camera is available, you can enable it, but recording must still work with the wrist camera only:

```bash
ros2 launch robot_imitation_pipeline start_recording_cameras.launch.py \
  use_wrist_camera:=true \
  wrist_video_device:=/dev/video0 \
  use_head_camera:=true \
  head_camera_namespace:=head_camera
```

Check that the required wrist topic exists before starting the recorder:

```bash
ros2 topic list | grep image
ros2 topic hz /right_wrist_camera/image_raw
python3 RL_training/Imitation_Learning/tools/check_camera_topics.py
```

Start the real-robot stack and recorder:

```bash
ros2 launch robot_commander robot_moveit.launch.xml use_simulation:=false
ros2 launch robot_imitation_pipeline demo_recorder.launch.py
```

Record one episode:

```bash
ros2 run robot_imitation_pipeline demo_control start
ros2 run robot_imitation_pipeline demo_control stop --success
```

The recorder writes episodes under `data/imitation_raw`. To stop a failed run and keep it marked invalid for training:

```bash
ros2 run robot_imitation_pipeline demo_control stop --failure
```

If the RealSense does not start, set `use_head_camera:=false` and continue recording with the wrist camera only. See [docs/imitation_data_recording.md](docs/imitation_data_recording.md) for device listing, troubleshooting, and the full recorder workflow.

## Diffusion Policy 迁移评估

当前仓库里有两条和 diffusion policy 相关的路线：

- `RL_training/Imitation_Learning/train_diffusion_policy.py`、`RL_training/Imitation_Learning/policy.py` 和 `RL_training/Imitation_Learning/models/` 是一个本地简化版的低维 state-action diffusion baseline。它可以直接读取当前的 episode 目录，例如 `data/episode_*` 或 `data/imitation_raw/episode_*`，用 `robot_state.npy` 作为观测，用 `action.npy` 作为目标动作。
- `diffusion_policy/` 是原版 Diffusion Policy 代码库。它的真实机器人流程可以通过 UR RTDE 驱动 UR5/UR5e，用 RealSense 和 SpaceMouse 采集演示数据，训练视觉策略，再把策略部署回 UR 机械臂。

新放进来的 `diffusion_policy/` 很适合作为参考实现，但不能不改就直接跑在当前 humanoid 机器人上。它的真实机器人脚本默认假设硬件是 UR5/UR5e、控制接口是 TCP 位姿 RTDE、相机是 RealSense、遥操作是 SpaceMouse，任务动作空间也是 Push-T 风格的 TCP `x,y` 二维动作。当前机器人栈使用 ROS 2 topic、右臂关节命令、腕部/头部相机 topic，已经录下来的动作是 7 维的右臂关节加夹爪命令。

### 推荐复现路线

建议先从风险最低、最贴合现有数据的版本开始，再逐步加视觉：

1. 先复现当前低维 state-action baseline。
   - 观测：`robot_state.npy`，目前包含右臂关节位置、右臂关节速度和右夹爪状态。
   - 动作：`action.npy`，目前包含 6 个右臂关节命令和 1 个夹爪命令。
   - 这是第一步最合适的检查点，因为它和当前 recorder、ROS 2 控制接口最匹配。

2. 再迁移到原版 Diffusion Policy 的 dataset/workspace 体系。
   - 新增一个 humanoid dataset adapter，用来读取现有 episode 文件夹。
   - 新增 task config，在 `shape_meta` 里定义 `robot_state` 和 7 维 action。
   - 数据适配确认无误后，再用 `diffusion_policy/train.py` 训练。

3. 低维闭环稳定后再加入图像观测。
   - 先只用 `right_wrist_camera`，后续再考虑加入 `head_camera`。
   - 录制、训练、部署时必须保持图像尺寸、时间戳对齐方式和 camera key 一致。
   - 第一版只接一个相机，调试面会小很多。

4. 最后加实时 ROS 2 策略推理。
   - 原版 `eval_real_robot.py` 只能当参考，不建议作为当前机器人的部署节点。
   - 实际部署应该走现有 ROS 2 topic：`/right_joint_command` 和 `/open_right_gripper`。
   - 在预测结果确认合理之前，保持 `publish_commands:=false` 的 dry-run 模式。

### 当前操作流程

录制演示数据：

```bash
cd ~/humanoid
colcon build --packages-select robot_imitation_pipeline robot_keyboard_control robot_commander
source install/setup.bash

ros2 launch robot_commander robot_moveit.launch.xml use_simulation:=false
ros2 launch robot_imitation_pipeline start_recording_cameras.launch.py \
  use_wrist_camera:=true \
  wrist_video_device:=/dev/video0 \
  use_head_camera:=false
ros2 launch robot_imitation_pipeline demo_recorder.launch.py
```

每一条 episode 的录制流程：

```bash
ros2 run robot_imitation_pipeline demo_control start
ros2 run robot_imitation_pipeline demo_control stop --success
```

验证和检查数据：

```bash
ros2 run robot_imitation_pipeline validate_demo data/imitation_raw
python3 RL_training/Imitation_Learning/tools/inspect_demo.py --episode data/imitation_raw/episode_000001
```

训练当前低维 state-action diffusion baseline：

```bash
python RL_training/Imitation_Learning/train_diffusion_policy.py \
  --dataset data/imitation_raw \
  --checkpoint-dir logs/diffusion_policy/checkpoints \
  --obs-horizon 2 \
  --pred-horizon 16 \
  --epochs 50
```

上真机前先做离线评估：

```bash
python RL_training/Imitation_Learning/eval_policy.py \
  --checkpoint logs/diffusion_policy/checkpoints/latest.pt \
  --dataset data/imitation_raw \
  --max-samples 100
```

先以 dry-run 模式启动策略推理：

```bash
ros2 launch robot_imitation_pipeline policy_inference.launch.py
```

只有当 dry-run 输出的命令幅度受限、方向基本正确后，才应该在 `src/robot_imitation_pipeline/config/policy_inference.yaml` 里打开 `publish_commands`。

### 使用原版包需要新增的内容

原版 Diffusion Policy 包默认期望 Push-T 的真实数据格式，也就是 `replay_buffer.zarr + videos/`，或者为任务单独写 dataset class。要接到当前机器人上，建议新增这些内容：

- `diffusion_policy/diffusion_policy/dataset/humanoid_image_dataset.py`：读取 `data/imitation_raw/episode_*`，加载 `robot_state.npy`、`action.npy`，以及 `obs/right_wrist_camera` 下的图像帧。
- `diffusion_policy/diffusion_policy/config/task/humanoid_image.yaml`：定义 `shape_meta`，例如一个 RGB 腕部相机、可选的低维 `robot_state`、以及 7 维 action。
- 如果先不做图像训练，可以先加一个低维版本，例如 `humanoid_lowdim.yaml`。
- 新增一个 ROS 2 推理桥接节点，用来加载训练好的原版 policy，并发布到现有控制 topic。部署时它应该替代原版 UR RTDE 专用的 `RealEnv` 路径。
- 如果希望完全沿用原版 `ReplayBuffer` 工作流，可以额外做一个转换工具，把每条 episode 转成 `replay_buffer.zarr` 和同步的 `videos/<episode>/<camera>.mp4`。

### 接口映射

| 原版真实机器人代码 | 当前 humanoid 栈 | 需要适配的点 |
|---|---|---|
| `RTDEInterpolationController` 发送 UR TCP waypoint | ROS 2 发布关节/夹爪命令 | 替换 controller，改成 ROS 2 publisher，或复用现有 inference node |
| `ActualTCPPose` / `TargetTCPPose` | `/joint_states` 里的右臂关节和夹爪 | 先决定 policy 输出关节空间动作还是笛卡尔空间动作 |
| RealSense SDK 输出的 `camera_0`、`camera_1` 等 | `/right_wrist_camera/image_raw`，可选 `/head_camera/color/image_raw` | 保持 camera key 和帧对齐方式一致 |
| Push-T action shape `[2]` | 当前右臂 action shape `[7]` | 更新 `shape_meta`、dataset、normalizer 和部署侧限幅 |
| SpaceMouse 示教 | 键盘/ROS 遥操作加 recorder | 除非明确要加 SpaceMouse，否则先保留当前 recorder |

### 数据集要求

需要为目标任务重新采集数据，且采集时的任务、相机位置、光照、夹爪行为和 action 接口都要和部署时一致。UR5e Push-T 的数据不能可靠迁移到当前 humanoid 右臂，因为机器人本体、动作语义、相机视角和任务分布都不一样。

建议的起步数据量：

- 非常固定的单一行为，先采 30-50 条成功 episode。
- 如果物体位置、光照、接近方向有变化，建议 100 条以上成功 episode。
- 失败或中断的数据保留也可以，但要标记 `valid_for_training: false`。
- 每条可训练 episode 至少要有 `obs_horizon + pred_horizon - 1` 个样本；当前默认值下至少需要 17 帧。
- 采集和部署使用同样的控制频率，目前是 10 Hz。

### 安全检查

- 默认保持 dry-run 推理，不直接发控制命令。
- 每一步发布前都要限制关节最大变化量。
- 拒绝过期的 `/joint_states` 和过期相机帧。
- 真正执行前增加工作空间限制或关节限位检查。
- 每次真实机器人评估时，急停必须在手边。
- 顺序建议是：仿真或 replay 测试、机器人上电但不发命令、极小命令限幅真机测试、逐步放宽限幅。

### 近期实现计划

1. 清理现有录制数据，重新采几条有效且非空的 episode。
2. 先训练并离线评估当前低维 diffusion baseline。
3. 新增一个带限幅的 ROS 2 diffusion 推理节点，或者扩展当前 inference node，让它能加载 `policy.py` 的 diffusion checkpoint。
4. 低维部署可用后，再给原版 `diffusion_policy/` 增加 humanoid dataset 和 task config。
5. 接入 `right_wrist_camera` 图像观测并重新训练。
6. 最后再考虑用完整原版 Diffusion Policy workspace 替换当前简化 baseline，作为正式实验路线。

## Notes

Some contents in this repository are intended for development and testing purposes and are not part of the final runtime system.

## Contributors

Zhangyu Fan: Hardware

Tongbin Hu: Software

Zhe Wang: Software

Wu Jie: Software
