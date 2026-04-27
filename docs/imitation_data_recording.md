# 模仿学习数据录制说明

## 1. 编译与环境准备

```bash
cd ~/humanoid
colcon build --packages-select robot_imitation_pipeline robot_keyboard_control robot_commander
source install/setup.bash
```

## 2. 启动机器人控制栈

```bash
ros2 launch robot_commander robot_moveit.launch.xml use_simulation:=false
```

如需单独启动 commander：

```bash
ros2 run robot_commander commander
```

## 3. 启动键盘遥操作（专家示教）

```bash
ros2 run robot_keyboard_control joint_control
```

## 4. 启动相机

### 列出相机设备

```bash
v4l2-ctl --list-devices
ls -l /dev/video*
```

### 仅启动腕部 USB 相机

```bash
ros2 launch robot_imitation_pipeline start_recording_cameras.launch.py \
  use_wrist_camera:=true \
  wrist_video_device:=/dev/video0 \
  use_head_camera:=false
```

### 启动腕部 + RealSense 头部相机

```bash
ros2 launch robot_imitation_pipeline start_recording_cameras.launch.py \
  use_wrist_camera:=true \
  wrist_video_device:=/dev/video0 \
  use_head_camera:=true \
  head_camera_namespace:=head_camera
```

### 相机话题

- 必需：`/right_wrist_camera/image_raw`
- 可选：`/right_wrist_camera/camera_info`
- 可选：`/head_camera/color/image_raw`

检查话题是否正常：

```bash
ros2 topic list | grep image
ros2 topic hz /right_wrist_camera/image_raw
python3 tools/check_camera_topics.py
```

如果 RealSense 启动失败，仅使用腕部相机即可继续录制。

## 5. 启动录制器

```bash
ros2 launch robot_imitation_pipeline demo_recorder.launch.py
```

默认配置：`src/robot_imitation_pipeline/config/recording.yaml`  
默认输出目录：`~/humanoid/data/imitation_raw`

## 6. 录制一个 Episode

开始录制：

```bash
ros2 run robot_imitation_pipeline demo_control start
```

停止并标记成功：

```bash
ros2 run robot_imitation_pipeline demo_control stop --success
```

停止并标记失败：

```bash
ros2 run robot_imitation_pipeline demo_control stop --failure
```

录制器为只读模式，不会发布机器人控制指令。

## 7. 数据验证

验证整个数据集：

```bash
ros2 run robot_imitation_pipeline validate_demo data/imitation_raw
```

验证单个 episode：

```bash
ros2 run robot_imitation_pipeline validate_demo data/imitation_raw/episode_000001
```

或使用本地脚本：

```bash
python3 tools/validate_demo.py data/imitation_raw/episode_000001
```

验证内容包括：文件完整性、shape 一致性、NaN/Inf 检查、时间戳单调性、采样率、图像可读性、metadata 维度一致性。

## 8. 数据检查与调试

查看 episode 摘要：

```bash
python3 tools/inspect_demo.py --episode data/imitation_raw/episode_000001
```

生成假数据用于测试：

```bash
python3 tools/make_fake_demo.py --output data/imitation_raw/episode_fake_000001
python3 tools/validate_demo.py data/imitation_raw/episode_fake_000001
```

## 9. 数据回放

Dry-run 回放：

```bash
ros2 run robot_imitation_pipeline replay_demo data/imitation_raw/episode_000001
```

真机回放（需手动开启）：

```bash
cp src/robot_imitation_pipeline/config/replay.yaml /tmp/replay_execute.yaml
# 编辑 /tmp/replay_execute.yaml，设置 execute_on_robot: true
ros2 run robot_imitation_pipeline replay_demo data/imitation_raw/episode_000001 \
  --config /tmp/replay_execute.yaml \
  --execute-on-robot
```

## 10. 转换为训练格式

```bash
ros2 run robot_imitation_pipeline convert_to_hdf5 data/imitation_raw \
  --output-dir data/imitation_converted
```


## 11. 数据集格式

### Episode 目录结构

```text
data/imitation_raw/episode_000001/
  meta.json
  timestamps.npy
  robot_state.npy
  action.npy
  success.json
  joint_state_timestamps.npy
  joint_pos.npy
  joint_vel.npy
  actions.npy
  action_valid.npy
  gripper.npy
  right_wrist_camera_timestamps.npy
  right_wrist_camera_ros_timestamps.npy
  obs/
    right_wrist_camera/
      000000.jpg
      000001.jpg
      ...
```

### 对齐规则

```text
obs[i], robot_state[i] -> action[i]
```

每个帧索引 `i` 对应同一个控制步：`timestamps[i]`、`robot_state[i]`、`action[i]`、`obs/right_wrist_camera/{i:06d}.jpg`。

### Robot State

`robot_state.npy` shape: `(T, 13)`

- 右臂关节位置 (6)
- 右臂关节速度 (6)
- 右夹爪状态 (1)

### Action

`action.npy` shape: `(T, 7)`

- 右臂关节指令 (6)
- 右夹爪指令 (1)

控制模式：`joint_target`

### Metadata (meta.json)

包含：`control_mode`、`robot_state_dim`、`action_dim`、`right_arm_joint_names`、`action_names`、`control_rate_hz`、`image_size`、`camera_name`、`topic_names`、`start_time`、`end_time`、`task_name`、`robot_name`

### success.json

```json
{
  "success": true,
  "valid_for_training": true
}
```

## 12. 关键话题与关节名

### 控制话题

| 话题 | 类型 | 含义 |
| --- | --- | --- |
| `/left_joint_command` | `Float64MultiArray` | 左臂 6 关节目标 |
| `/right_joint_command` | `Float64MultiArray` | 右臂 6 关节目标 |
| `/neck_joint_command` | `Float64MultiArray` | 颈部 2 关节目标 [pitch, yaw] |
| `/open_left_gripper` | `Bool` | true=开, false=关 |
| `/open_right_gripper` | `Bool` | true=开, false=关 |
| `/joint_states` | `JointState` | 机器人状态反馈 |

### 关节顺序

1. left_base_pitch_joint
2. left_shoulder_roll_joint
3. left_shoulder_yaw_joint
4. left_elbow_pitch_joint
5. left_wrist_pitch_joint
6. left_wrist_yaw_joint
7. right_base_pitch_joint
8. right_shoulder_roll_joint
9. right_shoulder_yaw_joint
10. right_elbow_pitch_joint
11. right_wrist_pitch_joint
12. right_wrist_yaw_joint
13. neck_pitch_joint
14. neck_yaw_joint
15. left_gripper1_joint
16. right_gripper1_joint

## 13. 常见录制错误

- action off-by-one：`action[i]` 对应了 `obs[i-1]`
- 相机帧缺失：图像数量 ≠ T
- 关节顺序错误：保存的关节名与数组布局不匹配
- 夹爪指令缺失：有臂指令但无夹爪通道
- 话题未发布：joint_state / camera / command 话题无数据
