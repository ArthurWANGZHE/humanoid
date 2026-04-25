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
| Imitation data logging | [docs/imitation_quickstart.md](docs/imitation_quickstart.md) |
| Camera startup for recording | [docs/start_recording_cameras.md](docs/start_recording_cameras.md) |
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
python3 tools/check_camera_topics.py
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

If the RealSense does not start, set `use_head_camera:=false` and continue recording with the wrist camera only. See [docs/start_recording_cameras.md](docs/start_recording_cameras.md) for device listing and troubleshooting, and [docs/imitation_quickstart.md](docs/imitation_quickstart.md) for the full recorder workflow.

## Notes

Some contents in this repository are intended for development and testing purposes and are not part of the final runtime system.

## Contributors

Zhangyu Fan: Hardware

Tongbin Hu: Software

Zhe Wang: Software

Wu Jie: Software
