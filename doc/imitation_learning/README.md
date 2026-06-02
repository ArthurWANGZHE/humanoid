# Imitation Learning 代码组织

这个目录用于放置当前机器人模仿学习相关的训练、评估、数据检查和实验报告。它和 `Isaac_RL/`、`Mujoco_RL/` 并列，作为独立的 imitation learning 子项目维护。

## 目录结构

```text
RL_training/Imitation_Learning/
  README.md
  requirements.txt
  train_diffusion_policy.py
  eval_policy.py
  policy.py
  models/
    diffusion_policy.py
    imitation_dataset.py
  tools/
    check_camera_topics.py
    inspect_demo.py
    make_fake_demo.py
    validate_demo.py
    create_imitation_ppt.py
  config/
    cameras.yaml
  imitation_learning_report.md
  humanoid_imitation_learning_report.pptx
```

## 各部分职责

- `train_diffusion_policy.py`：训练低维 state-action diffusion baseline。输入是 episode 目录，默认读取 `robot_state.npy` 和 `action.npy`。
- `eval_policy.py`：离线评估训练好的 diffusion checkpoint，计算预测动作和数据集中真实动作之间的误差。
- `policy.py`：推理封装，提供 `DiffusionPolicyWrapper` 和 `act()`，后续 ROS 2 推理节点可以复用这里的接口。
- `models/diffusion_policy.py`：简化版 diffusion policy 网络结构。
- `models/imitation_dataset.py`：当前 episode 数据格式的 PyTorch Dataset，负责读取、切片、归一化和统计量保存。
- `tools/`：数据采集和检查工具，包括相机 topic 检查、episode 检查、假数据生成和报告生成。
- `config/`：imitation learning 相关配置。目前主要放相机配置。

## 推荐工作流

1. 用 ROS 2 recorder 采集数据到 `data/imitation_raw/episode_*`。
2. 用工具检查数据是否完整：

```bash
python RL_training/Imitation_Learning/tools/validate_demo.py data/imitation_raw
python RL_training/Imitation_Learning/tools/inspect_demo.py --episode data/imitation_raw/episode_000001
```

3. 训练低维 diffusion baseline：

```bash
python RL_training/Imitation_Learning/train_diffusion_policy.py \
  --dataset data/imitation_raw \
  --checkpoint-dir data/checkpoints/diffusion_policy \
  --obs-horizon 2 \
  --pred-horizon 16 \
  --epochs 50
```

4. 离线评估：

```bash
python RL_training/Imitation_Learning/eval_policy.py \
  --checkpoint data/checkpoints/diffusion_policy/latest.pt \
  --dataset data/imitation_raw \
  --max-samples 100
```

5. 确认离线结果合理后，再接入 ROS 2 inference 节点，并先保持 dry-run。

## 后续整理方向

- 增加 `ros_inference/` 或在 `src/robot_imitation_pipeline` 中扩展现有 inference node，用来加载 diffusion checkpoint。
- 增加 `datasets/` 或 `converters/`，把当前 episode 格式转成原版 Diffusion Policy 可用的 dataset。
- 增加 `configs/` 中的训练配置，统一管理 `obs_horizon`、`pred_horizon`、batch size、checkpoint 输出目录等参数。
- 图像版训练稳定后，把 `right_wrist_camera` 图像读取逻辑从工具脚本中抽出来，进入正式 dataset。
