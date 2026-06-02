# PushCube Training Pipeline

This pipeline keeps the current PushCube teleop scene and robot setup unchanged:

1. Record manual demos with `pushcube_isaac_v0/record_manual_pushcube.py`.
2. Inspect the dataset with `pushcube_isaac_v0/inspect_pushcube_dataset.py`.
3. Plot trajectories and summary charts with `pushcube_isaac_v0/plot_pushcube_dataset.py`.
4. Filter the dataset with `pushcube_isaac_v0/filter_pushcube_dataset.py`.
5. Train the behavior-cloning baseline with `pushcube_isaac_v0/train_pushcube_bc.py`.
6. Evaluate the BC policy with `pushcube_isaac_v0/eval_pushcube_policy.py`.
7. Only after the BC pipeline is working, add a diffusion-policy trainer on the same `obs_dim=13` and `action_dim=2` dataset format.

## Commands

Record 5 test demos:

```bash
/home/arthur/isaacsim/python.sh pushcube_isaac_v0/record_manual_pushcube.py \
  --headless false \
  --show-ranges \
  --input-backend terminal \
  --control-mode joint \
  --layout-preset opposite_edges \
  --target-size 0.22 \
  --expand-right-arm-limits 0.5 \
  --joint-alpha 0.2 \
  --quiet-warnings \
  --num-episodes 5 \
  --output data/simulation/isaac_rl/datasets/pushcube_manual_test.hdf5
```

Inspect:

```bash
python3 pushcube_isaac_v0/inspect_pushcube_dataset.py \
  --input data/simulation/isaac_rl/datasets/pushcube_manual_test.hdf5
```

Plot:

```bash
python3 pushcube_isaac_v0/plot_pushcube_dataset.py \
  --input data/simulation/isaac_rl/datasets/pushcube_manual_test.hdf5 \
  --output-dir plots/pushcube_manual_test \
  --all
```

Record the full batch:

```bash
/home/arthur/isaacsim/python.sh pushcube_isaac_v0/record_manual_pushcube.py \
  --headless false \
  --show-ranges \
  --input-backend terminal \
  --control-mode joint \
  --layout-preset opposite_edges \
  --target-size 0.22 \
  --expand-right-arm-limits 0.5 \
  --joint-alpha 0.2 \
  --quiet-warnings \
  --num-episodes 50 \
  --output data/simulation/isaac_rl/datasets/pushcube_manual_v1.hdf5
```

Filter:

```bash
python3 pushcube_isaac_v0/filter_pushcube_dataset.py \
  --input data/simulation/isaac_rl/datasets/pushcube_manual_v1.hdf5 \
  --output data/simulation/isaac_rl/datasets/pushcube_manual_v1_success_quality.hdf5 \
  --success-only \
  --min-cube-motion 0.03 \
  --max-final-distance 0.15
```

Train:

```bash
python3 pushcube_isaac_v0/train_pushcube_bc.py \
  --dataset data/simulation/isaac_rl/datasets/pushcube_manual_v1_success_quality.hdf5 \
  --output data/simulation/isaac_rl/runs/pushcube_bc_v1 \
  --epochs 200
```

Evaluate:

```bash
/home/arthur/isaacsim/python.sh pushcube_isaac_v0/eval_pushcube_policy.py \
  --checkpoint data/simulation/isaac_rl/runs/pushcube_bc_v1/model.pt \
  --episodes 20 \
  --headless false \
  --layout-preset opposite_edges \
  --target-size 0.22 \
  --hold-open
```
