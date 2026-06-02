# PushCube Dataset and Training

The PushCube pipeline keeps the existing Isaac teleoperation and recording scene intact: the table layout, robot base pose, fixed gripper, fixed left arm, right-arm-only motion, and cube geometry are reused rather than redesigned. The HDF5 dataset remains the canonical interface between recording, analysis, augmentation, BC training, evaluation, and rendering.

## Why Both 2D and 3D Outputs Exist

2D matplotlib plots are for quantitative trajectory analysis. They make it easy to compare all demonstrations, inspect cube and end-effector motion on the table plane, compute summary statistics, and produce thesis figures such as overlays, histograms, and action distributions. These plots are generated offline without Isaac Sim and are suitable for:

- Trajectory overlay comparisons across all demos
- Statistical distributions (episode length, final distance, cube motion, action)
- Per-episode trajectory inspection with annotations
- Dataset quality assessment and filtering decisions
- Publication-ready figures with high DPI and clear axis labels

3D IsaacSim videos are for qualitative demonstration. They show the real scene geometry, robot motion, cube contact behavior, and camera perspective that top-down plots cannot capture. These renders are appropriate for:

- Thesis defense presentations and video supplements
- Failure-case inspection where 3D context matters
- Demonstrating policy behavior to non-technical audiences
- Verifying that replay matches the original recording fidelity

## Original vs Augmented Data

The original dataset still contains 10 real manual demonstrations. Augmentation expands that dataset to roughly 100 trajectories to stabilize early supervised learning, but the augmented episodes must never be described as additional human demonstrations. In writing, distinguish clearly between:

- Real manual demos: 10
- Augmented synthetic demos: approximately 90
- Total training set after augmentation: approximately 100

Augmentation methods applied:
1. XY translation: small table-plane shifts (±0.03 m) applied uniformly to cube, target, and ee trajectories
2. Time resampling: stretch/compress trajectory length by factor 0.85–1.15
3. Small noise: Gaussian noise on cube_xy (≤0.003), ee_xy (≤0.003), action (≤0.02)
4. Action recomputation: actions derived from augmented ee_xy displacements, clipped to [-1, 1]
5. Mirror: disabled by default (right-arm task, no valid left-right symmetry)

## Training Strategy

Behavior cloning is the baseline and should be validated before any more ambitious policy class is attempted. The expected order is:

1. Inspect and analyze the original HDF5 dataset.
2. Identify poor-quality or low-motion trajectories.
3. Augment the validated dataset.
4. Train and evaluate a simple BC MLP on the low-dimensional observation/action format.
5. Confirm that evaluation rollouts, metrics, and rendered videos are consistent.

Only after the BC and evaluation pipeline is working should diffusion policy be attempted. Diffusion policy adds substantially more modeling complexity, so it should not be the first point of failure in the pipeline.

## BC Model Architecture

MLP baseline:
- Input: obs_dim=13 (normalized by training set mean/std)
- Hidden: 256 → 256 → 128 (ReLU activations)
- Output: action_dim=2 (normalized planar delta xy)
- Loss: MSE
- Optimizer: Adam, lr=1e-3
- Validation split: 20% of episodes

## Pipeline Command Order

The full pipeline should be executed in this order. Do not skip analysis before training.

### Step 1: Analyze original 10 demos

```bash
/home/arthur/isaacsim/python.sh pushcube_isaac_v0/analyze_pushcube_dataset.py \
  --input data/simulation/isaac_rl/datasets/pushcube_manual_v1.hdf5 \
  --output-dir plots/pushcube_manual_v1 \
  --all
```

### Step 2: Generate quality report

```bash
/home/arthur/isaacsim/python.sh pushcube_isaac_v0/report_pushcube_dataset_quality.py \
  --input data/simulation/isaac_rl/datasets/pushcube_manual_v1.hdf5 \
  --output plots/pushcube_manual_v1/dataset_quality_report.md
```

### Step 3: Render 3D videos from original demos

```bash
/home/arthur/isaacsim/python.sh pushcube_isaac_v0/render_pushcube_dataset_3d.py \
  --input data/simulation/isaac_rl/datasets/pushcube_manual_v1.hdf5 \
  --output-dir plots/pushcube_manual_v1/videos_3d \
  --episodes 0 1 2 \
  --camera presentation \
  --fps 20
```

### Step 4: Augment to 100

```bash
/home/arthur/isaacsim/python.sh pushcube_isaac_v0/augment_pushcube_dataset.py \
  --input data/simulation/isaac_rl/datasets/pushcube_manual_v1.hdf5 \
  --output data/simulation/isaac_rl/datasets/pushcube_manual_v1_aug100.hdf5 \
  --target-count 100 \
  --seed 42
```

### Step 5: Analyze augmented dataset

```bash
/home/arthur/isaacsim/python.sh pushcube_isaac_v0/analyze_pushcube_dataset.py \
  --input data/simulation/isaac_rl/datasets/pushcube_manual_v1_aug100.hdf5 \
  --output-dir plots/pushcube_manual_v1_aug100 \
  --all
```

### Step 6: Train BC

```bash
python3 pushcube_isaac_v0/train_pushcube_bc.py \
  --dataset data/simulation/isaac_rl/datasets/pushcube_manual_v1_aug100.hdf5 \
  --output data/simulation/isaac_rl/runs/pushcube_bc_aug100_v1 \
  --epochs 300 \
  --batch-size 256 \
  --lr 1e-3 \
  --val-split 0.2 \
  --seed 42
```

### Step 7: Plot training curves

```bash
python3 pushcube_isaac_v0/plot_training_curves.py \
  --run-dir data/simulation/isaac_rl/runs/pushcube_bc_aug100_v1
```

### Step 8: Evaluate BC

```bash
/home/arthur/isaacsim/python.sh pushcube_isaac_v0/eval_pushcube_policy.py \
  --checkpoint data/simulation/isaac_rl/runs/pushcube_bc_aug100_v1/model.pt \
  --episodes 20 \
  --headless false \
  --layout-preset opposite_edges \
  --target-size 0.22 \
  --save-video \
  --output-dir data/simulation/isaac_rl/runs/pushcube_bc_aug100_v1/eval
```

### Step 9: Render 3D eval videos

```bash
/home/arthur/isaacsim/python.sh pushcube_isaac_v0/render_pushcube_dataset_3d.py \
  --input data/simulation/isaac_rl/runs/pushcube_bc_aug100_v1/eval/eval_rollouts.hdf5 \
  --output-dir figure/simulation/isaac_rl/runs/pushcube_bc_aug100_v1/eval/videos_3d \
  --episodes 0 1 2 3 4 \
  --camera presentation \
  --fps 20
```
