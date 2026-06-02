# PushCube2D

Minimal 2D PushCube imitation-learning pipeline. A circular pusher moves on a
unit-square table and pushes a square cube toward a target center/region.

This first version is deliberately simple and debuggable:

- observation: `[pusher_x, pusher_y, cube_x, cube_y, cube_theta, goal_x, goal_y]`
- action: normalized pusher delta/velocity `[dx, dy]` in `[-1, 1]`
- success: `||cube_xy - goal_xy|| < success_threshold`
- physics: the pusher moves with clipping; when its circle touches the cube,
  the cube moves in the pusher direction
- yaw: included in the API, fixed at zero by default in v1

## Install

From the repository root:

```bash
pip install -r pushcube2d/requirements.txt
```

`pygame` and `gymnasium` are optional at runtime. Headless data collection,
training, validation, and plotting only require NumPy and Matplotlib.

## Quick Start

Collect scripted demonstrations:

```bash
python -m pushcube2d.collect_data --episodes 100 --out data/pushcube2d/demo.npz --render false
```

The default `near` reset distribution starts the pusher near the cube with a
fairly wide angular spread, so scripted success should be high but not
guaranteed. Use `aligned` for clean expert data, or `random` for a harder stress
test:

```bash
python -m pushcube2d.collect_data --episodes 100 --reset-mode aligned --out data/pushcube2d/demo_aligned.npz
python -m pushcube2d.collect_data --episodes 100 --reset-mode random --out data/pushcube2d/demo_random.npz
```

For custom resets or noisy experiments, you can still keep only successful
scripted rollouts:

```bash
python -m pushcube2d.collect_data --episodes 100 --success-only true --out data/pushcube2d/demo_success.npz
```

Validate the dataset:

```bash
python -m pushcube2d.check_dataset data/pushcube2d/demo.npz
```

Visualize dataset sanity plots:

```bash
python -m pushcube2d.visualize_dataset data/pushcube2d/demo.npz --out outputs/pushcube2d/demo_summary.png
```

Train behavior cloning:

```bash
python -m pushcube2d.train_bc --dataset data/pushcube2d/demo.npz --out models/pushcube2d/bc_demo.npz
```

Train a larger model with experiment logs:

```bash
python -m pushcube2d.train_bc \
  --dataset data/pushcube2d/demo_aligned.npz \
  --out models/pushcube2d/bc_large.npz \
  --run-name bc_large_aligned \
  --epochs 300 \
  --hidden-dim 128 \
  --num-layers 4 \
  --activation silu \
  --batch-size 256 \
  --lr 1e-3 \
  --lr-schedule cosine \
  --warmup-epochs 5 \
  --weight-decay 1e-5 \
  --grad-clip 5.0 \
  --success-only-data true \
  --tensorboard true
```

Training logs are written to `logs/pushcube2d/<run-name>/`:

- `config.json`: full hyperparameter config
- `metrics.csv`: epoch-wise metrics for plotting
- `metrics.jsonl`: machine-readable metric stream
- TensorBoard event files when `--tensorboard true`

By default, `train_bc.py` uses only successful episodes
(`--success-only-data true`). This is usually what you want for BC expert
training because long failed rollouts can dominate the action regression loss.
Use `--success-only-data false` only when you intentionally want to study mixed
success/failure datasets.

For a full experiment after changing the scripted expert or reset distribution,
run the PowerShell pipeline. It recollects all three datasets by default,
trains both `all_data` and `success_only` BC variants for each dataset, evaluates
every model on `aligned`, `near`, and `random`, then writes comparison tables and
figures:

```powershell
.\run_all.ps1 -CollectEpisodes 300 -Epochs 300
```

For a single dataset:

```powershell
.\run_all.ps1 -Only demo_random -CollectEpisodes 300 -Epochs 300
```

The aggregate outputs are written under
`outputs/pushcube2d/experiment_compare/`, including:

- `pushcube2d_bc_eval_summary.csv`
- `pushcube2d_bc_success_only_delta.csv`
- `pushcube2d_bc_eval_summary.tex`
- success-rate and final-distance heatmaps in PNG/PDF/SVG formats

Use `-SkipRecollect` to reuse existing datasets, and avoid
`-CollectSuccessOnly` when comparing `all_data` vs `success_only` training.

Launch TensorBoard:

```bash
tensorboard --logdir logs/pushcube2d
```

Create publication-style training figures:

```bash
python -m pushcube2d.visualize_training \
  --log-dir logs/pushcube2d/bc_large_aligned \
  --out-prefix outputs/pushcube2d/bc_large_aligned \
  --formats png,pdf,svg
```

Evaluate the trained policy:

```bash
python -m pushcube2d.eval_policy --model models/pushcube2d/bc_demo.npz --episodes 20 --render false
```

Create a trajectory plot:

```bash
python -m pushcube2d.visualize_trajectory --source scripted --out outputs/pushcube2d/scripted_rollout.png
python -m pushcube2d.visualize_trajectory --source dataset --dataset data/pushcube2d/demo.npz --episode 0
python -m pushcube2d.visualize_trajectory --source model --model models/pushcube2d/bc_demo.npz
```

Render an MP4 video:

```bash
python -m pushcube2d.visualize_video --source scripted --out outputs/pushcube2d/scripted_rollout.mp4
python -m pushcube2d.visualize_video --source dataset --dataset data/pushcube2d/demo.npz --episode 0 --out outputs/pushcube2d/demo_ep0.mp4
python -m pushcube2d.visualize_video --source model --model models/pushcube2d/bc_demo.npz --out outputs/pushcube2d/bc_rollout.mp4
```

Collect keyboard demonstrations:

```bash
python -m pushcube2d.keyboard_collect --episodes 5 --out data/pushcube2d/keyboard_demo.npz
```

Controls: WASD/arrows move the pusher, `N` starts the next episode, `Q` or
Esc quits and saves completed episodes.

## Dataset Format

Datasets are compressed `.npz` files with padded variable-length episodes:

- `obs`: `[N, T, 7]`
- `actions`: `[N, T, 2]`
- `states`: `[N, T, 8]`
- `rewards`: `[N, T]`
- `dones`: `[N, T]`
- `success`: `[N]`
- `episode_lengths`: `[N]`
- `metadata_json`: JSON string with env config, names, timestamp, and counts

Use `episode_lengths` to ignore padded timesteps.

## Files

- `env.py`: environment construction, step logic, state, rendering
- `scripted_policy.py`: geometric scripted demonstrator, optimized for the default reset distribution
- `collect_data.py`: scripted data collection CLI
- `keyboard_collect.py`: pygame keyboard teleop collector
- `dataset.py`: save/load/pad/validate helpers
- `check_dataset.py`: dataset validation CLI
- `train_bc.py`: pure-NumPy behavior cloning trainer
- `eval_policy.py`: rollout evaluation for trained policies
- `visualize_trajectory.py`: rollout/dataset/model trajectory plots
- `visualize_video.py`: rollout/dataset/model MP4 rendering
- `visualize_training.py`: paper-style training curves and hyperparameter tables
- `compare_experiments.py`: aggregate all-data vs success-only and reset-mode comparisons
- `visualize_dataset.py`: training-data sanity-check plots
