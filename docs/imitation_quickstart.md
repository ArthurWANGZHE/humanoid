# Imitation Pipeline Quickstart

## Build

```bash
cd ~/humanoid
colcon build --packages-select robot_imitation_pipeline robot_keyboard_control robot_commander
source install/setup.bash
```

## Start The Existing Robot Stack

Use the current real-robot path that already works for joint-space teleoperation:

```bash
ros2 launch robot_commander robot_moveit.launch.xml use_simulation:=false
```

If needed, start the commander separately:

```bash
ros2 run robot_commander commander
```

## Start Joint Teleop

The verified expert path is still joint-space target control:

```bash
ros2 run robot_keyboard_control joint_control
```

## Start The Recorder

The existing recorder entrypoint is unchanged:

```bash
ros2 launch robot_imitation_pipeline demo_recorder.launch.py
```

Default recording config lives at [recording.yaml](/home/arthur/humanoid/src/robot_imitation_pipeline/config/recording.yaml).

If a YAML file is passed to ROS 2 with `--params-file`, it must use ROS 2 parameter-file format:

```yaml
demo_recorder:
  ros__parameters:
    key: value
```

Do not pass arbitrary nested application config YAML as a ROS 2 params file.

Default output:

```text
~/humanoid/data/imitation_raw
```

## Record One Episode

Start:

```bash
ros2 run robot_imitation_pipeline demo_control start
```

Stop and mark success:

```bash
ros2 run robot_imitation_pipeline demo_control stop --success
```

Stop and mark failure:

```bash
ros2 run robot_imitation_pipeline demo_control stop --failure
```

The recorder is listen-only. It does not publish robot commands.

## Validate

Validate a dataset root:

```bash
ros2 run robot_imitation_pipeline validate_demo data/imitation_raw
```

Validate one episode:

```bash
ros2 run robot_imitation_pipeline validate_demo data/imitation_raw/episode_000001
```

Local wrapper:

```bash
python3 tools/validate_demo.py data/imitation_raw/episode_000001
```

The validator checks:

- required files
- shape consistency
- NaN and Inf
- timestamp monotonicity
- average sampling rate against `control_rate_hz`
- image readability
- metadata dimension consistency
- `success.json.valid_for_training`

Legacy episodes are reported as `LEGACY/WARN` instead of failing with unclear errors.

## Inspect One Episode

```bash
python3 tools/inspect_demo.py --episode data/imitation_raw/episode_000001
```

This prints a compact summary of metadata, timestamps, state, action, and images.

## Create A Fake Demo For Testing

```bash
python3 tools/make_fake_demo.py --output data/imitation_raw/episode_fake_000001
python3 tools/validate_demo.py data/imitation_raw/episode_fake_000001
```

## Replay

Dry-run replay is unchanged:

```bash
ros2 run robot_imitation_pipeline replay_demo data/imitation_raw/episode_000001
```

Real robot replay remains explicitly gated:

```bash
cp src/robot_imitation_pipeline/config/replay.yaml /tmp/replay_execute.yaml
# Edit /tmp/replay_execute.yaml and set execute_on_robot: true
ros2 run robot_imitation_pipeline replay_demo data/imitation_raw/episode_000001 \
  --config /tmp/replay_execute.yaml \
  --execute-on-robot
```

## Convert For Training

```bash
ros2 run robot_imitation_pipeline convert_to_hdf5 data/imitation_raw \
  --output-dir data/imitation_converted
```

The converter still writes the existing BC-friendly outputs and now also preserves `episode_ends` for future sequence models.

## Important Rule

The strict raw format uses:

```text
obs[i], robot_state[i] -> action[i]
```

If image count is not equal to `T`, joint order is wrong, or gripper commands are missing, do not train on that episode until validation is clean.
