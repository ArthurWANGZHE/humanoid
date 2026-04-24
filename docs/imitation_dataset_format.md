# Imitation Dataset Format

The current raw episode format is designed to keep the existing behavior cloning pipeline working while making new recordings strict enough for future Diffusion Policy conversion.

## Raw Episode Tree

```text
dataset/raw/episode_000001/
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

The strict files for new recordings are:

- `timestamps.npy`
- `robot_state.npy`
- `action.npy`
- `success.json`
- `obs/<camera_name>/*.jpg`

Legacy compatibility files are still written for existing replay, conversion, and baseline BC tools:

- `joint_pos.npy`
- `joint_vel.npy`
- `actions.npy`
- `action_valid.npy`
- `gripper.npy`

## Alignment Rule

The required semantic contract is:

```text
obs[i], robot_state[i] -> action[i]
```

For every frame index `i`, these refer to the same sampled control step:

- `timestamps[i]`
- `robot_state[i]`
- `action[i]`
- `obs/right_wrist_camera/{i:06d}.jpg`

`action[i]` is the command that the recorder observed as the command to execute immediately after `obs[i]`. The recorder never shifts actions by one step.

## Robot State

For the current first strict format, `robot_state.npy` is right-arm only and has shape `(T, 13)`:

1. right arm joint positions, 6 values
2. right arm joint velocities, 6 values
3. right gripper state, 1 value

The exact ordering is recorded in `meta.json`:

- `right_arm_joint_names`
- `robot_state_names`
- `robot_state_dim`

## Action

`action.npy` has shape `(T, 7)`:

1. right arm joint command actually observed on the right-arm command topic, 6 values
2. right gripper command, 1 value

The action semantics are explicit in `meta.json`:

- `control_mode`
- `action_names`
- `action_dim`

Current default control mode is `joint_target`. If action semantics change later, `meta.json` must change with them.

## Metadata

Each episode `meta.json` records at least:

- `control_mode`
- `robot_state_dim`
- `action_dim`
- `right_arm_joint_names`
- `action_names`
- `control_rate_hz`
- `image_size`
- `camera_name`
- `topic_names`
- `start_time`
- `end_time`
- `task_name`
- `robot_name`

The file also includes the explicit alignment rule and camera path metadata.

## Success Labels

`success.json` contains:

```json
{
  "success": true,
  "valid_for_training": true
}
```

Older episodes may only have `success`. The validator reports those as legacy/incomplete instead of crashing.

## Why This Supports BC And Future Diffusion Policy

- Existing BC and replay tools still have access to the legacy arrays.
- New recordings now have a strict one-image-per-step raw format.
- Episode boundaries can be preserved during conversion via `episode_ends`.
- The raw data is now suitable for future `obs_horizon`, `pred_horizon`, and `action_horizon` windowing without guessing camera alignment from nearest timestamps.

## Common Recording Errors

- Off-by-one action: `action[i]` accidentally corresponds to `obs[i-1]`
- Missing camera frame: image count is not equal to `T`
- Wrong joint order: saved joint names do not match array layout
- Gripper command missing: arm commands exist but gripper action channel is absent
- Topic not publishing: joint state, camera, or command topic is stale or absent
- Legacy camera layout: images saved outside `obs/<camera_name>/`
