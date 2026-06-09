# Robot Keyboard Control

Keyboard control package for humanoid robot with two control modes.

## Installation

```bash
cd ~/humanoid
colcon build --packages-select robot_keyboard_control
source install/setup.bash
```

## Usage

### Mode 1: Joint Control

Direct control of each joint (DOF 1-6 for arms, 2 for neck).

```bash
ros2 run robot_keyboard_control joint_control
```

**Key Mapping:**

Left Arm (1-6):
- `1` / `!` (Shift+1): Base pitch +/-
- `2` / `@` (Shift+2): Shoulder yaw +/-
- `3` / `#` (Shift+3): Shoulder roll +/-
- `4` / `$` (Shift+4): Elbow pitch +/-
- `5` / `%` (Shift+5): Wrist pitch +/-
- `6` / `^` (Shift+6): Wrist yaw +/-
- `7` / `&` (Shift+7): Gripper open/close

Right Arm (q-y):
- `q` / `Q`: Base pitch +/-
- `w` / `W`: Shoulder yaw +/-
- `e` / `E`: Shoulder roll +/-
- `r` / `R`: Elbow pitch +/-
- `t` / `T`: Wrist pitch +/-
- `y` / `Y`: Wrist yaw +/-
- `u` / `U`: Gripper open/close

Neck (a,s):
- `a` / `A`: Yaw +/-
- `s` / `S`: Pitch +/-

### Mode 2: Cartesian Control

Control end-effector position and orientation in Cartesian space.

```bash
ros2 run robot_keyboard_control cartesian_control
```

**Key Mapping:**

Left Gripper (qweasdzxc):
- `q` / `a`: X +/-
- `w` / `s`: Y +/-
- `e` / `d`: Z +/-
- `z` / `x`: Roll +/-
- `c`: Yaw +

Right Gripper (uiojklm,.):
- `u` / `j`: X +/-
- `i` / `k`: Y +/-
- `o` / `l`: Z +/-
- `m` / `,`: Roll +/-
- `.`: Yaw +

Neck (g,h):
- `g` / `G`: Yaw +/-
- `h` / `H`: Pitch +/-

## Parameters

You can modify step sizes in the code:
- Joint mode: `joint_step = 0.1` (radians)
- Cartesian mode: `position_step = 0.02` (meters), `orientation_step = 0.1` (radians)

## Topics Published

- `/left_joint_command` (Float64MultiArray)
- `/right_joint_command` (Float64MultiArray)
- `/neck_joint_command` (Float64MultiArray)
- `/open_left_gripper` (Bool)
- `/open_right_gripper` (Bool)
- `/left_pose_command` (PoseCommand)
- `/right_pose_command` (PoseCommand)

## Notes

- Make sure `robot_commander` node is running to process these commands
- Press ESC or Ctrl+C to quit
- Terminal must support raw input mode

## Updates

### joint_keyboard_control.py

- Added functionality to read and initialize the robot's starting joint positions.

### cartesian_keyboard_control.py

- Added support for simulation time (`use_sim_time`).
- Fixed quaternion computation error.
```py
    def publish_pose(self, publisher, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    """Publish relative pose command"""
    msg = PoseCommand()
    msg.x = x
    msg.y = y
    msg.z = z
    msg.roll = roll
    msg.pitch = pitch
    msg.yaw = yaw
    msg.relative = True
    msg.cartesian_path = False #笛卡尔规划部分可用，需要测试。
    publisher.publish(msg)
```
