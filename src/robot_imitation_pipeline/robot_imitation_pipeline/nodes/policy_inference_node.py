#!/usr/bin/env python3
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import rclpy
from example_interfaces.msg import Bool, Float64MultiArray
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from sensor_msgs.msg import JointState


def _make_mlp(input_dim: int, action_dim: int, hidden_dim: int, num_layers: int):
    import torch.nn as nn

    layers = []
    last = input_dim
    for _ in range(num_layers):
        layers += [nn.Linear(last, hidden_dim), nn.ReLU()]
        last = hidden_dim
    layers.append(nn.Linear(last, action_dim))
    return nn.Sequential(*layers)


class PolicyInferenceNode(Node):
    def __init__(self) -> None:
        super().__init__("policy_inference")
        self._declare_parameters()
        self._load_parameters()

        self.latest_joint_state: Optional[JointState] = None
        self.latest_joint_state_wall = 0.0

        self.model = self._load_policy(self.checkpoint_path)
        self.model.eval()

        self.joint_sub = self.create_subscription(
            JointState, self.joint_states_topic, self._joint_state_cb, 10
        )
        self.right_joint_pub = self.create_publisher(
            Float64MultiArray, self.right_joint_command_topic, 10
        )
        self.right_gripper_pub = self.create_publisher(Bool, self.right_gripper_topic, 10)
        self.timer = self.create_timer(1.0 / self.inference_rate_hz, self._infer_once)

        mode = "publishing to robot" if self.publish_commands else "dry-run logging only"
        self.get_logger().info(f"Policy loaded from {self.checkpoint_path} ({mode}).")

    def _declare_parameters(self) -> None:
        string_array_desc = ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        self.declare_parameter("checkpoint_path", "data/imitation_runs/bc_state/best.pt")
        self.declare_parameter("publish_commands", False)
        self.declare_parameter("inference_rate_hz", 10.0)
        self.declare_parameter("stale_joint_state_sec", 0.5)
        self.declare_parameter("max_joint_step_rad", 0.08)
        self.declare_parameter("gripper_open_threshold", 0.5)
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("right_joint_command_topic", "/right_joint_command")
        self.declare_parameter("right_gripper_topic", "/open_right_gripper")
        self.declare_parameter(
            "right_arm_joint_names",
            [
                "right_base_pitch_joint",
                "right_shoulder_roll_joint",
                "right_shoulder_yaw_joint",
                "right_elbow_pitch_joint",
                "right_wrist_pitch_joint",
                "right_wrist_yaw_joint",
            ],
            string_array_desc,
        )
        self.declare_parameter("right_gripper_name", "right_gripper1_joint")

    def _load_parameters(self) -> None:
        self.checkpoint_path = Path(str(self.get_parameter("checkpoint_path").value))
        if not self.checkpoint_path.is_absolute():
            self.checkpoint_path = Path.cwd() / self.checkpoint_path
        self.publish_commands = bool(self.get_parameter("publish_commands").value)
        self.inference_rate_hz = max(float(self.get_parameter("inference_rate_hz").value), 1.0)
        self.stale_joint_state_sec = float(self.get_parameter("stale_joint_state_sec").value)
        self.max_joint_step_rad = float(self.get_parameter("max_joint_step_rad").value)
        self.gripper_open_threshold = float(self.get_parameter("gripper_open_threshold").value)
        self.joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self.right_joint_command_topic = str(self.get_parameter("right_joint_command_topic").value)
        self.right_gripper_topic = str(self.get_parameter("right_gripper_topic").value)
        self.right_arm_joint_names = list(self.get_parameter("right_arm_joint_names").value)
        self.right_gripper_name = str(self.get_parameter("right_gripper_name").value)

    def _load_policy(self, checkpoint_path: Path):
        try:
            import torch
        except ImportError as exc:
            raise RuntimeError("policy_inference_node requires PyTorch.") from exc

        if not checkpoint_path.exists():
            raise RuntimeError(f"Policy checkpoint not found: {checkpoint_path}")
        try:
            checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
        except TypeError:
            checkpoint = torch.load(checkpoint_path, map_location="cpu")
        config = checkpoint.get("config", {})
        model_cfg: Dict = config.get("model", {}) if isinstance(config, dict) else {}
        input_dim = int(checkpoint.get("input_dim", model_cfg.get("input_dim", 13)))
        action_dim = int(checkpoint.get("action_dim", model_cfg.get("action_dim", 7)))
        if input_dim != 13:
            raise RuntimeError(f"Expected policy input_dim=13 for robot_state, got {input_dim}")
        if action_dim != 7:
            raise RuntimeError(f"Expected policy action_dim=7 for right-arm action, got {action_dim}")
        model = _make_mlp(
            input_dim,
            action_dim,
            int(model_cfg.get("hidden_dim", 256)),
            int(model_cfg.get("num_layers", 3)),
        )
        model.load_state_dict(checkpoint["model"])
        return model

    def _joint_state_cb(self, msg: JointState) -> None:
        self.latest_joint_state = msg
        self.latest_joint_state_wall = self.get_clock().now().nanoseconds * 1e-9

    def _infer_once(self) -> None:
        if self.latest_joint_state is None:
            self.get_logger().warn("Waiting for joint state before policy inference.")
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        age = now - self.latest_joint_state_wall
        if age > self.stale_joint_state_sec:
            self.get_logger().warn(f"Joint state is stale: {age:.3f}s.")
            return

        state = self._make_state(self.latest_joint_state)
        if state is None:
            return

        try:
            import torch

            with torch.no_grad():
                action = self.model(torch.from_numpy(state.astype(np.float32)).unsqueeze(0))
                action_np = action.squeeze(0).cpu().numpy().astype(np.float64)
        except Exception as exc:
            self.get_logger().error(f"Policy inference failed: {exc}")
            return

        current_right = state[:6]
        target_right = self._limit_joint_step(action_np[:6], current_right)
        gripper_open = bool(action_np[6] >= self.gripper_open_threshold)

        if not self.publish_commands:
            self.get_logger().info(
                "Policy dry-run: "
                f"right={np.array2string(target_right, precision=3)} "
                f"gripper_open={gripper_open}"
            )
            return

        right_msg = Float64MultiArray()
        right_msg.data = target_right.astype(float).tolist()
        gripper_msg = Bool()
        gripper_msg.data = gripper_open
        self.right_joint_pub.publish(right_msg)
        self.right_gripper_pub.publish(gripper_msg)

    def _make_state(self, msg: JointState) -> Optional[np.ndarray]:
        name_to_pos = dict(zip(msg.name, msg.position))
        name_to_vel = dict(zip(msg.name, msg.velocity))
        missing = [
            name
            for name in self.right_arm_joint_names + [self.right_gripper_name]
            if name not in name_to_pos
        ]
        if missing:
            self.get_logger().warn(f"Joint state missing required names: {missing}")
            return None
        pos = np.asarray([name_to_pos[name] for name in self.right_arm_joint_names], dtype=np.float64)
        vel = np.asarray([name_to_vel.get(name, 0.0) for name in self.right_arm_joint_names], dtype=np.float64)
        gripper = np.asarray([name_to_pos[self.right_gripper_name]], dtype=np.float64)
        return np.concatenate([pos, vel, gripper], axis=0)

    def _limit_joint_step(self, target: np.ndarray, current: np.ndarray) -> np.ndarray:
        if self.max_joint_step_rad <= 0.0:
            return target
        delta = np.clip(target - current, -self.max_joint_step_rad, self.max_joint_step_rad)
        return current + delta


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[PolicyInferenceNode] = None
    try:
        node = PolicyInferenceNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        logger = node.get_logger() if node is not None else rclpy.logging.get_logger("policy_inference")
        logger.error(str(exc))
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
