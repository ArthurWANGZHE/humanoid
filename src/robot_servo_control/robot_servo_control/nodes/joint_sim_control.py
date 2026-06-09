#!/usr/bin/env python3
from __future__ import annotations

import os
import select
import sys
import termios
import threading
import time
import tty
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.publisher import Publisher
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


@dataclass(frozen=True)
class JointCommand:
    target: str
    joint_name: str
    velocity: float


class JointSimControl(Node):
    HEAD_JOINTS = ["neck_pitch_joint", "neck_yaw_joint"]
    LEFT_ARM_JOINTS = [
        "left_base_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_pitch_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
    ]
    RIGHT_ARM_JOINTS = [
        "right_base_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_pitch_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint",
    ]

    JOINT_LIMITS: Dict[str, Tuple[float, float]] = {
        "neck_pitch_joint": (-1.5707963267949, 1.5707963267949),
        "neck_yaw_joint": (-1.5707963267949, 0.785398163397448),
        "left_base_pitch_joint": (-1.22173047639603, 1.83259571459405),
        "left_shoulder_roll_joint": (-0.0174532925199433, 1.5707963267949),
        "left_shoulder_yaw_joint": (-2.61799387799149, 2.61799387799149),
        "left_elbow_pitch_joint": (-0.0174532925199433, 1.5707963267949),
        "left_wrist_pitch_joint": (-2.96705972839036, 2.96705972839036),
        "left_wrist_yaw_joint": (-2.0943951023932, 2.0943951023932),
        "right_base_pitch_joint": (-1.83259571459405, 1.22173047639603),
        "right_shoulder_roll_joint": (-1.5707963267949, 0.0174532925199433),
        "right_shoulder_yaw_joint": (-2.61799387799149, 2.61799387799149),
        "right_elbow_pitch_joint": (-1.5707963267949, 0.0174532925199433),
        "right_wrist_pitch_joint": (-2.96705972839036, 2.96705972839036),
        "right_wrist_yaw_joint": (-2.0943951023932, 2.0943951023932),
    }

    def __init__(self) -> None:
        super().__init__("joint_sim_control")

        self.declare_parameter("head_trajectory_topic", "/neck_controller/joint_trajectory")
        self.declare_parameter("left_trajectory_topic", "/left_arm_controller/joint_trajectory")
        self.declare_parameter("right_trajectory_topic", "/right_arm_controller/joint_trajectory")
        self.declare_parameter("head_joint_speed", 1.0)
        self.declare_parameter("arm_joint_speed", 0.8)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("command_timeout", 0.35)
        self.declare_parameter("hold_last_command", True)
        self.declare_parameter("trajectory_time_sec", 0.04)

        self.head_joint_speed = self.get_double_param("head_joint_speed")
        self.arm_joint_speed = self.get_double_param("arm_joint_speed")
        self.publish_rate = self.get_double_param("publish_rate")
        self.command_timeout = self.get_double_param("command_timeout")
        self.hold_last_command = (
            self.get_parameter("hold_last_command").get_parameter_value().bool_value
        )
        self.trajectory_time_sec = self.get_double_param("trajectory_time_sec")

        self.joints_by_target: Dict[str, List[str]] = {
            "head": self.HEAD_JOINTS,
            "left": self.LEFT_ARM_JOINTS,
            "right": self.RIGHT_ARM_JOINTS,
        }
        self.publishers: Dict[str, Publisher] = {
            "head": self.create_publisher(
                JointTrajectory,
                self.get_string_param("head_trajectory_topic"),
                10,
            ),
            "left": self.create_publisher(
                JointTrajectory,
                self.get_string_param("left_trajectory_topic"),
                10,
            ),
            "right": self.create_publisher(
                JointTrajectory,
                self.get_string_param("right_trajectory_topic"),
                10,
            ),
        }

        self.active_arm = "left"
        self.direction = 1.0
        self.current_command: Optional[JointCommand] = None
        self.last_key_time = 0.0
        self.last_update_time = time.monotonic()
        self.running = True
        self.targets: Dict[str, float] = {}
        self.lock = threading.Lock()

        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 20)

        self.keyboard_thread = threading.Thread(
            target=self.keyboard_loop,
            daemon=True,
        )
        self.keyboard_thread.start()

        self.timer = self.create_timer(
            1.0 / max(self.publish_rate, 1.0),
            self.publish_current_command,
        )

        self.print_help()

    def get_string_param(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def get_double_param(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def on_joint_state(self, msg: JointState) -> None:
        with self.lock:
            for name, position in zip(msg.name, msg.position):
                if name in self.JOINT_LIMITS and name not in self.targets:
                    self.targets[name] = position

    def print_help(self) -> None:
        help_text = (
            "\n"
            "Joint sim control ready. This node bypasses MoveIt Servo and publishes "
            "JointTrajectory commands directly to simulation controllers.\n"
            "Head joint control:\n"
            "  w/s: neck_pitch_joint positive/negative\n"
            "  a/d: neck_yaw_joint positive/negative\n"
            "Arm joint control:\n"
            "  1-6: jog selected arm joint\n"
            "  Tab: switch selected arm between left/right\n"
            "  r: reverse arm jog direction\n"
            "Other:\n"
            "  space: stop all\n"
            "  q: quit node\n"
        )
        self.get_logger().info(help_text)

    def keyboard_loop(self) -> None:
        if os.name == "nt":
            raise RuntimeError("joint_sim_control supports Linux terminals only.")

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while rclpy.ok() and self.running:
                ready, _, _ = select.select([sys.stdin], [], [], 0.02)
                if not ready:
                    continue

                key = sys.stdin.read(1)
                self.handle_key(key)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def handle_key(self, key: str) -> None:
        normalized = key.lower()

        if normalized == "q":
            self.get_logger().info("Quit requested from keyboard.")
            self.running = False
            self.stop_all()
            return

        if normalized == "\t":
            self.switch_arm()
            return

        if normalized == "r":
            self.reverse_direction()
            return

        if normalized == " ":
            self.stop_all()
            return

        command = self.key_to_joint_command(normalized)
        if command is None:
            return

        with self.lock:
            self.current_command = command
            self.last_key_time = time.monotonic()

        self.get_logger().info(
            f"[{command.target}] {command.joint_name}: "
            f"velocity={command.velocity:.3f}",
            throttle_duration_sec=0.2,
        )

    def switch_arm(self) -> None:
        with self.lock:
            self.active_arm = "right" if self.active_arm == "left" else "left"
            self.current_command = None

        self.get_logger().info(f"Active arm switched to: {self.active_arm}")

    def reverse_direction(self) -> None:
        with self.lock:
            self.direction *= -1.0
            direction_text = "positive" if self.direction > 0.0 else "negative"

        self.get_logger().info(f"Arm jog direction switched to: {direction_text}")

    def key_to_joint_command(self, key: str) -> Optional[JointCommand]:
        if key == "w":
            return JointCommand("head", "neck_yaw_joint", -self.head_joint_speed)
        if key == "s":
            return JointCommand("head", "neck_yaw_joint", self.head_joint_speed)
        if key == "a":
            return JointCommand("head", "neck_pitch_joint", self.head_joint_speed)
        if key == "d":
            return JointCommand("head", "neck_pitch_joint", -self.head_joint_speed)

        if key not in {"1", "2", "3", "4", "5", "6"}:
            return None

        joint_index = int(key) - 1
        joint_name = self.joints_by_target[self.active_arm][joint_index]
        velocity = self.direction * self.arm_joint_speed
        return JointCommand(self.active_arm, joint_name, velocity)

    def publish_current_command(self) -> None:
        now = time.monotonic()
        dt = max(now - self.last_update_time, 0.0)
        self.last_update_time = now

        with self.lock:
            command = self.current_command
            elapsed = now - self.last_key_time if self.last_key_time else None

            if (
                not self.hold_last_command
                and elapsed is not None
                and elapsed > self.command_timeout
            ):
                self.current_command = None
                command = None

            if command is None:
                return

            if command.joint_name not in self.targets:
                self.get_logger().warn(
                    f"Waiting for /joint_states before commanding {command.joint_name}",
                    throttle_duration_sec=2.0,
                )
                return

            lower, upper = self.JOINT_LIMITS[command.joint_name]
            target_position = self.targets[command.joint_name] + command.velocity * dt
            self.targets[command.joint_name] = min(max(target_position, lower), upper)
            target = command.target
            joint_positions = {
                joint: self.targets.get(joint)
                for joint in self.joints_by_target[target]
            }

        if any(position is None for position in joint_positions.values()):
            self.get_logger().warn(
                f"Waiting for complete /joint_states for {target}",
                throttle_duration_sec=2.0,
            )
            return

        self.publish_trajectory(
            target=target,
            joint_names=list(joint_positions.keys()),
            positions=[float(position) for position in joint_positions.values()],
        )

    def stop_all(self) -> None:
        with self.lock:
            self.current_command = None
            snapshots = {
                target: [
                    self.targets[joint]
                    for joint in joints
                    if joint in self.targets
                ]
                for target, joints in self.joints_by_target.items()
            }

        for target, positions in snapshots.items():
            joint_names = self.joints_by_target[target]
            if len(positions) == len(joint_names):
                self.publish_trajectory(target, joint_names, positions)

    def publish_trajectory(
        self,
        target: str,
        joint_names: List[str],
        positions: List[float],
    ) -> None:
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(
            nanoseconds=int(max(self.trajectory_time_sec, 0.01) * 1_000_000_000)
        ).to_msg()
        msg.points = [point]

        self.publishers[target].publish(msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[JointSimControl] = None

    try:
        node = JointSimControl()
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop_all()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
