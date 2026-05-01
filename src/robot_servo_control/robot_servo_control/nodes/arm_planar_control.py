#!/usr/bin/env python3
from __future__ import annotations

import os
import select
import sys
import termios
import threading
import time
import tty
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_srvs.srv import Trigger


class ArmPlanarControl(Node):
    def __init__(self) -> None:
        super().__init__("arm_planar_control")

        self.declare_parameter(
            "twist_topic",
            "/right_servo/moveit_servo/delta_twist_cmds",
        )
        self.declare_parameter(
            "start_service",
            "/right_servo/moveit_servo/start_servo",
        )
        self.declare_parameter("command_frame", "base_link")
        self.declare_parameter("linear_speed", 0.08)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("command_timeout", 0.35)
        self.declare_parameter("hold_last_command", True)
        self.declare_parameter("wait_service_timeout_sec", 30.0)

        self.command_frame = self.get_parameter("command_frame").value
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.hold_last_command = bool(self.get_parameter("hold_last_command").value)
        self.wait_service_timeout_sec = float(
            self.get_parameter("wait_service_timeout_sec").value
        )

        self.publisher = self.create_publisher(
            TwistStamped,
            self.get_parameter("twist_topic").value,
            10,
        )

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.last_key_time = 0.0
        self.running = True
        self.lock = threading.Lock()

        self.start_servo()

        self.keyboard_thread = threading.Thread(
            target=self.keyboard_loop,
            daemon=True,
        )
        self.keyboard_thread.start()

        self.timer = self.create_timer(
            1.0 / max(self.publish_rate, 1.0),
            self.publish_velocity,
        )

        self.print_help()

    def start_servo(self) -> None:
        service_name = self.get_parameter("start_service").value
        client = self.create_client(Trigger, service_name)

        self.get_logger().info(f"waiting for servo service: {service_name}")
        if not client.wait_for_service(timeout_sec=self.wait_service_timeout_sec):
            raise RuntimeError(f"servo start service unavailable: {service_name}")

        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=self.wait_service_timeout_sec,
        )
        if not future.done() or future.result() is None:
            raise RuntimeError("failed to call servo start service")
        if not future.result().success:
            raise RuntimeError(f"servo failed to start: {future.result().message}")

        self.get_logger().info(f"servo started: {future.result().message}")

    def print_help(self) -> None:
        self.get_logger().info(
            "\n"
            "Arm planar control ready. WASD moves the right TCP on the table plane.\n"
            "  w/s: world +X / -X\n"
            "  a/d: world +Y / -Y\n"
            "  e/c: up / down for setup\n"
            "  x or space: stop\n"
            "  q: quit\n"
            "Nominal push plane: world z ~= 0.47, table top z ~= 0.41.\n"
        )

    def keyboard_loop(self) -> None:
        if os.name == "nt":
            raise RuntimeError("arm_planar_control supports Linux terminals only.")

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while rclpy.ok() and self.running:
                ready, _, _ = select.select([sys.stdin], [], [], 0.02)
                if not ready:
                    continue

                self.handle_key(sys.stdin.read(1))
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def handle_key(self, key: str) -> None:
        normalized = key.lower()

        if normalized == "q":
            self.get_logger().info("Quit requested from keyboard.")
            self.running = False
            self.stop()
            return

        if normalized in {" ", "x"}:
            self.stop()
            return

        velocity = self.key_to_base_velocity(normalized)
        if velocity is None:
            return

        with self.lock:
            self.vx, self.vy, self.vz = velocity
            self.last_key_time = time.monotonic()

        self.get_logger().info(
            f"right TCP velocity in base_link: "
            f"x={self.vx:.3f}, y={self.vy:.3f}, z={self.vz:.3f}",
            throttle_duration_sec=0.2,
        )

    def key_to_base_velocity(self, key: str) -> Optional[Tuple[float, float, float]]:
        speed = self.linear_speed
        # simulation.launch.py spawns the robot with yaw=+90deg:
        # world +X == base -Y, world +Y == base +X.
        return {
            "w": (0.0, -speed, 0.0),
            "s": (0.0, speed, 0.0),
            "a": (speed, 0.0, 0.0),
            "d": (-speed, 0.0, 0.0),
            "e": (0.0, 0.0, speed),
            "c": (0.0, 0.0, -speed),
        }.get(key)

    def stop(self) -> None:
        with self.lock:
            self.vx = 0.0
            self.vy = 0.0
            self.vz = 0.0
            self.last_key_time = time.monotonic()
        self.publish_velocity()

    def publish_velocity(self) -> None:
        with self.lock:
            vx = self.vx
            vy = self.vy
            vz = self.vz
            elapsed = (
                time.monotonic() - self.last_key_time
                if self.last_key_time
                else None
            )

            if (
                not self.hold_last_command
                and elapsed is not None
                and elapsed > self.command_timeout
            ):
                vx = 0.0
                vy = 0.0
                vz = 0.0
                self.vx = 0.0
                self.vy = 0.0
                self.vz = 0.0

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.command_frame
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        self.publisher.publish(msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[ArmPlanarControl] = None

    try:
        node = ArmPlanarControl()
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
