#!/usr/bin/env python3
from __future__ import annotations

import os
import select
import sys
import termios
import threading
import time
import tty
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class PlanarPusherControl(Node):
    def __init__(self) -> None:
        super().__init__("planar_pusher_control")

        self.declare_parameter("cmd_vel_topic", "/pusher/cmd_vel")
        self.declare_parameter("linear_speed", 0.22)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("command_timeout", 0.35)
        self.declare_parameter("hold_last_command", True)

        self.linear_speed = self.get_parameter("linear_speed").value
        self.publish_rate = self.get_parameter("publish_rate").value
        self.command_timeout = self.get_parameter("command_timeout").value
        self.hold_last_command = self.get_parameter("hold_last_command").value

        self.publisher = self.create_publisher(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            10,
        )

        self.vx = 0.0
        self.vy = 0.0
        self.last_key_time = 0.0
        self.running = True
        self.lock = threading.Lock()

        self.keyboard_thread = threading.Thread(
            target=self.keyboard_loop,
            daemon=True,
        )
        self.keyboard_thread.start()

        self.timer = self.create_timer(
            1.0 / max(float(self.publish_rate), 1.0),
            self.publish_velocity,
        )

        self.print_help()

    def print_help(self) -> None:
        self.get_logger().info(
            "\n"
            "Planar pusher control ready.\n"
            "  w/s: move +X / -X on the table\n"
            "  a/d: move +Y / -Y on the table\n"
            "  x or space: stop\n"
            "  q: quit\n"
        )

    def keyboard_loop(self) -> None:
        if os.name == "nt":
            raise RuntimeError("planar_pusher_control supports Linux terminals only.")

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
        speed = float(self.linear_speed)

        if normalized == "q":
            self.get_logger().info("Quit requested from keyboard.")
            self.running = False
            self.stop()
            return

        if normalized in {" ", "x"}:
            self.stop()
            return

        velocity = {
            "w": (speed, 0.0),
            "s": (-speed, 0.0),
            "a": (0.0, speed),
            "d": (0.0, -speed),
        }.get(normalized)

        if velocity is None:
            return

        with self.lock:
            self.vx, self.vy = velocity
            self.last_key_time = time.monotonic()

        self.get_logger().info(
            f"pusher velocity: x={self.vx:.3f}, y={self.vy:.3f}",
            throttle_duration_sec=0.2,
        )

    def stop(self) -> None:
        with self.lock:
            self.vx = 0.0
            self.vy = 0.0
            self.last_key_time = time.monotonic()
        self.publish_velocity()

    def publish_velocity(self) -> None:
        with self.lock:
            vx = self.vx
            vy = self.vy
            elapsed = (
                time.monotonic() - self.last_key_time
                if self.last_key_time
                else None
            )

            if (
                not bool(self.hold_last_command)
                and elapsed is not None
                and elapsed > float(self.command_timeout)
            ):
                vx = 0.0
                vy = 0.0
                self.vx = 0.0
                self.vy = 0.0

        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[PlanarPusherControl] = None

    try:
        node = PlanarPusherControl()
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
