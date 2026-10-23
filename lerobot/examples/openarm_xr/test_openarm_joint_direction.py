#!/usr/bin/env python3
"""Safely pulse one OpenArm joint and verify the joint-state sign."""

from __future__ import annotations

import argparse
import math
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class JointDirectionTest(Node):
    def __init__(self, side: str) -> None:
        super().__init__("openarm_joint_direction_test")
        self.side = side
        self.joint_names = [f"openarm_{side}_joint{i}" for i in range(1, 8)]
        self.positions: dict[str, float] = {}
        self.state_event = threading.Event()
        self.publisher = self.create_publisher(
            Float64MultiArray,
            f"/{side}_forward_position_controller/commands",
            10,
        )
        self.subscription = self.create_subscription(
            JointState, "/joint_states", self._state_callback, 10
        )

    def _state_callback(self, message: JointState) -> None:
        for index, name in enumerate(message.name):
            if index < len(message.position):
                self.positions[name] = float(message.position[index])
        if all(name in self.positions for name in self.joint_names):
            self.state_event.set()

    def arm_positions(self) -> list[float]:
        return [self.positions[name] for name in self.joint_names]

    def publish(self, positions: list[float]) -> None:
        message = Float64MultiArray()
        message.data = positions
        self.publisher.publish(message)


def interpolate(
    node: JointDirectionTest,
    start: list[float],
    goal: list[float],
    duration_s: float,
    rate_hz: float,
    measured_index: int | None = None,
) -> list[float]:
    steps = max(1, round(duration_s * rate_hz))
    period = 1.0 / rate_hz
    measured_samples: list[float] = []
    for step in range(1, steps + 1):
        alpha = step / steps
        node.publish([a + alpha * (b - a) for a, b in zip(start, goal)])
        time.sleep(period)
        if measured_index is not None:
            measured_samples.append(node.arm_positions()[measured_index])
    return measured_samples


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--joint", type=int, choices=range(1, 8), required=True)
    parser.add_argument("--delta-deg", type=float, default=1.0)
    parser.add_argument("--move-time-s", type=float, default=2.0)
    parser.add_argument("--hold-time-s", type=float, default=1.0)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    args = parser.parse_args()

    if abs(args.delta_deg) > 5.0:
        parser.error("abs(delta-deg) must not exceed 5 degrees")
    if args.move_time_s < 1.0:
        parser.error("move-time-s must be at least 1 second")

    rclpy.init()
    node = JointDirectionTest(args.side)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    initial: list[float] | None = None
    try:
        if not node.state_event.wait(timeout=3.0):
            raise RuntimeError("no complete arm state received from /joint_states")

        initial = node.arm_positions()
        index = args.joint - 1
        delta = math.radians(args.delta_deg)
        target = initial.copy()
        target[index] += delta

        if node.count_subscribers(
            f"/{args.side}_forward_position_controller/commands"
        ) == 0:
            raise RuntimeError("position controller is not subscribed to its command topic")

        print(f"Initial {args.side} joint positions: {initial}")
        print(
            f"Moving joint {args.joint} by {args.delta_deg:+.3f} deg "
            f"over {args.move_time_s:.1f}s"
        )
        measured_samples = interpolate(
            node,
            initial,
            target,
            args.move_time_s,
            args.rate_hz,
            measured_index=index,
        )
        hold_end = time.monotonic() + args.hold_time_s
        while time.monotonic() < hold_end:
            node.publish(target)
            time.sleep(1.0 / args.rate_hz)
            measured_samples.append(node.arm_positions()[index])

        measured = node.arm_positions()
        measured_delta = measured[index] - initial[index]
        print(
            f"Measured joint {args.joint} delta: "
            f"{measured_delta:+.6f} rad ({math.degrees(measured_delta):+.3f} deg)"
        )
        peak_delta = max(
            measured_samples,
            key=lambda position: abs(position - initial[index]),
        ) - initial[index]
        print(
            f"Peak measured delta: {peak_delta:+.6f} rad "
            f"({math.degrees(peak_delta):+.3f} deg)"
        )
        sign_ok = measured_delta * delta > 0.0
        magnitude_ok = abs(measured_delta) >= 0.25 * abs(delta)
        print(f"Numeric direction result: {'PASS' if sign_ok and magnitude_ok else 'FAIL'}")

        print("Returning smoothly to the captured initial position")
        interpolate(node, node.arm_positions(), initial, args.move_time_s, args.rate_hz)
        return 0 if sign_ok and magnitude_ok else 1
    except KeyboardInterrupt:
        print("Interrupted; returning to the captured initial position")
        return 130
    except Exception as error:
        print(f"ERROR: {error}")
        return 2
    finally:
        if initial is not None:
            for _ in range(50):
                node.publish(initial)
                time.sleep(0.02)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    raise SystemExit(main())
