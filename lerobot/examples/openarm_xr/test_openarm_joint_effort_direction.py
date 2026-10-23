#!/usr/bin/env python3
"""Apply a bounded effort pulse while holding the current OpenArm pose."""

from __future__ import annotations

import argparse
import math
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class JointEffortTest(Node):
    def __init__(self, side: str) -> None:
        super().__init__("openarm_joint_effort_direction_test")
        self.side = side
        self.joint_names = [f"openarm_{side}_joint{i}" for i in range(1, 8)]
        self.positions: dict[str, float] = {}
        self.state_event = threading.Event()
        self.position_topic = f"/{side}_forward_position_controller/commands"
        self.effort_topic = f"/{side}_forward_effort_controller/commands"
        self.position_publisher = self.create_publisher(
            Float64MultiArray, self.position_topic, 10
        )
        self.effort_publisher = self.create_publisher(
            Float64MultiArray, self.effort_topic, 10
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

    def publish(self, positions: list[float], efforts: list[float]) -> None:
        position_message = Float64MultiArray()
        position_message.data = positions
        self.position_publisher.publish(position_message)

        effort_message = Float64MultiArray()
        effort_message.data = efforts
        self.effort_publisher.publish(effort_message)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--joint", type=int, choices=range(1, 8), required=True)
    parser.add_argument("--effort-nm", type=float, default=0.2)
    parser.add_argument("--ramp-time-s", type=float, default=2.0)
    parser.add_argument("--hold-time-s", type=float, default=2.0)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    args = parser.parse_args()

    effort_limits = (0.5, 0.5, 0.3, 0.3, 0.2, 0.2, 0.2)
    effort_limit = effort_limits[args.joint - 1]
    if not math.isfinite(args.effort_nm) or abs(args.effort_nm) > effort_limit:
        parser.error(
            f"abs(effort-nm) for joint {args.joint} must not exceed "
            f"{effort_limit:.1f} Nm"
        )
    if args.ramp_time_s < 1.0:
        parser.error("ramp-time-s must be at least 1 second")

    rclpy.init()
    node = JointEffortTest(args.side)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    zero_efforts = [0.0] * 7
    initial: list[float] | None = None
    try:
        if not node.state_event.wait(timeout=3.0):
            raise RuntimeError("no complete arm state received from /joint_states")
        if node.count_subscribers(node.position_topic) == 0:
            raise RuntimeError("position controller is not active")
        if node.count_subscribers(node.effort_topic) == 0:
            raise RuntimeError(
                "effort controller is not active; launch with "
                "enable_effort_controllers:=true"
            )
        if node.count_publishers(node.effort_topic) > 1:
            raise RuntimeError(
                "another effort publisher is active; stop gravity compensation first"
            )

        initial = node.arm_positions()
        index = args.joint - 1
        steps = max(1, round(args.ramp_time_s * args.rate_hz))
        period = 1.0 / args.rate_hz
        measured_samples: list[float] = []

        print(f"Initial {args.side} joint positions: {initial}")
        print(
            f"Ramping joint {args.joint} effort to {args.effort_nm:+.3f} Nm "
            f"over {args.ramp_time_s:.1f}s"
        )
        for step in range(1, steps + 1):
            efforts = zero_efforts.copy()
            efforts[index] = args.effort_nm * step / steps
            node.publish(initial, efforts)
            time.sleep(period)
            measured_samples.append(node.arm_positions()[index])

        hold_end = time.monotonic() + args.hold_time_s
        while time.monotonic() < hold_end:
            efforts = zero_efforts.copy()
            efforts[index] = args.effort_nm
            node.publish(initial, efforts)
            time.sleep(period)
            measured_samples.append(node.arm_positions()[index])

        measured_delta = node.arm_positions()[index] - initial[index]
        peak_delta = max(
            measured_samples,
            key=lambda position: abs(position - initial[index]),
        ) - initial[index]
        print(
            f"Measured delta at full effort: {measured_delta:+.6f} rad "
            f"({math.degrees(measured_delta):+.3f} deg)"
        )
        print(
            f"Peak measured delta: {peak_delta:+.6f} rad "
            f"({math.degrees(peak_delta):+.3f} deg)"
        )
        sign_ok = peak_delta * args.effort_nm > 0.0
        print(f"Effort direction result: {'PASS' if sign_ok else 'FAIL'}")

        print("Ramping effort back to zero")
        for step in range(steps - 1, -1, -1):
            efforts = zero_efforts.copy()
            efforts[index] = args.effort_nm * step / steps
            node.publish(initial, efforts)
            time.sleep(period)
        return 0 if sign_ok else 1
    except KeyboardInterrupt:
        print("Interrupted; removing effort command")
        return 130
    except Exception as error:
        print(f"ERROR: {error}")
        return 2
    finally:
        if initial is not None:
            for _ in range(100):
                node.publish(initial, zero_efforts)
                time.sleep(0.02)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    raise SystemExit(main())
