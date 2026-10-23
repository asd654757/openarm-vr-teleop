#!/usr/bin/env python3
"""Replay a recorded OpenArmX left-arm LeRobot episode over ROS 2.

This matches ``record_openarmx_left_lerobot.py``: arm commands are published
as radians on the forward-position-controller topic and the gripper command is
published as a one-point ``JointTrajectory`` in meters.
"""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from lerobot.datasets import LeRobotDataset


ARM_JOINT_NAMES = [f"openarm_left_joint{idx}" for idx in range(1, 8)]
GRIPPER_JOINT_NAME = "openarm_left_finger_joint1"
GRIPPER_CLOSED_DEGREES = 0.0
GRIPPER_OPEN_DEGREES = -65.0
GRIPPER_CLOSED_POSITION_M = 0.0
GRIPPER_OPEN_POSITION_M = 0.044


def gripper_degrees_to_position(degrees: float) -> float:
    span_degrees = GRIPPER_OPEN_DEGREES - GRIPPER_CLOSED_DEGREES
    if span_degrees == 0.0:
        return GRIPPER_CLOSED_POSITION_M
    ratio = (degrees - GRIPPER_CLOSED_DEGREES) / span_degrees
    ratio = max(0.0, min(1.0, ratio))
    return GRIPPER_CLOSED_POSITION_M + ratio * (
        GRIPPER_OPEN_POSITION_M - GRIPPER_CLOSED_POSITION_M
    )


class OpenArmXLeftReplay(Node):
    def __init__(self, command_topic: str, gripper_topic: str):
        super().__init__("openarmx_left_lerobot_replay")
        self._arm_pub = self.create_publisher(Float64MultiArray, command_topic, 10)
        self._gripper_pub = self.create_publisher(JointTrajectory, gripper_topic, 10)
        self._positions: dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)

    def _on_joint_state(self, msg: JointState) -> None:
        for idx, name in enumerate(msg.name):
            if idx < len(msg.position):
                self._positions[name] = float(msg.position[idx])

    def wait_for_joint_states(self, timeout_s: float = 5.0) -> None:
        deadline = time.perf_counter() + timeout_s
        while time.perf_counter() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(name in self._positions for name in ARM_JOINT_NAMES):
                return
        missing = [name for name in ARM_JOINT_NAMES if name not in self._positions]
        raise TimeoutError(f"Did not receive required joint states; missing: {missing}")

    def wait_for_controller(self, timeout_s: float = 5.0) -> None:
        deadline = time.perf_counter() + timeout_s
        while time.perf_counter() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._arm_pub.get_subscription_count() > 0 and self._gripper_pub.get_subscription_count() > 0:
                return
        raise TimeoutError(
            "ROS2 controller topics have no subscribers. Check that "
            "left_forward_position_controller and left_gripper_controller are active."
        )

    def initial_arm_error_degrees(self, values: list[float]) -> float:
        errors = [
            abs(values[idx] - self._positions[name] * 180.0 / 3.141592653589793)
            for idx, name in enumerate(ARM_JOINT_NAMES)
        ]
        return max(errors)

    def publish_action(self, values: list[float]) -> None:
        if len(values) < 8:
            raise ValueError(f"Expected 8 action values, got {len(values)}")

        arm = Float64MultiArray()
        # Dataset arm values are degrees; ROS2 controller values are radians.
        arm.data = [value * 3.141592653589793 / 180.0 for value in values[:7]]
        self._arm_pub.publish(arm)

        trajectory = JointTrajectory()
        trajectory.joint_names = [GRIPPER_JOINT_NAME]
        point = JointTrajectoryPoint()
        point.positions = [gripper_degrees_to_position(values[7])]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 50_000_000
        trajectory.points = [point]
        self._gripper_pub.publish(trajectory)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-id", default="local/openarmx_left_novision")
    parser.add_argument(
        "--root", type=Path, default=Path("/home/w/OPENARM/lerobot/data/openarmx_left_novision")
    )
    parser.add_argument("--episode", type=int, default=0)
    parser.add_argument("--command-topic", default="/left_forward_position_controller/commands")
    parser.add_argument("--gripper-topic", default="/left_gripper_controller/joint_trajectory")
    parser.add_argument(
        "--fps",
        type=float,
        default=100.0,
        help="Output command rate; actions are linearly interpolated (default: 100 Hz)",
    )
    parser.add_argument(
        "--max-initial-delta-deg",
        type=float,
        default=10.0,
        help="Abort if any arm joint differs from the first action by more than this",
    )
    parser.add_argument(
        "--confirm-real",
        action="store_true",
        help="Required before publishing commands to a real robot",
    )
    parser.add_argument("--yes", action="store_true", help="Skip the safety confirmation prompt")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    dataset = LeRobotDataset(args.repo_id, root=args.root, episodes=[args.episode])
    source_fps = float(dataset.fps)
    replay_fps = float(args.fps)
    if replay_fps <= 0:
        raise ValueError("FPS must be positive")

    print(
        f"Loaded episode {args.episode}: {dataset.num_frames} frames recorded at "
        f"{source_fps:g} Hz; replaying at {replay_fps:g} Hz with interpolation.\n"
        f"Arm topic: {args.command_topic}\nGripper topic: {args.gripper_topic}"
    )
    if not args.yes:
        input("Ensure the area is clear, then press Enter to start replay (Ctrl-C to abort): ")

    if not args.confirm_real:
        raise SystemExit(
            "Refusing to publish: this is a real-robot replay. "
            "Re-run with --confirm-real after checking the controller and safety setup."
        )

    rclpy.init()
    node = OpenArmXLeftReplay(args.command_topic, args.gripper_topic)
    period = 1.0 / replay_fps
    try:
        # Give ROS2 discovery/publishers a moment to match the controller.
        time.sleep(0.5)
        node.wait_for_controller()
        node.wait_for_joint_states()
        first_action = [float(value) for value in dataset[0]["action"]]
        initial_error = node.initial_arm_error_degrees(first_action)
        if initial_error > args.max_initial_delta_deg:
            raise RuntimeError(
                f"Initial pose mismatch: max arm error is {initial_error:.2f} deg, "
                f"limit is {args.max_initial_delta_deg:.2f} deg. "
                "Move the robot to the recorded starting pose or adjust the limit deliberately."
            )
        print(f"Initial pose check passed: max arm error {initial_error:.2f} deg")
        next_tick = time.perf_counter()
        actions = [[float(value) for value in dataset[idx]["action"]] for idx in range(dataset.num_frames)]
        replay_steps = int(round((len(actions) - 1) * replay_fps / source_fps)) + 1
        for step in range(replay_steps):
            if not rclpy.ok():
                break
            source_position = step * source_fps / replay_fps
            low_index = min(math.floor(source_position), len(actions) - 1)
            high_index = min(low_index + 1, len(actions) - 1)
            alpha = source_position - low_index
            action = [
                actions[low_index][axis]
                + (actions[high_index][axis] - actions[low_index][axis]) * alpha
                for axis in range(8)
            ]
            node.publish_action(action)
            rclpy.spin_once(node, timeout_sec=0.0)
            next_tick += period
            remaining = next_tick - time.perf_counter()
            if remaining > 0:
                time.sleep(remaining)
        print(f"Replay finished: {replay_steps} commands")
    except KeyboardInterrupt:
        print("\nReplay stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
