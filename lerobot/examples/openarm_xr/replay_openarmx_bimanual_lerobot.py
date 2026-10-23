#!/usr/bin/env python3
"""Replay a bimanual OpenArmX no-vision LeRobot episode over ROS 2."""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from lerobot.datasets import LeRobotDataset


SIDES = ("left", "right")
ARM_JOINT_NAMES = {side: [f"openarm_{side}_joint{i}" for i in range(1, 8)] for side in SIDES}
GRIPPER_JOINT_NAMES = {side: f"openarm_{side}_finger_joint1" for side in SIDES}
DEG_PER_RAD = 180.0 / math.pi


def gripper_degrees_to_position(degrees: float) -> float:
    ratio = max(0.0, min(1.0, degrees / -65.0))
    return ratio * 0.044


class BimanualReplay(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("openarmx_bimanual_lerobot_replay")
        self._arm_pubs = {
            side: self.create_publisher(Float64MultiArray, getattr(args, f"{side}_command_topic"), 10)
            for side in SIDES
        }
        self._gripper_pubs = {
            side: self.create_publisher(JointTrajectory, getattr(args, f"{side}_gripper_topic"), 10)
            for side in SIDES
        }
        self._positions: dict[str, float] = {}
        self.create_subscription(JointState, args.joint_state_topic, self._on_joint_state, 20)

    def _on_joint_state(self, msg: JointState) -> None:
        for index, name in enumerate(msg.name):
            if index < len(msg.position):
                self._positions[name] = float(msg.position[index])

    def wait_ready(self, timeout_s: float) -> None:
        deadline = time.perf_counter() + timeout_s
        required = [name for side in SIDES for name in ARM_JOINT_NAMES[side]]
        required += list(GRIPPER_JOINT_NAMES.values())
        while time.perf_counter() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(name in self._positions for name in required):
                if all(pub.get_subscription_count() > 0 for pub in self._arm_pubs.values()):
                    if all(pub.get_subscription_count() > 0 for pub in self._gripper_pubs.values()):
                        return
        missing = [name for name in required if name not in self._positions]
        raise TimeoutError(
            f"ROS2 topics/joint states not ready. Missing joints: {missing}; "
            "check both forward-position and gripper controllers."
        )

    def max_initial_error(self, values: list[float]) -> float:
        errors = []
        for side_index, side in enumerate(SIDES):
            offset = side_index * 8
            errors.extend(
                abs(values[offset + index] - self._positions[name] * DEG_PER_RAD)
                for index, name in enumerate(ARM_JOINT_NAMES[side])
            )
        return max(errors)

    def get_observation(self) -> dict[str, list[float]]:
        """Return current joint positions in the dataset's degree convention."""
        observation: list[float] = []
        for side in SIDES:
            observation.extend(
                self._positions.get(name, 0.0) * DEG_PER_RAD
                for name in ARM_JOINT_NAMES[side]
            )
            gripper_m = self._positions.get(GRIPPER_JOINT_NAMES[side], 0.0)
            observation.append(gripper_m / 0.044 * -65.0)
        return {"observation.state": np.asarray(observation, dtype=np.float32)}

    def publish_action(self, values: list[float]) -> None:
        if len(values) < 16:
            raise ValueError(f"Expected 16 action values, got {len(values)}")
        for side_index, side in enumerate(SIDES):
            offset = side_index * 8
            arm = Float64MultiArray()
            arm.data = [math.radians(value) for value in values[offset : offset + 7]]
            self._arm_pubs[side].publish(arm)

            trajectory = JointTrajectory()
            trajectory.joint_names = [GRIPPER_JOINT_NAMES[side]]
            point = JointTrajectoryPoint()
            point.positions = [gripper_degrees_to_position(values[offset + 7])]
            point.time_from_start.nanosec = 50_000_000
            trajectory.points = [point]
            self._gripper_pubs[side].publish(trajectory)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-id", default="local/openarmx_bimanual_novision_100hz")
    parser.add_argument(
        "--root",
        type=Path,
        default=Path("/home/w/OPENARM/lerobot/data/openarmx_bimanual_novision_100hz"),
    )
    parser.add_argument("--episode", type=int, default=0)
    parser.add_argument("--fps", type=float, default=100.0)
    parser.add_argument("--joint-state-topic", default="/joint_states")
    parser.add_argument("--left-command-topic", default="/left_forward_position_controller/commands")
    parser.add_argument("--right-command-topic", default="/right_forward_position_controller/commands")
    parser.add_argument("--left-gripper-topic", default="/left_gripper_controller/joint_trajectory")
    parser.add_argument("--right-gripper-topic", default="/right_gripper_controller/joint_trajectory")
    parser.add_argument("--max-initial-delta-deg", type=float, default=10.0)
    parser.add_argument("--connect-timeout-s", type=float, default=5.0)
    parser.add_argument("--confirm-real", action="store_true")
    parser.add_argument("--yes", action="store_true")
    parser.add_argument(
        "--display-data",
        action="store_true",
        help="Open a Rerun viewer and display observed/action joint values",
    )
    parser.add_argument("--display-ip", default=None)
    parser.add_argument("--display-port", type=int, default=None)
    parser.add_argument(
        "--display-every",
        type=int,
        default=5,
        help="Log every Nth replay command to Rerun/terminal (default: 5)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not args.confirm_real:
        raise SystemExit("Refusing to publish to a real robot; add --confirm-real after safety checks.")
    dataset = LeRobotDataset(args.repo_id, root=args.root, episodes=[args.episode])
    source_fps = float(dataset.fps)
    if args.fps <= 0:
        raise ValueError("--fps must be positive")
    print(f"Loaded episode {args.episode}: {dataset.num_frames} frames at {source_fps:g} Hz")
    if not args.yes:
        input("Clear the area and press Enter to start bimanual replay (Ctrl-C to abort): ")

    rclpy.init()
    node = BimanualReplay(args)
    rerun_enabled = False
    try:
        if args.display_data:
            from lerobot.utils.visualization_utils import init_rerun

            init_rerun(session_name="openarmx_bimanual_replay", ip=args.display_ip, port=args.display_port)
            rerun_enabled = True
            print("Rerun viewer started. Joint values are under observation.state and action.")
        node.wait_ready(args.connect_timeout_s)
        actions = [[float(value) for value in dataset[index]["action"]] for index in range(dataset.num_frames)]
        error = node.max_initial_error(actions[0])
        if error > args.max_initial_delta_deg:
            raise RuntimeError(f"Initial pose mismatch: {error:.2f} deg > {args.max_initial_delta_deg:.2f} deg")
        print(f"Initial pose check passed: max arm error {error:.2f} deg")
        steps = int(round((len(actions) - 1) * args.fps / source_fps)) + 1
        period = 1.0 / args.fps
        next_tick = time.perf_counter()
        for step in range(steps):
            source_position = step * source_fps / args.fps
            low = min(math.floor(source_position), len(actions) - 1)
            high = min(low + 1, len(actions) - 1)
            alpha = source_position - low
            action = [actions[low][axis] + (actions[high][axis] - actions[low][axis]) * alpha for axis in range(16)]
            node.publish_action(action)
            rclpy.spin_once(node, timeout_sec=0.0)
            if args.display_data and step % max(args.display_every, 1) == 0:
                action_data = {"action": np.asarray(action, dtype=np.float32)}
                observation = node.get_observation()
                if rerun_enabled:
                    from lerobot.utils.visualization_utils import log_rerun_data

                    log_rerun_data(observation=observation, action=action_data)
                if step % max(args.display_every * 20, 1) == 0:
                    left = ", ".join(f"{value:6.1f}" for value in observation["observation.state"][:8])
                    right = ", ".join(f"{value:6.1f}" for value in observation["observation.state"][8:])
                    print(f"Observed left [{left}] | right [{right}]")
            next_tick += period
            if (remaining := next_tick - time.perf_counter()) > 0:
                time.sleep(remaining)
        print(f"Replay finished: {steps} commands")
    except KeyboardInterrupt:
        print("\nReplay stopped by user.")
    finally:
        if rerun_enabled:
            from lerobot.utils.visualization_utils import shutdown_rerun

            shutdown_rerun()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
