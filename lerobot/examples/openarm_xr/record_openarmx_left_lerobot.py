#!/usr/bin/env python3
"""Record no-vision LeRobot episodes from the OpenArmX left-arm teleop.

The script only subscribes to ROS 2 topics. It does not send commands, so it
can run alongside the existing OpenArmX VR teleop without fighting it.
"""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path
from threading import Lock

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory

from lerobot.datasets import LeRobotDataset


LEFT_ROS_JOINT_NAMES = [
    "openarm_left_joint1",
    "openarm_left_joint2",
    "openarm_left_joint3",
    "openarm_left_joint4",
    "openarm_left_joint5",
    "openarm_left_joint6",
    "openarm_left_joint7",
]
LEFT_GRIPPER_ROS_JOINT_NAME = "openarm_left_finger_joint1"

FEATURE_NAMES = [f"left_joint_{idx}.pos" for idx in range(1, 8)] + ["left_gripper.pos"]
FEATURE_SIZE = len(FEATURE_NAMES)

GRIPPER_CLOSED_DEGREES = 0.0
GRIPPER_OPEN_DEGREES = -65.0
GRIPPER_CLOSED_POSITION_M = 0.0
GRIPPER_OPEN_POSITION_M = 0.044


def make_features() -> dict:
    return {
        "observation.state": {
            "dtype": "float32",
            "shape": (FEATURE_SIZE,),
            "names": FEATURE_NAMES,
        },
        "action": {
            "dtype": "float32",
            "shape": (FEATURE_SIZE,),
            "names": FEATURE_NAMES,
        },
    }


def gripper_position_to_degrees(position_m: float) -> float:
    span_m = GRIPPER_OPEN_POSITION_M - GRIPPER_CLOSED_POSITION_M
    if span_m == 0.0:
        return GRIPPER_CLOSED_DEGREES
    ratio = (position_m - GRIPPER_CLOSED_POSITION_M) / span_m
    ratio = max(0.0, min(1.0, ratio))
    return GRIPPER_CLOSED_DEGREES + ratio * (
        GRIPPER_OPEN_DEGREES - GRIPPER_CLOSED_DEGREES
    )


class OpenArmXLeftRecorder(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("openarmx_left_lerobot_recorder")
        self._lock = Lock()
        self._positions: dict[str, float] = {}
        self._arm_command: list[float] | None = None
        self._gripper_action_m: float = GRIPPER_CLOSED_POSITION_M
        self._gripper_action_received = False

        self.create_subscription(
            JointState, args.joint_state_topic, self._on_joint_state, 10
        )
        self.create_subscription(
            Float64MultiArray, args.command_topic, self._on_arm_command, 10
        )
        self.create_subscription(
            JointTrajectory, args.gripper_topic, self._on_gripper_command, 10
        )
        # ``RcutilsLogger`` follows the ROS 2 logging API and accepts one
        # already-formatted message (unlike the stdlib ``logging`` module).
        self.get_logger().info(
            "Listening for left OpenArm data on "
            f"{args.joint_state_topic}, {args.command_topic}, {args.gripper_topic}"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        with self._lock:
            for idx, name in enumerate(msg.name):
                if idx < len(msg.position):
                    self._positions[name] = msg.position[idx]

    def _on_arm_command(self, msg: Float64MultiArray) -> None:
        if not msg.data:
            return
        with self._lock:
            self._arm_command = list(msg.data)

    def _on_gripper_command(self, msg: JointTrajectory) -> None:
        if not msg.points or not msg.points[-1].positions:
            return
        with self._lock:
            self._gripper_action_m = float(msg.points[-1].positions[0])
            self._gripper_action_received = True

    def wait_ready(self, timeout_s: float = 10.0) -> None:
        deadline = time.perf_counter() + timeout_s
        required = set(LEFT_ROS_JOINT_NAMES + [LEFT_GRIPPER_ROS_JOINT_NAME])
        while time.perf_counter() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            with self._lock:
                if required.issubset(self._positions):
                    return
        with self._lock:
            missing = sorted(required.difference(self._positions))
        raise TimeoutError(
            "Timed out waiting for joint states. Missing joints: " + str(missing)
        )

    def get_frame(self) -> dict | None:
        with self._lock:
            try:
                arm_state = [
                    self._positions[name]
                    for name in LEFT_ROS_JOINT_NAMES
                ]
                gripper_state_m = self._positions[LEFT_GRIPPER_ROS_JOINT_NAME]
                arm_command = (
                    list(self._arm_command) if self._arm_command is not None else None
                )
                gripper_action_m = self._gripper_action_m
                gripper_action_received = self._gripper_action_received
            except KeyError:
                return None

        state = np.array(
            [math.degrees(value) for value in arm_state]
            + [gripper_position_to_degrees(gripper_state_m)],
            dtype=np.float32,
        )

        if arm_command is not None:
            arm_action = arm_command[:7]
            if len(arm_command) >= FEATURE_SIZE:
                gripper_action_m = arm_command[7]
            elif not gripper_action_received:
                gripper_action_m = gripper_state_m
            action = np.array(
                [math.degrees(value) for value in arm_action]
                + [gripper_position_to_degrees(gripper_action_m)],
                dtype=np.float32,
            )
        else:
            action = state.copy()

        return {
            "observation.state": state,
            "action": action,
        }


def record_episode(
    node: OpenArmXLeftRecorder,
    dataset: LeRobotDataset,
    task: str,
    duration_s: float,
    fps: int,
) -> int:
    frames = 0
    period = 1.0 / fps
    deadline = time.perf_counter() + duration_s
    next_tick = time.perf_counter()

    while time.perf_counter() < deadline:
        rclpy.spin_once(node, timeout_sec=0.0)
        frame = node.get_frame()
        if frame is not None:
            dataset.add_frame({**frame, "task": task})
            frames += 1
        next_tick += period
        sleep_for = next_tick - time.perf_counter()
        if sleep_for > 0:
            time.sleep(sleep_for)
    return frames


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-id", default="local/openarmx_left_novision")
    parser.add_argument("--root", type=Path, default=Path("/home/w/OPENARM/lerobot/data/openarmx_left_novision"))
    parser.add_argument(
        "--task",
        default="Move the object with the left OpenArm using Pico 4 VR",
    )
    parser.add_argument("--joint-state-topic", default="/joint_states")
    parser.add_argument(
        "--command-topic", default="/left_forward_position_controller/commands"
    )
    parser.add_argument(
        "--gripper-topic", default="/left_gripper_controller/joint_trajectory"
    )
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--episodes", type=int, default=5)
    parser.add_argument("--episode-time-s", type=float, default=20.0)
    parser.add_argument("--connect-timeout-s", type=float, default=10.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = OpenArmXLeftRecorder(args)
    dataset: LeRobotDataset | None = None
    total_frames = 0
    pending_frames = 0
    try:
        node.wait_ready(args.connect_timeout_s)
        print("Joint states received. Dataset will contain no camera images.")

        dataset = LeRobotDataset.create(
            repo_id=args.repo_id,
            fps=args.fps,
            root=args.root,
            robot_type="openarm_ros2_follower",
            features=make_features(),
            use_videos=False,
        )

        for episode_idx in range(1, args.episodes + 1):
            input(f"Press Enter to start episode {episode_idx}/{args.episodes}: ")
            print(f"Recording episode {episode_idx} for {args.episode_time_s:.1f}s...")
            frames = record_episode(
                node,
                dataset,
                args.task,
                args.episode_time_s,
                args.fps,
            )
            pending_frames = frames
            dataset.save_episode()
            total_frames += frames
            pending_frames = 0
            print(f"Saved episode {episode_idx}: {frames} frames")

        dataset.finalize()
        print(f"repo_id={args.repo_id}")
        print(f"root={args.root}")
        print(f"episodes={args.episodes}")
        print(f"frames={total_frames}")
        print("task=" + args.task)
    except KeyboardInterrupt:
        print("\nStopped by user.")
        if dataset is not None:
            if pending_frames > 0:
                dataset.save_episode()
                total_frames += pending_frames
            dataset.finalize()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
