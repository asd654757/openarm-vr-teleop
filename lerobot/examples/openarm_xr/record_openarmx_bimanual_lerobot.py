#!/usr/bin/env python3
"""Record bimanual OpenArmX LeRobot episodes, optionally with a global RGB camera.

The recorder is read-only with respect to the robot: it subscribes to joint
states and commands already produced by OpenArmX VR teleoperation. Arm values
are stored in degrees and gripper values in degrees (0 closed, -65 open),
matching the existing LeRobot OpenArm adapters.  Passing
``--camera-serial-number-or-name`` additionally records a fixed RealSense RGB
camera as ``observation.images.front``.
"""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path
from threading import Lock, Thread

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory

from lerobot.cameras import ColorMode, Cv2Rotation
from lerobot.cameras.realsense import RealSenseCamera, RealSenseCameraConfig
from lerobot.datasets import LeRobotDataset


SIDES = ("left", "right")
ARM_JOINT_NAMES = {
    side: [f"openarm_{side}_joint{idx}" for idx in range(1, 8)] for side in SIDES
}
GRIPPER_JOINT_NAMES = {side: f"openarm_{side}_finger_joint1" for side in SIDES}
# Keep each arm contiguous in the action/state vector: 7 arm joints + gripper.
FEATURE_NAMES = [
    name
    for side in SIDES
    for name in [*[f"{side}_joint_{idx}.pos" for idx in range(1, 8)], f"{side}_gripper.pos"]
]
FEATURE_SIZE = len(FEATURE_NAMES)

GRIPPER_CLOSED_DEGREES = 0.0
GRIPPER_OPEN_DEGREES = -65.0
GRIPPER_CLOSED_POSITION_M = 0.0
GRIPPER_OPEN_POSITION_M = 0.044


def gripper_position_to_degrees(position_m: float) -> float:
    ratio = (position_m - GRIPPER_CLOSED_POSITION_M) / (
        GRIPPER_OPEN_POSITION_M - GRIPPER_CLOSED_POSITION_M
    )
    ratio = max(0.0, min(1.0, ratio))
    return GRIPPER_CLOSED_DEGREES + ratio * (GRIPPER_OPEN_DEGREES - GRIPPER_CLOSED_DEGREES)


def make_features(image_shape: tuple[int, int, int] | None = None, use_videos: bool = False) -> dict:
    features = {
        "observation.state": {"dtype": "float32", "shape": (FEATURE_SIZE,), "names": FEATURE_NAMES},
        "action": {"dtype": "float32", "shape": (FEATURE_SIZE,), "names": FEATURE_NAMES},
    }
    if image_shape is not None:
        features["observation.images.front"] = {
            # LeRobot uses the ``video`` feature type when visual observations
            # are encoded as MP4; ``image`` is reserved for PNG-backed data.
            "dtype": "video" if use_videos else "image",
            "shape": image_shape,
            "names": ["height", "width", "channels"],
        }
    return features


class OpenArmXBimanualRecorder(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("openarmx_bimanual_lerobot_recorder")
        self._lock = Lock()
        self._positions: dict[str, float] = {}
        self._arm_commands: dict[str, list[float] | None] = {side: None for side in SIDES}
        self._gripper_commands_m = {side: GRIPPER_CLOSED_POSITION_M for side in SIDES}
        self._gripper_received = {side: False for side in SIDES}
        self._callback_counts = {
            "joint_states": 0,
            **{f"{side}_arm": 0 for side in SIDES},
            **{f"{side}_gripper": 0 for side in SIDES},
        }

        self.create_subscription(JointState, args.joint_state_topic, self._on_joint_state, 20)
        for side in SIDES:
            self.create_subscription(
                Float64MultiArray,
                getattr(args, f"{side}_command_topic"),
                lambda msg, side=side: self._on_arm_command(side, msg),
                20,
            )
            self.create_subscription(
                JointTrajectory,
                getattr(args, f"{side}_gripper_topic"),
                lambda msg, side=side: self._on_gripper_command(side, msg),
                20,
            )
        self.get_logger().info(
            "Listening for bimanual OpenArm data at 100 Hz from /joint_states, "
            f"{args.left_command_topic}, {args.right_command_topic}"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        with self._lock:
            self._callback_counts["joint_states"] += 1
            for idx, name in enumerate(msg.name):
                if idx < len(msg.position):
                    self._positions[name] = float(msg.position[idx])

    def _on_arm_command(self, side: str, msg: Float64MultiArray) -> None:
        if msg.data:
            with self._lock:
                self._callback_counts[f"{side}_arm"] += 1
                self._arm_commands[side] = list(msg.data)

    def _on_gripper_command(self, side: str, msg: JointTrajectory) -> None:
        if msg.points and msg.points[-1].positions:
            with self._lock:
                self._callback_counts[f"{side}_gripper"] += 1
                self._gripper_commands_m[side] = float(msg.points[-1].positions[0])
                self._gripper_received[side] = True

    def wait_ready(self, timeout_s: float) -> None:
        required = {name for side in SIDES for name in ARM_JOINT_NAMES[side]}
        required.update(GRIPPER_JOINT_NAMES.values())
        deadline = time.perf_counter() + timeout_s
        while time.perf_counter() < deadline:
            with self._lock:
                if required.issubset(self._positions):
                    return
            time.sleep(0.01)
        missing = sorted(required.difference(self._positions))
        raise TimeoutError(f"Timed out waiting for joint states. Missing joints: {missing}")

    def callback_counts(self) -> dict[str, int]:
        with self._lock:
            return dict(self._callback_counts)

    def get_frame(self) -> dict | None:
        with self._lock:
            required = [name for side in SIDES for name in ARM_JOINT_NAMES[side]] + list(GRIPPER_JOINT_NAMES.values())
            if not all(name in self._positions for name in required):
                return None
            state: list[float] = []
            action: list[float] = []
            for side in SIDES:
                state.extend(math.degrees(self._positions[name]) for name in ARM_JOINT_NAMES[side])
                state.append(gripper_position_to_degrees(self._positions[GRIPPER_JOINT_NAMES[side]]))
                command = self._arm_commands[side]
                arm_action = command[:7] if command and len(command) >= 7 else None
                if arm_action is None:
                    action.extend(state[-8:])
                else:
                    action.extend(math.degrees(value) for value in arm_action)
                    gripper_m = self._gripper_commands_m[side]
                    if command and len(command) >= 8:
                        gripper_m = command[7]
                    elif not self._gripper_received[side]:
                        gripper_m = self._positions[GRIPPER_JOINT_NAMES[side]]
                    action.append(gripper_position_to_degrees(gripper_m))
            return {
                "observation.state": np.asarray(state, dtype=np.float32),
                "action": np.asarray(action, dtype=np.float32),
            }


def record_episode(
    node,
    dataset,
    task: str,
    duration_s: float,
    fps: int,
    camera: RealSenseCamera | None = None,
) -> int:
    frames = 0
    period = 1.0 / fps
    deadline = time.perf_counter() + duration_s
    next_tick = time.perf_counter()
    while time.perf_counter() < deadline:
        if (frame := node.get_frame()) is not None:
            if camera is not None:
                # RealSenseCamera owns a background capture thread.  Peeking
                # avoids blocking the 30 Hz dataset loop while still taking
                # the newest RGB frame available at each sample time.
                frame["observation.images.front"] = camera.read_latest(max_age_ms=500)
            dataset.add_frame({**frame, "task": task})
            frames += 1
        next_tick += period
        if (sleep_for := next_tick - time.perf_counter()) > 0:
            time.sleep(sleep_for)
    return frames


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-id", default="local/openarmx_bimanual_novision_100hz")
    parser.add_argument(
        "--root",
        type=Path,
        default=Path("/home/w/OPENARM/lerobot/data/openarmx_bimanual_novision_100hz"),
    )
    parser.add_argument("--task", default="Move the object with both OpenArms using Pico 4 VR")
    parser.add_argument("--joint-state-topic", default="/joint_states")
    parser.add_argument("--left-command-topic", default="/left_forward_position_controller/commands")
    parser.add_argument("--right-command-topic", default="/right_forward_position_controller/commands")
    parser.add_argument("--left-gripper-topic", default="/left_gripper_controller/joint_trajectory")
    parser.add_argument("--right-gripper-topic", default="/right_gripper_controller/joint_trajectory")
    parser.add_argument("--fps", type=int, default=100)
    parser.add_argument(
        "--camera-serial-number-or-name",
        default=None,
        help="Enable the fixed global RealSense RGB camera using its serial number (recommended).",
    )
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=30)
    parser.add_argument("--camera-warmup-s", type=int, default=2)
    parser.add_argument(
        "--vcodec",
        default="h264",
        choices=["h264", "hevc", "libsvtav1", "auto"],
        help="Video codec used when a camera is enabled.",
    )
    parser.add_argument(
        "--streaming-encoding",
        action="store_true",
        help="Encode camera video while recording instead of waiting until episode save.",
    )
    parser.add_argument("--episodes", type=int, default=5)
    parser.add_argument("--episode-time-s", type=float, default=20.0)
    parser.add_argument("--connect-timeout-s", type=float, default=10.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.fps <= 0:
        raise ValueError("--fps must be positive")
    rclpy.init()
    node = OpenArmXBimanualRecorder(args)
    # ROS subscriptions must be serviced independently of the 100 Hz dataset
    # sampling loop. Calling spin_once only once per sample starves the three
    # high-rate callbacks and turns smooth commands into four-frame stairs.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = Thread(target=executor.spin, name="openarmx_recorder_ros", daemon=True)
    spin_thread.start()
    dataset = None
    camera: RealSenseCamera | None = None
    pending_frames = 0
    total_frames = 0
    try:
        node.wait_ready(args.connect_timeout_s)
        image_shape = None
        if args.camera_serial_number_or_name:
            camera_config = RealSenseCameraConfig(
                serial_number_or_name=str(args.camera_serial_number_or_name),
                fps=args.camera_fps,
                width=args.camera_width,
                height=args.camera_height,
                color_mode=ColorMode.RGB,
                use_depth=False,
                rotation=Cv2Rotation.NO_ROTATION,
                warmup_s=args.camera_warmup_s,
            )
            camera = RealSenseCamera(camera_config)
            camera.connect()
            image_shape = (int(camera.height), int(camera.width), 3)
            print(
                "Joint states received. Recording global RGB camera "
                f"{camera.serial_number} at {camera.width}x{camera.height}@{camera.fps} Hz."
            )
        else:
            print("Joint states received. Dataset will contain no camera images.")
        dataset = LeRobotDataset.create(
            repo_id=args.repo_id,
            fps=args.fps,
            root=args.root,
            robot_type="bi_openarm_ros2_follower",
            features=make_features(image_shape, use_videos=camera is not None),
            use_videos=camera is not None,
            image_writer_threads=4 if camera is not None else 0,
            vcodec=args.vcodec,
            streaming_encoding=args.streaming_encoding,
            encoder_threads=2 if args.streaming_encoding else None,
        )
        for episode_idx in range(1, args.episodes + 1):
            input(f"Press Enter to start episode {episode_idx}/{args.episodes}: ")
            print(f"Recording episode {episode_idx} for {args.episode_time_s:.1f}s at {args.fps} Hz...")
            counts_before = node.callback_counts()
            frames = record_episode(node, dataset, args.task, args.episode_time_s, args.fps, camera)
            counts_after = node.callback_counts()
            pending_frames = frames
            dataset.save_episode()
            total_frames += frames
            pending_frames = 0
            print(f"Saved episode {episode_idx}: {frames} frames")
            print(
                "ROS callbacks during episode: "
                + ", ".join(
                    f"{name}={counts_after[name] - counts_before[name]}"
                    for name in counts_after
                )
            )
        dataset.finalize()
        print(f"repo_id={args.repo_id}\nroot={args.root}\nepisodes={args.episodes}\nframes={total_frames}")
    except KeyboardInterrupt:
        print("\nStopped by user.")
        if dataset is not None:
            if pending_frames:
                dataset.save_episode()
            dataset.finalize()
    finally:
        if camera is not None and camera.is_connected:
            camera.disconnect()
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
