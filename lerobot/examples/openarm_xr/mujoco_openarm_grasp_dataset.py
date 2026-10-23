#!/usr/bin/env python

"""Collect a tiny rendered OpenArm MuJoCo grasp dataset.

This uses the official `enactic/openarm_mujoco` v2 MJCF demo scene and records
LeRobot frames from the same robot adapter used for rollout.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import cv2
import numpy as np

from lerobot.datasets import LeRobotDataset
from lerobot.robots.openarm_mujoco_follower import OpenArmMujocoFollower, OpenArmMujocoFollowerConfig
from lerobot.robots.openarm_mujoco_follower.openarm_mujoco_follower import ACTION_NAMES

TASK = "Grasp the orange cube and move it to the black target frame with OpenArm"


def default_xml_path() -> Path:
    return Path(__file__).resolve().parents[3] / "openarm_mujoco/v2/openarm_mujoco_v2/demo.xml"


def scripted_action(frame_idx: int, frames_per_episode: int, episode_idx: int) -> dict[str, float]:
    t = frame_idx / max(1, frames_per_episode - 1)
    action = _home_action()
    action.update({f"left_joint_{idx}.pos": val for idx, val in enumerate([0, 0, 0, 90, 0, 0, 0], 1)})
    action["left_gripper.pos"] = -65.0

    jitter = 5.0 * math.sin(episode_idx * 0.7)
    if t < 0.25:
        u = t / 0.25
        right = _interp_pose([0, 0, 0, 90, 0, 0, 0, -65], [28 + jitter, 44, 5, 108, 0, 8, 0, -65], u)
    elif t < 0.45:
        u = (t - 0.25) / 0.20
        right = _interp_pose([28 + jitter, 44, 5, 108, 0, 8, 0, -65], [28 + jitter, 44, 5, 108, 0, 8, 0, 0], u)
    elif t < 0.72:
        u = (t - 0.45) / 0.27
        right = _interp_pose([28 + jitter, 44, 5, 108, 0, 8, 0, 0], [-15, 62, -8, 82, 8, -4, 0, 0], u)
    else:
        u = (t - 0.72) / 0.28
        right = _interp_pose([-15, 62, -8, 82, 8, -4, 0, 0], [-15, 62, -8, 82, 8, -4, 0, -65], u)

    action.update({f"right_joint_{idx}.pos": val for idx, val in enumerate(right[:7], 1)})
    action["right_gripper.pos"] = right[7]
    return action


def _home_action() -> dict[str, float]:
    action = {name: 0.0 for name in ACTION_NAMES}
    action["right_joint_4.pos"] = 90.0
    action["left_joint_4.pos"] = 90.0
    action["right_gripper.pos"] = -65.0
    action["left_gripper.pos"] = -65.0
    return action


def _interp_pose(start: list[float], end: list[float], u: float) -> list[float]:
    u = max(0.0, min(1.0, u))
    return [s + (e - s) * u for s, e in zip(start, end, strict=True)]


def make_features(height: int, width: int, camera_keys: list[str]) -> dict:
    features = {
        "observation.state": {
            "dtype": "float32",
            "shape": (len(ACTION_NAMES),),
            "names": list(ACTION_NAMES),
        },
        "action": {
            "dtype": "float32",
            "shape": (len(ACTION_NAMES),),
            "names": list(ACTION_NAMES),
        },
    }
    for key in camera_keys:
        features[f"observation.images.{key}"] = {
            "dtype": "image",
            "shape": (height, width, 3),
            "names": ["height", "width", "channels"],
        }
    return features


def vector_from_mapping(mapping: dict[str, float]) -> np.ndarray:
    return np.array([mapping[name] for name in ACTION_NAMES], dtype=np.float32)


def write_preview(path: Path, frames: list[np.ndarray], fps: int) -> None:
    if not frames:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    height, width = frames[0].shape[:2]
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError(f"Failed to open preview video: {path}")
    for frame in frames:
        writer.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    writer.release()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-id", default="local/openarm_mujoco_grasp_smoke")
    parser.add_argument("--root", type=Path, default=Path("/tmp/lerobot_openarm_mujoco_grasp"))
    parser.add_argument("--xml-path", type=Path, default=default_xml_path())
    parser.add_argument("--episodes", type=int, default=2)
    parser.add_argument("--frames", type=int, default=24)
    parser.add_argument("--fps", type=int, default=6)
    parser.add_argument("--height", type=int, default=96)
    parser.add_argument("--width", type=int, default=128)
    parser.add_argument("--preview-video", type=Path, default=Path("/tmp/openarm_mujoco_grasp_preview.mp4"))
    parser.add_argument("--cameras", default="front:camera_head")
    args = parser.parse_args()

    camera_views = dict(item.split(":", 1) for item in args.cameras.split(",") if item)
    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=args.fps,
        root=args.root,
        robot_type="openarm_mujoco_follower",
        features=make_features(args.height, args.width, list(camera_views)),
        use_videos=False,
    )

    robot = OpenArmMujocoFollower(
        OpenArmMujocoFollowerConfig(
            xml_path=str(args.xml_path),
            calibration_dir=Path("/tmp/lerobot_openarm_mujoco_calibration"),
            fps=args.fps,
            height=args.height,
            width=args.width,
            camera_views=camera_views,
            n_substeps=8,
        )
    )
    robot.connect()
    preview_frames: list[np.ndarray] = []
    try:
        for episode_idx in range(args.episodes):
            robot._reset()  # Simulation-only dataset script; reset between demos.
            for frame_idx in range(args.frames):
                action = scripted_action(frame_idx, args.frames, episode_idx)
                sent = robot.send_action(action)
                obs = robot.get_observation()
                state = vector_from_mapping({name: float(obs[name]) for name in ACTION_NAMES})
                frame = {
                    "observation.state": state,
                    "action": vector_from_mapping({name: float(sent.get(name, action[name])) for name in ACTION_NAMES}),
                    "task": TASK,
                }
                for key in camera_views:
                    frame[f"observation.images.{key}"] = obs[key]
                dataset.add_frame(frame)
                if episode_idx == 0 and "front" in camera_views:
                    preview_frames.append(obs["front"])
            dataset.save_episode()
    finally:
        robot.disconnect()

    dataset.finalize()
    write_preview(args.preview_video, preview_frames, args.fps)
    print(f"repo_id={args.repo_id}")
    print(f"root={args.root}")
    print(f"xml_path={args.xml_path}")
    print(f"preview_video={args.preview_video}")
    print(f"frames={args.episodes * args.frames}")
    print(f"task={TASK}")


if __name__ == "__main__":
    main()
