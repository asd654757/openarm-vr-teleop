#!/usr/bin/env python

"""Create a tiny synthetic Pico/OpenArm LeRobot dataset for smoke testing.

The data is intentionally simple: a sine-wave "Pico teleop" action trajectory,
lagged robot state, and a small generated front camera image. It is not useful
for learning behavior, only for validating dataset -> training -> rollout wiring.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import cv2
import numpy as np

from lerobot.datasets import LeRobotDataset


ACTION_NAMES = (
    [f"right_joint_{i}.pos" for i in range(1, 8)]
    + ["right_gripper.pos"]
    + [f"left_joint_{i}.pos" for i in range(1, 8)]
    + ["left_gripper.pos"]
)


def make_image(frame_idx: int, episode_idx: int, height: int, width: int) -> np.ndarray:
    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[:, :, 0] = (frame_idx * 9 + episode_idx * 30) % 255
    image[:, :, 1] = np.linspace(0, 220, width, dtype=np.uint8)[None, :]
    image[:, :, 2] = np.linspace(0, 180, height, dtype=np.uint8)[:, None]
    x = 8 + (frame_idx * 3) % max(1, width - 20)
    y = 16 + (episode_idx * 5) % max(1, height - 28)
    image[y : y + 12, x : x + 12] = (255, 255, 255)
    return image


def make_action(frame_idx: int, episode_idx: int, frames_per_episode: int) -> np.ndarray:
    phase = 2.0 * math.pi * (frame_idx / frames_per_episode) + episode_idx * 0.35
    action = np.zeros(len(ACTION_NAMES), dtype=np.float32)
    for idx in range(7):
        action[idx] = 18.0 * math.sin(phase + idx * 0.3)
        action[8 + idx] = 18.0 * math.sin(phase + idx * 0.3 + 0.5)
    action[7] = -25.0 + 10.0 * math.sin(phase)
    action[15] = -25.0 + 10.0 * math.cos(phase)
    return action


def write_front_video(path: Path, height: int, width: int, fps: int, frames: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError(f"Failed to open video writer for {path}")
    for idx in range(frames):
        writer.write(make_image(idx, 0, height, width))
    writer.release()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-id", default="local/openarm_pico4_smoke")
    parser.add_argument("--root", type=Path, default=Path("/tmp/lerobot_openarm_pico4_smoke"))
    parser.add_argument("--episodes", type=int, default=2)
    parser.add_argument("--frames", type=int, default=16)
    parser.add_argument("--fps", type=int, default=5)
    parser.add_argument("--height", type=int, default=64)
    parser.add_argument("--width", type=int, default=64)
    parser.add_argument("--front-video", type=Path, default=Path("/tmp/openarm_front_smoke.mp4"))
    args = parser.parse_args()

    features = {
        "observation.state": {
            "dtype": "float32",
            "shape": (len(ACTION_NAMES),),
            "names": list(ACTION_NAMES),
        },
        "observation.images.front": {
            "dtype": "image",
            "shape": (args.height, args.width, 3),
            "names": ["height", "width", "channels"],
        },
        "action": {
            "dtype": "float32",
            "shape": (len(ACTION_NAMES),),
            "names": list(ACTION_NAMES),
        },
    }

    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=args.fps,
        root=args.root,
        robot_type="bi_openarm_ros2_follower",
        features=features,
        use_videos=False,
    )

    task = "Smoke test synthetic Pico control for OpenArm"
    previous_action = np.zeros(len(ACTION_NAMES), dtype=np.float32)
    for episode_idx in range(args.episodes):
        for frame_idx in range(args.frames):
            action = make_action(frame_idx, episode_idx, args.frames)
            state = 0.85 * previous_action + 0.15 * action
            previous_action = action
            dataset.add_frame(
                {
                    "observation.state": state.astype(np.float32),
                    "observation.images.front": make_image(frame_idx, episode_idx, args.height, args.width),
                    "action": action.astype(np.float32),
                    "task": task,
                }
            )
        dataset.save_episode()

    dataset.finalize()
    write_front_video(args.front_video, args.height, args.width, args.fps, args.frames * args.episodes)

    print(f"repo_id={args.repo_id}")
    print(f"root={args.root}")
    print(f"front_video={args.front_video}")
    print(f"frames={args.episodes * args.frames}")


if __name__ == "__main__":
    main()
