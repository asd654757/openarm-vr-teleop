#!/usr/bin/env python

"""Generate a tiny rendered OpenArm grasp-task LeRobot dataset.

This is the first useful simulation-data step after the pure synthetic smoke
test: images come from a virtual tabletop grasp renderer and actions follow a
simple Pico-like demonstration trajectory.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from lerobot.cameras.openarm_sim.render_openarm_grasp import (
    ACTION_NAMES,
    OpenArmGraspSceneState,
    render_openarm_grasp,
    scripted_grasp_action,
)
from lerobot.datasets import LeRobotDataset


TASK = "Grasp the orange cube and move it to the green target with OpenArm"


def write_preview_video(path: Path, frames: list[np.ndarray], fps: int) -> None:
    if not frames:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    height, width = frames[0].shape[:2]
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError(f"Failed to open video writer for {path}")
    for frame in frames:
        writer.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    writer.release()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-id", default="local/openarm_rendered_grasp_smoke")
    parser.add_argument("--root", type=Path, default=Path("/tmp/lerobot_openarm_rendered_grasp"))
    parser.add_argument("--episodes", type=int, default=3)
    parser.add_argument("--frames", type=int, default=32)
    parser.add_argument("--fps", type=int, default=8)
    parser.add_argument("--height", type=int, default=128)
    parser.add_argument("--width", type=int, default=192)
    parser.add_argument("--preview-video", type=Path, default=Path("/tmp/openarm_rendered_grasp_preview.mp4"))
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

    preview_frames: list[np.ndarray] = []
    previous_action = np.zeros(len(ACTION_NAMES), dtype=np.float32)
    for episode_idx in range(args.episodes):
        scene_state = OpenArmGraspSceneState()
        previous_action[:] = 0.0
        previous_action[7] = -65.0
        previous_action[15] = -65.0

        for frame_idx in range(args.frames):
            action = scripted_grasp_action(frame_idx, args.frames, episode_idx)
            state = 0.75 * previous_action + 0.25 * action
            previous_action = state
            image = render_openarm_grasp(state, args.height, args.width, scene_state)
            if episode_idx == 0:
                preview_frames.append(image)

            dataset.add_frame(
                {
                    "observation.state": state.astype(np.float32),
                    "observation.images.front": image,
                    "action": action.astype(np.float32),
                    "task": TASK,
                }
            )

        dataset.save_episode()

    dataset.finalize()
    write_preview_video(args.preview_video, preview_frames, args.fps)

    print(f"repo_id={args.repo_id}")
    print(f"root={args.root}")
    print(f"preview_video={args.preview_video}")
    print(f"frames={args.episodes * args.frames}")
    print(f"task={TASK}")


if __name__ == "__main__":
    main()
