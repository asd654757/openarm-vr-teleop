#!/usr/bin/env python

from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np


RIGHT_ACTION_NAMES = [f"right_joint_{i}.pos" for i in range(1, 8)] + ["right_gripper.pos"]
LEFT_ACTION_NAMES = [f"left_joint_{i}.pos" for i in range(1, 8)] + ["left_gripper.pos"]
ACTION_NAMES = RIGHT_ACTION_NAMES + LEFT_ACTION_NAMES


@dataclass
class OpenArmGraspSceneState:
    cube_xy: np.ndarray = field(default_factory=lambda: np.array([0.52, 0.50], dtype=np.float32))
    attached: bool = False


def blank_action() -> np.ndarray:
    action = np.zeros(len(ACTION_NAMES), dtype=np.float32)
    action[7] = -65.0
    action[15] = -65.0
    return action


def scripted_grasp_action(frame_idx: int, frames_per_episode: int, episode_idx: int = 0) -> np.ndarray:
    """Return a small scripted right-arm grasp trajectory in LeRobot units.

    The action is not a physical IK solution. It is a stable, visible trajectory
    for validating the VLA data path before a full physics simulator is added.
    """

    t = frame_idx / max(1, frames_per_episode - 1)
    action = blank_action()
    action[8:15] = np.array([18.0, -25.0, 10.0, 35.0, 0.0, 0.0, 0.0], dtype=np.float32)
    action[15] = -55.0

    jitter = 4.0 * math.sin(episode_idx * 0.7)
    if t < 0.35:
        u = t / 0.35
        right = np.array(
            [
                _lerp(22.0, -34.0 + jitter, u),
                _lerp(8.0, -50.0, u),
                _lerp(0.0, 14.0, u),
                _lerp(55.0, 74.0, u),
                0.0,
                0.0,
                0.0,
                -65.0,
            ],
            dtype=np.float32,
        )
    elif t < 0.50:
        u = (t - 0.35) / 0.15
        right = np.array([-34.0 + jitter, -50.0, 14.0, 74.0, 0.0, 0.0, 0.0, _lerp(-65.0, 0.0, u)])
    elif t < 0.78:
        u = (t - 0.50) / 0.28
        right = np.array(
            [
                _lerp(-34.0 + jitter, -14.0, u),
                _lerp(-50.0, -98.0, u),
                14.0,
                _lerp(74.0, 48.0, u),
                0.0,
                0.0,
                0.0,
                0.0,
            ],
            dtype=np.float32,
        )
    else:
        u = (t - 0.78) / 0.22
        right = np.array([-14.0, -98.0, 14.0, 48.0, 0.0, 0.0, 0.0, _lerp(0.0, -65.0, u)])

    action[:8] = right.astype(np.float32)
    return action


def render_openarm_grasp(
    state: np.ndarray,
    height: int,
    width: int,
    scene_state: OpenArmGraspSceneState | None = None,
    update_scene: bool = True,
) -> np.ndarray:
    """Render a compact RGB tabletop grasp scene from a 16D OpenArm state/action."""

    if scene_state is None:
        scene_state = OpenArmGraspSceneState()

    action = np.zeros(16, dtype=np.float32)
    action[: min(16, len(state))] = state[:16]
    wrist = _right_wrist_xy(action)
    gripper = float(action[7])

    if update_scene:
        dist = float(np.linalg.norm(wrist - scene_state.cube_xy))
        if scene_state.attached:
            scene_state.cube_xy = 0.84 * wrist + 0.16 * scene_state.cube_xy
            if gripper < -35.0 and scene_state.cube_xy[1] < 0.36:
                scene_state.attached = False
        elif gripper > -18.0 and dist < 0.11:
            scene_state.attached = True

    img = np.full((height, width, 3), (238, 241, 235), dtype=np.uint8)
    _draw_table(img)
    _draw_target(img, np.array([0.57, 0.30], dtype=np.float32))
    _draw_arm(img, action, side="left")
    _draw_arm(img, action, side="right")
    _draw_cube(img, scene_state.cube_xy, scene_state.attached)
    _draw_gripper(img, wrist, gripper)
    _draw_text(img, "OpenArm virtual grasp sim")
    return img


def _lerp(a: float, b: float, u: float) -> float:
    return a + (b - a) * max(0.0, min(1.0, u))


def _right_wrist_xy(action: np.ndarray) -> np.ndarray:
    return np.array(
        [
            0.72 + float(action[0]) / 170.0 + 0.02 * math.sin(math.radians(float(action[3]))),
            0.72 + float(action[1]) / 220.0 - 0.015 * math.cos(math.radians(float(action[3]))),
        ],
        dtype=np.float32,
    )


def _left_wrist_xy(action: np.ndarray) -> np.ndarray:
    return np.array(
        [
            0.28 + float(action[8]) / 190.0,
            0.72 + float(action[9]) / 240.0,
        ],
        dtype=np.float32,
    )


def _pt(img: np.ndarray, xy: np.ndarray) -> tuple[int, int]:
    h, w = img.shape[:2]
    x = int(np.clip(xy[0], 0.02, 0.98) * (w - 1))
    y = int(np.clip(xy[1], 0.02, 0.98) * (h - 1))
    return x, y


def _draw_table(img: np.ndarray) -> None:
    h, w = img.shape[:2]
    cv2.rectangle(img, (int(0.06 * w), int(0.12 * h)), (int(0.94 * w), int(0.92 * h)), (202, 211, 198), -1)
    cv2.rectangle(img, (int(0.06 * w), int(0.12 * h)), (int(0.94 * w), int(0.92 * h)), (116, 128, 113), 2)


def _draw_target(img: np.ndarray, xy: np.ndarray) -> None:
    center = _pt(img, xy)
    radius = max(8, img.shape[1] // 34)
    cv2.circle(img, center, radius, (88, 160, 115), 2)
    cv2.circle(img, center, max(2, radius // 4), (88, 160, 115), -1)


def _draw_cube(img: np.ndarray, xy: np.ndarray, attached: bool) -> None:
    x, y = _pt(img, xy)
    size = max(10, img.shape[1] // 30)
    color = (235, 129, 68) if not attached else (246, 172, 70)
    cv2.rectangle(img, (x - size // 2, y - size // 2), (x + size // 2, y + size // 2), color, -1)
    cv2.rectangle(img, (x - size // 2, y - size // 2), (x + size // 2, y + size // 2), (90, 75, 60), 1)


def _draw_arm(img: np.ndarray, action: np.ndarray, side: str) -> None:
    if side == "right":
        base = np.array([0.76, 0.86], dtype=np.float32)
        wrist = _right_wrist_xy(action)
        elbow = 0.50 * base + 0.50 * wrist + np.array([0.03, -0.08], dtype=np.float32)
        color = (80, 112, 210)
    else:
        base = np.array([0.24, 0.86], dtype=np.float32)
        wrist = _left_wrist_xy(action)
        elbow = 0.52 * base + 0.48 * wrist + np.array([-0.02, -0.06], dtype=np.float32)
        color = (110, 130, 150)

    points = [_pt(img, base), _pt(img, elbow), _pt(img, wrist)]
    cv2.line(img, points[0], points[1], color, 7, cv2.LINE_AA)
    cv2.line(img, points[1], points[2], color, 6, cv2.LINE_AA)
    for point in points:
        cv2.circle(img, point, max(4, img.shape[1] // 70), (48, 56, 64), -1)


def _draw_gripper(img: np.ndarray, wrist: np.ndarray, gripper: float) -> None:
    x, y = _pt(img, wrist)
    opening = int(np.interp(gripper, [-65.0, 0.0], [16, 4]))
    length = max(10, img.shape[1] // 32)
    cv2.line(img, (x - opening, y - 2), (x - opening - length, y - 7), (35, 42, 47), 3, cv2.LINE_AA)
    cv2.line(img, (x + opening, y - 2), (x + opening + length, y - 7), (35, 42, 47), 3, cv2.LINE_AA)


def _draw_text(img: np.ndarray, text: str) -> None:
    cv2.putText(
        img,
        text,
        (10, max(18, img.shape[0] // 12)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (55, 64, 70),
        1,
        cv2.LINE_AA,
    )
