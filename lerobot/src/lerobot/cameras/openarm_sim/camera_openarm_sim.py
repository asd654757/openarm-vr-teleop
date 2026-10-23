#!/usr/bin/env python

from __future__ import annotations

import time
from threading import Lock
from typing import Any

import numpy as np
from numpy.typing import NDArray

from lerobot.cameras.camera import Camera

from .configuration_openarm_sim import OpenArmSimCameraConfig
from .render_openarm_grasp import OpenArmGraspSceneState, blank_action, render_openarm_grasp


RIGHT_ROS_JOINTS = [f"openarm_right_joint{i}" for i in range(1, 8)]
LEFT_ROS_JOINTS = [f"openarm_left_joint{i}" for i in range(1, 8)]


class OpenArmSimCamera(Camera):
    """Rendered virtual camera for the OpenArm fake-hardware grasp task."""

    def __init__(self, config: OpenArmSimCameraConfig):
        super().__init__(config)
        self.config = config
        self.width = config.width or 424
        self.height = config.height or 240
        self.fps = config.fps or 15
        self._connected = False
        self._lock = Lock()
        self._latest_positions: dict[str, float] = {}
        self._node = None
        self._rclpy = None
        self._owns_rclpy_context = False
        self._last_frame = render_openarm_grasp(blank_action(), self.height, self.width)
        self._last_frame_time = 0.0
        self._scene_state = OpenArmGraspSceneState()

    @property
    def is_connected(self) -> bool:
        return self._connected

    @staticmethod
    def find_cameras() -> list[dict[str, Any]]:
        return [{"type": "openarm_sim", "name": "front", "backend": "software_renderer"}]

    def connect(self, warmup: bool = True) -> None:
        del warmup
        if self._connected:
            return

        try:
            import rclpy
            from sensor_msgs.msg import JointState
        except ImportError:
            rclpy = None
            JointState = None

        self._rclpy = rclpy
        if rclpy is not None:
            if not rclpy.ok():
                rclpy.init(args=None)
                self._owns_rclpy_context = True
            self._node = rclpy.create_node("lerobot_openarm_sim_camera")
            self._node.create_subscription(JointState, self.config.joint_state_topic, self._on_joint_state, 10)

        self._connected = True
        if self.config.warmup_s > 0:
            deadline = time.perf_counter() + self.config.warmup_s
            while time.perf_counter() < deadline:
                self.read()
                time.sleep(1.0 / self.fps)

    def _on_joint_state(self, msg: Any) -> None:
        with self._lock:
            for idx, name in enumerate(msg.name):
                if idx < len(msg.position):
                    self._latest_positions[name] = float(msg.position[idx])

    def read(self, color_mode: Any | None = None) -> NDArray[Any]:
        del color_mode
        if not self._connected:
            raise RuntimeError("OpenArmSimCamera is not connected.")
        return self._render_latest()

    def async_read(self, timeout_ms: float = 200) -> NDArray[Any]:
        del timeout_ms
        return self.read()

    def read_latest(self, max_age_ms: int = 500) -> NDArray[Any]:
        del max_age_ms
        return self.read()

    def disconnect(self) -> None:
        if self._node is not None:
            self._node.destroy_node()
        if self._owns_rclpy_context and self._rclpy is not None and self._rclpy.ok():
            self._rclpy.shutdown()
        self._node = None
        self._rclpy = None
        self._owns_rclpy_context = False
        self._connected = False

    def _render_latest(self) -> NDArray[Any]:
        now = time.perf_counter()
        min_period = 1.0 / max(1, self.fps)
        if now - self._last_frame_time < min_period:
            return self._last_frame.copy()

        if self._rclpy is not None and self._node is not None:
            self._rclpy.spin_once(self._node, timeout_sec=0.0)

        action = blank_action()
        with self._lock:
            for idx, name in enumerate(RIGHT_ROS_JOINTS):
                action[idx] = np.degrees(self._latest_positions.get(name, 0.0))
            for idx, name in enumerate(LEFT_ROS_JOINTS):
                action[8 + idx] = np.degrees(self._latest_positions.get(name, 0.0))

            right_gripper = self._latest_positions.get("openarm_right_finger_joint1")
            left_gripper = self._latest_positions.get("openarm_left_finger_joint1")
            if right_gripper is not None:
                action[7] = float(np.interp(right_gripper, [0.0, 0.044], [0.0, -65.0]))
            if left_gripper is not None:
                action[15] = float(np.interp(left_gripper, [0.0, 0.044], [0.0, -65.0]))

        self._last_frame = render_openarm_grasp(action, self.height, self.width, self._scene_state)
        self._last_frame_time = now
        return self._last_frame.copy()
