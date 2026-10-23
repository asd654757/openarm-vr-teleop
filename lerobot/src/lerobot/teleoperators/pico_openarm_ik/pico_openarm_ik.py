#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

import logging
import math
import time
from threading import Lock, Thread
from typing import Any

from lerobot.types import RobotAction
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..teleoperator import Teleoperator
from .config_pico_openarm_ik import PicoOpenArmIKTeleopConfig

logger = logging.getLogger(__name__)


class PicoOpenArmIKTeleop(Teleoperator):
    """Convert teleop_xr OpenArm IK trajectories into LeRobot actions."""

    config_class = PicoOpenArmIKTeleopConfig
    name = "pico_openarm_ik"

    def __init__(self, config: PicoOpenArmIKTeleopConfig):
        super().__init__(config)
        self.config = config
        if self.config.output_arm not in {"both", "right", "left"}:
            raise ValueError(
                f"output_arm must be 'both', 'right', or 'left', got {self.config.output_arm!r}."
            )

        self._rclpy = None
        self._node = None
        self._owns_rclpy_context = False
        self._trajectory_sub = None
        self._joint_state_sub = None
        self._connected = False
        self._lock = Lock()
        self._latest_action: dict[str, float] = {}
        self._latest_joint_positions: dict[str, float] = {}
        self._received_trajectory = False
        self._trajectory_zero_positions: dict[str, float] | None = None
        # Keep ROS subscription callbacks running continuously.  The VR and
        # joint-state topics are high-rate (normally 100 Hz), while a visual
        # LeRobot recorder may sample at 30 Hz.  Servicing one callback from
        # get_action() would make actions stale and produce stair-step motion.
        self._executor = None
        self._executor_thread: Thread | None = None

    @property
    def action_features(self) -> dict[str, type]:
        features: dict[str, type] = {}
        if self.config.output_arm in {"both", "right"}:
            prefix = "right_" if self.config.output_arm == "both" else ""
            for idx in range(len(self.config.right_ros_joint_names)):
                features[f"{prefix}joint_{idx + 1}.pos"] = float
        if self.config.output_arm in {"both", "left"}:
            prefix = "left_" if self.config.output_arm == "both" else ""
            for idx in range(len(self.config.left_ros_joint_names)):
                features[f"{prefix}joint_{idx + 1}.pos"] = float
        if self.config.include_grippers:
            if self.config.output_arm in {"both", "right"}:
                features["right_gripper.pos" if self.config.output_arm == "both" else "gripper.pos"] = float
            if self.config.output_arm in {"both", "left"}:
                features["left_gripper.pos" if self.config.output_arm == "both" else "gripper.pos"] = float
        return features

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._connected

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        try:
            import rclpy
            from sensor_msgs.msg import JointState, Joy
            from trajectory_msgs.msg import JointTrajectory
            from rclpy.executors import SingleThreadedExecutor
        except ImportError as e:
            raise ImportError(
                "PicoOpenArmIKTeleop requires a sourced ROS2 Python environment with "
                "`rclpy`, `sensor_msgs`, and `trajectory_msgs` available."
            ) from e

        self._rclpy = rclpy
        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_rclpy_context = True

        node_name = f"lerobot_{self.name}"
        if self.id:
            node_name = f"{node_name}_{self.id}".replace("-", "_")
        self._node = rclpy.create_node(node_name)
        self._trajectory_sub = self._node.create_subscription(
            JointTrajectory, self.config.trajectory_topic, self._on_trajectory, 10
        )
        self._left_joy_sub = None
        self._right_joy_sub = None
        self._left_trigger_value = 0.0
        self._right_trigger_value = 0.0
        if self.config.include_grippers and self.config.enable_trigger_grippers:
            self._left_joy_sub = self._node.create_subscription(
                Joy, self.config.left_joy_topic, self._on_left_joy, 10
            )
            self._right_joy_sub = self._node.create_subscription(
                Joy, self.config.right_joy_topic, self._on_right_joy, 10
            )
        if self.config.require_joint_states:
            self._joint_state_sub = self._node.create_subscription(
                JointState, self.config.joint_state_topic, self._on_joint_state, 10
            )

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._executor_thread = Thread(
            target=self._spin_executor,
            name=f"{node_name}_ros",
            daemon=True,
        )
        self._executor_thread.start()

        try:
            self._connected = True
            if self.config.require_joint_states:
                self._wait_for_joint_states()
            else:
                self._latest_action = self._make_default_action()
        except Exception:
            self._connected = False
            self._stop_executor()
            if self._node is not None:
                self._node.destroy_node()
            if self._owns_rclpy_context and self._rclpy is not None and self._rclpy.ok():
                self._rclpy.shutdown()
            self._node = None
            self._rclpy = None
            self._trajectory_sub = None
            self._joint_state_sub = None
            self._left_joy_sub = None
            self._right_joy_sub = None
            self._owns_rclpy_context = False
            raise
        logger.info("%s connected to %s.", self, self.config.trajectory_topic)

    def _make_default_action(self) -> dict[str, float]:
        action: dict[str, float] = {}
        if self.config.output_arm in {"both", "right"}:
            prefix = "right_" if self.config.output_arm == "both" else ""
            for idx in range(len(self.config.right_ros_joint_names)):
                if self.config.relative_to_first_trajectory:
                    value = self._default_joint_degrees(self.config.relative_right_home_degrees, idx)
                else:
                    value = self._default_joint_degrees(self.config.default_right_arm_degrees, idx)
                action[f"{prefix}joint_{idx + 1}.pos"] = value
        if self.config.output_arm in {"both", "left"}:
            prefix = "left_" if self.config.output_arm == "both" else ""
            for idx in range(len(self.config.left_ros_joint_names)):
                if self.config.relative_to_first_trajectory:
                    value = self._default_joint_degrees(self.config.relative_left_home_degrees, idx)
                else:
                    value = self._default_joint_degrees(self.config.default_left_arm_degrees, idx)
                action[f"{prefix}joint_{idx + 1}.pos"] = value
        if self.config.include_grippers:
            if self.config.output_arm in {"both", "right"}:
                key = "right_gripper.pos" if self.config.output_arm == "both" else "gripper.pos"
                action[key] = self.config.gripper_open_degrees
            if self.config.output_arm in {"both", "left"}:
                key = "left_gripper.pos" if self.config.output_arm == "both" else "gripper.pos"
                action[key] = self.config.gripper_open_degrees
        return action

    def _default_joint_degrees(self, defaults: list[float], idx: int) -> float:
        if idx < len(defaults):
            return float(defaults[idx])
        return 0.0

    def _relative_joint_scale(self, scales: list[float], idx: int) -> float:
        if idx < len(scales):
            return float(scales[idx])
        return 1.0

    def _wait_for_joint_states(self) -> None:
        assert self._rclpy is not None
        deadline = time.perf_counter() + self.config.connect_timeout_s
        required = set(self.config.left_ros_joint_names + self.config.right_ros_joint_names)
        while time.perf_counter() < deadline:
            with self._lock:
                if required.issubset(self._latest_joint_positions):
                    self._latest_action = self._make_action_from_positions(self._latest_joint_positions)
                    return
            time.sleep(0.01)

        with self._lock:
            missing = sorted(required.difference(self._latest_joint_positions))
        raise TimeoutError(
            "Timed out waiting for OpenArm joint states before Pico teleop recording. "
            f"Missing joints: {missing}."
        )

    def _on_joint_state(self, msg: Any) -> None:
        with self._lock:
            for idx, name in enumerate(msg.name):
                if idx < len(msg.position):
                    self._latest_joint_positions[name] = msg.position[idx]
            if not self._received_trajectory:
                self._latest_action = self._make_action_from_positions(self._latest_joint_positions)

    def _on_trajectory(self, msg: Any) -> None:
        if not msg.points:
            return
        point = msg.points[-1]
        positions = {}
        for idx, name in enumerate(msg.joint_names):
            if idx < len(point.positions):
                positions[name] = point.positions[idx]

        with self._lock:
            self._received_trajectory = True
            if self.config.relative_to_first_trajectory and self._trajectory_zero_positions is None:
                self._trajectory_zero_positions = dict(positions)
                logger.info("%s set first %s message as relative teleop zero.", self, self.config.trajectory_topic)
            action = self._make_action_from_positions(positions, fallback=self._latest_action)
            self._latest_action = self._apply_trigger_grippers(action)

    def _on_left_joy(self, msg: Any) -> None:
        value = self._joy_trigger_value(msg, self.config.left_trigger_axis_index)
        with self._lock:
            self._left_trigger_value = value
            self._latest_action = self._apply_trigger_grippers(self._latest_action)

    def _on_right_joy(self, msg: Any) -> None:
        value = self._joy_trigger_value(msg, self.config.right_trigger_axis_index)
        with self._lock:
            self._right_trigger_value = value
            self._latest_action = self._apply_trigger_grippers(self._latest_action)

    def _joy_trigger_value(self, msg: Any, axis_index: int) -> float:
        if axis_index >= 0 and axis_index < len(msg.axes):
            return max(0.0, min(1.0, float(msg.axes[axis_index])))

        appended_axis_start = len(msg.axes) - len(msg.buttons)
        appended_axis_index = appended_axis_start + self.config.trigger_button_index
        if appended_axis_start >= 0 and appended_axis_index < len(msg.axes):
            return max(0.0, min(1.0, float(msg.axes[appended_axis_index])))

        if self.config.trigger_button_index < len(msg.buttons):
            return 1.0 if msg.buttons[self.config.trigger_button_index] else 0.0
        return 0.0

    def _make_action_from_positions(
        self, positions: dict[str, float], fallback: dict[str, float] | None = None
    ) -> dict[str, float]:
        action = dict(fallback or {})

        if self.config.output_arm in {"both", "right"}:
            prefix = "right_" if self.config.output_arm == "both" else ""
            for idx, ros_name in enumerate(self.config.right_ros_joint_names):
                key = f"{prefix}joint_{idx + 1}.pos"
                if ros_name in positions:
                    if self.config.relative_to_first_trajectory and self._trajectory_zero_positions:
                        zero = self._trajectory_zero_positions.get(ros_name, positions[ros_name])
                        value = self._default_joint_degrees(
                            self.config.relative_right_home_degrees, idx
                        ) + math.degrees(positions[ros_name] - zero) * self.config.relative_scale * self._relative_joint_scale(
                            self.config.relative_right_joint_scales, idx
                        )
                    else:
                        value = math.degrees(positions[ros_name])
                    if (
                        self.config.subtract_default_arm_degrees
                        and not self.config.relative_to_first_trajectory
                    ):
                        value -= self._default_joint_degrees(self.config.default_right_arm_degrees, idx)
                    action[key] = value

        if self.config.output_arm in {"both", "left"}:
            prefix = "left_" if self.config.output_arm == "both" else ""
            for idx, ros_name in enumerate(self.config.left_ros_joint_names):
                key = f"{prefix}joint_{idx + 1}.pos"
                if ros_name in positions:
                    if self.config.relative_to_first_trajectory and self._trajectory_zero_positions:
                        zero = self._trajectory_zero_positions.get(ros_name, positions[ros_name])
                        value = self._default_joint_degrees(
                            self.config.relative_left_home_degrees, idx
                        ) + math.degrees(positions[ros_name] - zero) * self.config.relative_scale * self._relative_joint_scale(
                            self.config.relative_left_joint_scales, idx
                        )
                    else:
                        value = math.degrees(positions[ros_name])
                    if (
                        self.config.subtract_default_arm_degrees
                        and not self.config.relative_to_first_trajectory
                    ):
                        value -= self._default_joint_degrees(self.config.default_left_arm_degrees, idx)
                    action[key] = value

        if self.config.include_grippers:
            if self.config.output_arm in {"both", "right"}:
                key = "right_gripper.pos" if self.config.output_arm == "both" else "gripper.pos"
                if self.config.right_gripper_joint_name in positions:
                    action[key] = self._gripper_position_to_degrees(
                        positions[self.config.right_gripper_joint_name]
                    )
                elif key not in action:
                    action[key] = self.config.gripper_closed_degrees

            if self.config.output_arm in {"both", "left"}:
                key = "left_gripper.pos" if self.config.output_arm == "both" else "gripper.pos"
                if self.config.left_gripper_joint_name in positions:
                    action[key] = self._gripper_position_to_degrees(
                        positions[self.config.left_gripper_joint_name]
                    )
                elif key not in action:
                    action[key] = self.config.gripper_closed_degrees

        return action

    def _spin_executor(self) -> None:
        """Continuously service Pico/ROS subscription callbacks."""
        executor = self._executor
        if executor is None:
            return
        try:
            executor.spin()
        except Exception:
            # Context shutdown interrupts spin() during normal disconnect.
            if self._rclpy is not None and self._rclpy.ok():
                logger.exception("Pico OpenArm ROS2 callback executor stopped unexpectedly.")

    def _stop_executor(self) -> None:
        executor = self._executor
        thread = self._executor_thread
        self._executor = None
        self._executor_thread = None
        if executor is not None:
            try:
                executor.shutdown(timeout_sec=1.0)
            except Exception:
                if self._rclpy is not None and self._rclpy.ok():
                    logger.exception("Failed to stop Pico OpenArm ROS2 callback executor.")
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.5)

    def _apply_trigger_grippers(self, action: dict[str, float]) -> dict[str, float]:
        if not self.config.include_grippers or not self.config.enable_trigger_grippers:
            return action

        updated = dict(action)
        if self.config.output_arm in {"both", "right"}:
            key = "right_gripper.pos" if self.config.output_arm == "both" else "gripper.pos"
            updated[key] = self._trigger_value_to_gripper_degrees(self._right_trigger_value)
        if self.config.output_arm in {"both", "left"}:
            key = "left_gripper.pos" if self.config.output_arm == "both" else "gripper.pos"
            updated[key] = self._trigger_value_to_gripper_degrees(self._left_trigger_value)
        return updated

    def _trigger_value_to_gripper_degrees(self, value: float) -> float:
        if value < self.config.trigger_pressed_threshold:
            value = 0.0
        value = max(0.0, min(1.0, value))
        return self.config.gripper_open_degrees + value * (
            self.config.gripper_closed_degrees - self.config.gripper_open_degrees
        )

    def _gripper_position_to_degrees(self, position_m: float) -> float:
        span_m = self.config.gripper_open_position_m - self.config.gripper_closed_position_m
        if span_m == 0:
            return self.config.gripper_closed_degrees
        ratio = (position_m - self.config.gripper_closed_position_m) / span_m
        ratio = max(0.0, min(1.0, ratio))
        return self.config.gripper_closed_degrees + ratio * (
            self.config.gripper_open_degrees - self.config.gripper_closed_degrees
        )

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        return None

    def configure(self) -> None:
        return None

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        with self._lock:
            action = dict(self._latest_action)

        missing = [key for key in self.action_features if key not in action]
        if missing:
            raise RuntimeError(f"Pico OpenArm IK action is missing features: {missing}")
        return action

    @check_if_not_connected
    def send_feedback(self, feedback: dict[str, Any]) -> None:
        del feedback
        return None

    @check_if_not_connected
    def disconnect(self) -> None:
        self._stop_executor()
        if self._node is not None:
            self._node.destroy_node()
        if self._owns_rclpy_context and self._rclpy is not None and self._rclpy.ok():
            self._rclpy.shutdown()

        self._node = None
        self._rclpy = None
        self._trajectory_sub = None
        self._joint_state_sub = None
        self._left_joy_sub = None
        self._right_joy_sub = None
        self._connected = False
        self._owns_rclpy_context = False
        logger.info("%s disconnected.", self)
