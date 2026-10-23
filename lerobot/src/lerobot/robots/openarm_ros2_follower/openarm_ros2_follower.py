#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import math
import time
from functools import cached_property
from threading import Lock, Thread
from typing import Any

from lerobot.cameras import make_cameras_from_configs
from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_openarm_ros2_follower import OpenArmROS2FollowerConfig

logger = logging.getLogger(__name__)


def _resolve_topic(namespace: str, topic: str) -> str:
    if topic.startswith("/"):
        return topic
    namespace = namespace.strip("/")
    if namespace:
        return f"/{namespace}/{topic}"
    return f"/{topic}"


class OpenArmROS2Follower(Robot):
    """LeRobot robot adapter for OpenArm ROS2 controllers.

    This class is intended for the OpenArm ROS2 mock/simulation path first:

    `ros2 launch openarm_bringup openarm.launch.py use_fake_hardware:=true
    robot_controller:=forward_position_controller`
    """

    config_class = OpenArmROS2FollowerConfig
    name = "openarm_ros2_follower"

    def __init__(self, config: OpenArmROS2FollowerConfig):
        super().__init__(config)
        self.config = config
        if len(config.ros_joint_names) != len(config.feature_names):
            raise ValueError("ros_joint_names and feature_names must have the same length.")

        self.cameras = make_cameras_from_configs(config.cameras)
        self._node = None
        self._rclpy = None
        self._owns_rclpy_context = False
        self._arm_command_pub = None
        self._joint_state_sub = None
        self._gripper_action_client = None
        self._gripper_action_type = None
        self._gripper_goal_futures = []
        self._connected = False
        self._lock = Lock()
        # ROS callbacks must be serviced independently of LeRobot's control
        # loop.  The latter may run at 30 Hz while OpenArmX publishes at
        # 100 Hz; calling spin_once() from get_observation() would otherwise
        # leave high-rate messages queued and make the sampled trajectory
        # appear as multi-frame stair steps.
        self._executor = None
        self._executor_thread: Thread | None = None
        self._latest_positions: dict[str, float] = {}
        self._latest_velocities: dict[str, float] = {}
        self._last_arm_command = [0.0] * len(config.ros_joint_names)
        self._last_gripper_command = config.gripper_closed_degrees

    @property
    def _motor_features(self) -> dict[str, type]:
        features = {f"{self.config.feature_prefix}{motor}.pos": float for motor in self.config.feature_names}
        if self.config.include_gripper:
            features[f"{self.config.feature_prefix}{self.config.gripper_feature_name}.pos"] = float
        return features

    @property
    def _camera_features(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motor_features, **self._camera_features}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motor_features

    @property
    def is_connected(self) -> bool:
        return self._connected and all(cam.is_connected for cam in self.cameras.values())

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        try:
            import rclpy
            from sensor_msgs.msg import JointState
            from std_msgs.msg import Float64MultiArray
            from rclpy.executors import SingleThreadedExecutor
            if self.config.include_gripper:
                from control_msgs.action import GripperCommand
                from rclpy.action import ActionClient
            else:
                GripperCommand = None
                ActionClient = None
        except ImportError as e:
            raise ImportError(
                "OpenArmROS2Follower requires a sourced ROS2 Python environment with "
                "`rclpy`, `sensor_msgs`, and `std_msgs` available. Gripper support also "
                "requires `control_msgs`."
            ) from e

        self._rclpy = rclpy
        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_rclpy_context = True

        node_name = f"lerobot_{self.name}"
        if self.id:
            node_name = f"{node_name}_{self.id}".replace("-", "_")
        self._node = rclpy.create_node(node_name)

        joint_state_topic = _resolve_topic(self.config.namespace, self.config.joint_state_topic)
        command_topic = _resolve_topic(self.config.namespace, self.config.command_topic)
        self._arm_command_pub = self._node.create_publisher(Float64MultiArray, command_topic, 10)
        self._joint_state_sub = self._node.create_subscription(
            JointState, joint_state_topic, self._on_joint_state, 10
        )

        if self.config.include_gripper:
            gripper_action_name = _resolve_topic(self.config.namespace, self.config.gripper_action_name)
            self._gripper_action_type = GripperCommand
            self._gripper_action_client = ActionClient(self._node, GripperCommand, gripper_action_name)

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._executor_thread = Thread(target=self._spin_executor, name=f"{node_name}_ros", daemon=True)
        self._executor_thread.start()

        try:
            for cam in self.cameras.values():
                cam.connect()
            self._connected = True
            self._wait_for_joint_states()
        except Exception:
            self._connected = False
            self._stop_executor()
            for cam in self.cameras.values():
                cam.disconnect()
            if self._node is not None:
                self._node.destroy_node()
            if self._owns_rclpy_context and self._rclpy is not None and self._rclpy.ok():
                self._rclpy.shutdown()
            self._node = None
            self._rclpy = None
            self._arm_command_pub = None
            self._joint_state_sub = None
            self._gripper_action_client = None
            self._owns_rclpy_context = False
            raise
        logger.info("%s connected to ROS2 topics.", self)

    def _on_joint_state(self, msg: Any) -> None:
        with self._lock:
            for idx, name in enumerate(msg.name):
                if idx < len(msg.position):
                    self._latest_positions[name] = msg.position[idx]
                if idx < len(msg.velocity):
                    self._latest_velocities[name] = msg.velocity[idx]

    def _spin_executor(self) -> None:
        """Continuously service ROS callbacks in a background thread."""
        executor = self._executor
        if executor is None:
            return
        try:
            executor.spin()
        except Exception:
            # A context shutdown naturally interrupts executor.spin().  Do
            # not emit a traceback during normal disconnect; retain logging
            # for unexpected failures while the ROS context is still alive.
            if self._rclpy is not None and self._rclpy.ok():
                logger.exception("OpenArm ROS2 callback executor stopped unexpectedly.")

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
                    logger.exception("Failed to stop OpenArm ROS2 callback executor.")
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.5)

    def _wait_for_joint_states(self) -> None:
        assert self._rclpy is not None
        deadline = time.perf_counter() + self.config.connect_timeout_s
        required = set(self.config.ros_joint_names)
        if self.config.include_gripper:
            required.add(self.config.ros_gripper_joint_name)

        while time.perf_counter() < deadline:
            with self._lock:
                if required.issubset(self._latest_positions):
                    return
            time.sleep(0.01)

        with self._lock:
            missing = sorted(required.difference(self._latest_positions))
        raise TimeoutError(
            "Timed out waiting for OpenArm ROS2 joint states. Missing joints: "
            f"{missing}. Start OpenArm ROS2 with fake hardware and "
            "`robot_controller:=forward_position_controller` first."
        )

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        return None

    def configure(self) -> None:
        return None

    def setup_motors(self) -> None:
        raise NotImplementedError("OpenArmROS2Follower uses ROS2 controllers; motor setup is external.")

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        obs: dict[str, Any] = {}
        with self._lock:
            for feature_name, ros_name in zip(
                self.config.feature_names, self.config.ros_joint_names, strict=True
            ):
                value_deg = math.degrees(self._latest_positions.get(ros_name, 0.0))
                obs[f"{self.config.feature_prefix}{feature_name}.pos"] = value_deg

            if self.config.include_gripper:
                gripper_position_m = self._latest_positions.get(self.config.ros_gripper_joint_name, 0.0)
                obs[f"{self.config.feature_prefix}{self.config.gripper_feature_name}.pos"] = (
                    self._gripper_position_to_degrees(gripper_position_m)
                )

        for cam_key, cam in self.cameras.items():
            obs[cam_key] = cam.read_latest()

        return obs

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        sent_action: dict[str, float] = {}
        goal_pos: dict[str, float] = {}

        for feature_name in self.config.feature_names:
            key = f"{self.config.feature_prefix}{feature_name}.pos"
            if key in action:
                goal_pos[feature_name] = float(action[key])

        if self.config.include_gripper:
            gripper_key = f"{self.config.feature_prefix}{self.config.gripper_feature_name}.pos"
            if gripper_key in action:
                goal_pos[self.config.gripper_feature_name] = float(action[gripper_key])

        if self.config.max_relative_target is not None:
            obs = self.get_observation()
            goal_present_pos = {
                key: (target, obs[f"{self.config.feature_prefix}{key}.pos"])
                for key, target in goal_pos.items()
                if f"{self.config.feature_prefix}{key}.pos" in obs
            }
            goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        for key, value in goal_pos.items():
            if key in self.config.joint_limits:
                min_limit, max_limit = self.config.joint_limits[key]
                value = max(min_limit, min(max_limit, value))
            sent_action[f"{self.config.feature_prefix}{key}.pos"] = value

        self._publish_arm_command(sent_action)
        self._send_gripper_command(sent_action)
        return sent_action

    def _publish_arm_command(self, action: dict[str, float]) -> None:
        assert self._arm_command_pub is not None
        from std_msgs.msg import Float64MultiArray

        for idx, feature_name in enumerate(self.config.feature_names):
            key = f"{self.config.feature_prefix}{feature_name}.pos"
            if key in action:
                self._last_arm_command[idx] = math.radians(action[key])

        msg = Float64MultiArray()
        msg.data = list(self._last_arm_command)
        self._arm_command_pub.publish(msg)

    def _send_gripper_command(self, action: dict[str, float]) -> None:
        if not self.config.include_gripper or self._gripper_action_client is None:
            return

        key = f"{self.config.feature_prefix}{self.config.gripper_feature_name}.pos"
        if key not in action:
            return

        if abs(action[key] - self._last_gripper_command) < 1e-6:
            return
        self._last_gripper_command = action[key]
        if not self._gripper_action_client.wait_for_server(
            timeout_sec=self.config.gripper_server_timeout_s
        ):
            logger.warning(
                "OpenArm ROS2 gripper action server is not available yet; skipping gripper command."
            )
            return

        goal = self._gripper_action_type.Goal()
        goal.command.position = self._gripper_degrees_to_position(action[key])
        goal.command.max_effort = self.config.gripper_max_effort
        self._gripper_goal_futures = [future for future in self._gripper_goal_futures if not future.done()]
        self._gripper_goal_futures.append(self._gripper_action_client.send_goal_async(goal))

    def _gripper_degrees_to_position(self, degrees: float) -> float:
        span_deg = self.config.gripper_open_degrees - self.config.gripper_closed_degrees
        if span_deg == 0:
            return self.config.gripper_closed_position_m
        ratio = (degrees - self.config.gripper_closed_degrees) / span_deg
        ratio = max(0.0, min(1.0, ratio))
        return self.config.gripper_closed_position_m + ratio * (
            self.config.gripper_open_position_m - self.config.gripper_closed_position_m
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

    @check_if_not_connected
    def disconnect(self) -> None:
        self._stop_executor()
        for cam in self.cameras.values():
            cam.disconnect()

        if self._node is not None:
            self._node.destroy_node()
        if self._owns_rclpy_context and self._rclpy is not None and self._rclpy.ok():
            self._rclpy.shutdown()

        self._node = None
        self._rclpy = None
        self._arm_command_pub = None
        self._joint_state_sub = None
        self._gripper_action_client = None
        self._gripper_goal_futures = []
        self._connected = False
        self._owns_rclpy_context = False
        logger.info("%s disconnected.", self)
