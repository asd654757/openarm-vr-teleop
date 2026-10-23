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

"""Map teleop_xr ROS2 controller topics to OpenArm bimanual position commands.

This is intentionally a simple simulation-first mapper. Hold the deadman button,
move each Pico/WebXR controller relative to the capture pose, and this node sends
bounded joint-position commands to OpenArm's ROS2 fake hardware controllers.
"""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass, field

import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64MultiArray

from lerobot.robots.openarm_ros2_follower.config_openarm_ros2_follower import (
    DEFAULT_JOINT_LIMITS_DEGREES,
    OPENARM_FEATURE_NAMES,
    OPENARM_LEFT_JOINT_NAMES,
    OPENARM_RIGHT_JOINT_NAMES,
)


def _resolve_topic(topic: str) -> str:
    return topic if topic.startswith("/") else f"/{topic}"


def _clamp(value: float, limits: tuple[float, float]) -> float:
    return max(limits[0], min(limits[1], value))


def _pose_xyz(msg: PoseStamped) -> tuple[float, float, float]:
    return (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)


@dataclass
class ArmRuntime:
    side: str
    joint_names: tuple[str, ...]
    command_topic: str
    gripper_action: str
    pose: PoseStamped | None = None
    pose_received_at: float = 0.0
    joy: Joy | None = None
    snapshot_xyz: tuple[float, float, float] | None = None
    snapshot_deg: list[float] | None = None
    desired_deg: list[float] | None = None
    active: bool = False
    last_gripper_deg: float | None = None
    last_gripper_sent_at: float = 0.0
    publisher: object | None = None
    gripper_client: ActionClient | None = None
    have_warned_missing_state: bool = False
    last_missing_state_log_at: float = field(default_factory=time.monotonic)


class OpenArmXRMapper(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("openarm_xr_mapper")
        self.args = args
        self.latest_joint_positions_deg: dict[str, float] = {}

        self.arms = {
            "left": ArmRuntime(
                side="left",
                joint_names=tuple(OPENARM_LEFT_JOINT_NAMES),
                command_topic=args.left_command_topic,
                gripper_action=args.left_gripper_action,
            ),
            "right": ArmRuntime(
                side="right",
                joint_names=tuple(OPENARM_RIGHT_JOINT_NAMES),
                command_topic=args.right_command_topic,
                gripper_action=args.right_gripper_action,
            ),
        }

        for arm in self.arms.values():
            arm.publisher = self.create_publisher(
                Float64MultiArray, _resolve_topic(arm.command_topic), 10
            )
            arm.gripper_client = ActionClient(
                self, GripperCommand, _resolve_topic(arm.gripper_action)
            )

        self.create_subscription(JointState, _resolve_topic(args.joint_state_topic), self._on_joint_state, 10)
        self.create_subscription(
            PoseStamped,
            _resolve_topic(args.left_pose_topic),
            lambda msg: self._on_pose("left", msg),
            10,
        )
        self.create_subscription(
            PoseStamped,
            _resolve_topic(args.right_pose_topic),
            lambda msg: self._on_pose("right", msg),
            10,
        )
        self.create_subscription(
            Joy,
            _resolve_topic(args.left_joy_topic),
            lambda msg: self._on_joy("left", msg),
            10,
        )
        self.create_subscription(
            Joy,
            _resolve_topic(args.right_joy_topic),
            lambda msg: self._on_joy("right", msg),
            10,
        )

        self.create_timer(1.0 / args.rate_hz, self._tick)
        self.get_logger().info(
            "OpenArm XR mapper ready. Hold deadman button "
            f"{args.deadman_button} and move Pico/WebXR controllers."
        )

    def _on_joint_state(self, msg: JointState) -> None:
        for idx, name in enumerate(msg.name):
            if idx < len(msg.position):
                self.latest_joint_positions_deg[name] = math.degrees(float(msg.position[idx]))

    def _on_pose(self, side: str, msg: PoseStamped) -> None:
        arm = self.arms[side]
        arm.pose = msg
        arm.pose_received_at = time.monotonic()

    def _on_joy(self, side: str, msg: Joy) -> None:
        self.arms[side].joy = msg

    def _tick(self) -> None:
        for arm in self.arms.values():
            self._update_arm(arm)

    def _update_arm(self, arm: ArmRuntime) -> None:
        if not self._has_joint_state(arm):
            self._log_missing_joint_state(arm)
            return

        if arm.desired_deg is None:
            arm.desired_deg = self._current_degrees(arm)

        should_move = self._is_deadman_active(arm) and self._pose_is_fresh(arm)
        if not should_move:
            arm.active = False
            arm.snapshot_xyz = None
            arm.snapshot_deg = None
            self._handle_gripper_buttons(arm)
            self._publish_arm(arm)
            return

        if not arm.active:
            arm.active = True
            arm.snapshot_xyz = _pose_xyz(arm.pose)
            arm.snapshot_deg = self._current_degrees(arm)
            arm.desired_deg = list(arm.snapshot_deg)
            self.get_logger().info(f"{arm.side} controller engaged")

        assert arm.pose is not None
        assert arm.snapshot_xyz is not None
        assert arm.snapshot_deg is not None

        x, y, z = _pose_xyz(arm.pose)
        x0, y0, z0 = arm.snapshot_xyz
        deltas = {"x": x - x0, "y": y - y0, "z": z - z0}
        joy_x, joy_y = self._thumbstick_axes(arm)

        target = list(arm.snapshot_deg)
        target[0] += self.args.yaw_sign * deltas[self.args.yaw_axis] * self.args.yaw_gain_deg_per_m
        target[1] += self.args.reach_sign * deltas[self.args.reach_axis] * self.args.reach_gain_deg_per_m
        target[3] += self.args.lift_sign * deltas[self.args.lift_axis] * self.args.lift_gain_deg_per_m
        target[4] += self.args.wrist_x_sign * joy_x * self.args.wrist_gain_deg
        target[5] += self.args.wrist_y_sign * joy_y * self.args.wrist_gain_deg

        arm.desired_deg = self._clamp_degrees(target)
        self._handle_gripper_buttons(arm)
        self._publish_arm(arm)

    def _has_joint_state(self, arm: ArmRuntime) -> bool:
        return all(name in self.latest_joint_positions_deg for name in arm.joint_names)

    def _log_missing_joint_state(self, arm: ArmRuntime) -> None:
        now = time.monotonic()
        if now - arm.last_missing_state_log_at < 2.0:
            return
        arm.last_missing_state_log_at = now
        missing = [name for name in arm.joint_names if name not in self.latest_joint_positions_deg]
        self.get_logger().warning(
            f"Waiting for /joint_states before commanding {arm.side}; missing {missing[:3]}"
        )

    def _current_degrees(self, arm: ArmRuntime) -> list[float]:
        if arm.desired_deg is not None and arm.active:
            return list(arm.desired_deg)
        return [self.latest_joint_positions_deg[name] for name in arm.joint_names]

    def _is_deadman_active(self, arm: ArmRuntime) -> bool:
        if not self.args.require_deadman:
            return True
        return self._button_pressed(arm.joy, self.args.deadman_button)

    def _pose_is_fresh(self, arm: ArmRuntime) -> bool:
        if arm.pose is None:
            return False
        return (time.monotonic() - arm.pose_received_at) <= self.args.max_pose_age_s

    def _button_pressed(self, joy: Joy | None, index: int) -> bool:
        return bool(joy is not None and 0 <= index < len(joy.buttons) and joy.buttons[index])

    def _thumbstick_axes(self, arm: ArmRuntime) -> tuple[float, float]:
        if arm.joy is None:
            return 0.0, 0.0
        return (
            self._axis_value(arm.joy, self.args.thumbstick_x_axis),
            self._axis_value(arm.joy, self.args.thumbstick_y_axis),
        )

    def _axis_value(self, joy: Joy, index: int) -> float:
        if 0 <= index < len(joy.axes):
            value = float(joy.axes[index])
            return 0.0 if abs(value) < self.args.thumbstick_deadzone else value
        return 0.0

    def _clamp_degrees(self, degrees: list[float]) -> list[float]:
        return [
            _clamp(value, DEFAULT_JOINT_LIMITS_DEGREES[feature])
            for value, feature in zip(degrees, OPENARM_FEATURE_NAMES, strict=True)
        ]

    def _publish_arm(self, arm: ArmRuntime) -> None:
        if arm.desired_deg is None or arm.publisher is None:
            return
        msg = Float64MultiArray()
        msg.data = [math.radians(value) for value in arm.desired_deg]
        arm.publisher.publish(msg)

    def _handle_gripper_buttons(self, arm: ArmRuntime) -> None:
        if self._button_pressed(arm.joy, self.args.gripper_close_button):
            self._send_gripper(arm, self.args.gripper_closed_deg)
        elif self._button_pressed(arm.joy, self.args.gripper_open_button):
            self._send_gripper(arm, self.args.gripper_open_deg)

    def _send_gripper(self, arm: ArmRuntime, degrees: float) -> None:
        now = time.monotonic()
        if arm.last_gripper_deg is not None and abs(arm.last_gripper_deg - degrees) < 1e-6:
            return
        if now - arm.last_gripper_sent_at < self.args.gripper_min_interval_s:
            return
        if arm.gripper_client is None:
            return
        if not arm.gripper_client.wait_for_server(timeout_sec=0.0):
            return

        goal = GripperCommand.Goal()
        goal.command.position = self._gripper_degrees_to_meters(degrees)
        goal.command.max_effort = self.args.gripper_max_effort
        arm.gripper_client.send_goal_async(goal)
        arm.last_gripper_deg = degrees
        arm.last_gripper_sent_at = now

    def _gripper_degrees_to_meters(self, degrees: float) -> float:
        closed_deg = self.args.gripper_closed_deg
        open_deg = self.args.gripper_open_deg
        if abs(open_deg - closed_deg) < 1e-9:
            return self.args.gripper_closed_m
        ratio = (degrees - closed_deg) / (open_deg - closed_deg)
        ratio = max(0.0, min(1.0, ratio))
        return self.args.gripper_closed_m + ratio * (self.args.gripper_open_m - self.args.gripper_closed_m)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--joint-state-topic", default="joint_states")
    parser.add_argument("--left-pose-topic", default="xr/controller_left/pose")
    parser.add_argument("--right-pose-topic", default="xr/controller_right/pose")
    parser.add_argument("--left-joy-topic", default="xr/controller_left/joy")
    parser.add_argument("--right-joy-topic", default="xr/controller_right/joy")
    parser.add_argument("--left-command-topic", default="left_forward_position_controller/commands")
    parser.add_argument("--right-command-topic", default="right_forward_position_controller/commands")
    parser.add_argument("--left-gripper-action", default="left_gripper_controller/gripper_cmd")
    parser.add_argument("--right-gripper-action", default="right_gripper_controller/gripper_cmd")
    parser.add_argument("--rate-hz", type=float, default=30.0)
    parser.add_argument("--max-pose-age-s", type=float, default=0.5)
    parser.add_argument("--require-deadman", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--deadman-button", type=int, default=0)
    parser.add_argument("--gripper-close-button", type=int, default=1)
    parser.add_argument("--gripper-open-button", type=int, default=2)
    parser.add_argument("--yaw-gain-deg-per-m", type=float, default=90.0)
    parser.add_argument("--reach-gain-deg-per-m", type=float, default=80.0)
    parser.add_argument("--lift-gain-deg-per-m", type=float, default=90.0)
    parser.add_argument("--wrist-gain-deg", type=float, default=25.0)
    parser.add_argument("--yaw-axis", choices=("x", "y", "z"), default="y")
    parser.add_argument("--reach-axis", choices=("x", "y", "z"), default="x")
    parser.add_argument("--lift-axis", choices=("x", "y", "z"), default="z")
    parser.add_argument("--yaw-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--reach-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--lift-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--wrist-x-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--wrist-y-sign", type=float, choices=(-1.0, 1.0), default=-1.0)
    parser.add_argument("--thumbstick-x-axis", type=int, default=0)
    parser.add_argument("--thumbstick-y-axis", type=int, default=1)
    parser.add_argument("--thumbstick-deadzone", type=float, default=0.08)
    parser.add_argument("--gripper-closed-deg", type=float, default=0.0)
    parser.add_argument("--gripper-open-deg", type=float, default=-45.0)
    parser.add_argument("--gripper-closed-m", type=float, default=0.0)
    parser.add_argument("--gripper-open-m", type=float, default=0.044)
    parser.add_argument("--gripper-max-effort", type=float, default=0.0)
    parser.add_argument("--gripper-min-interval-s", type=float, default=0.25)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = OpenArmXRMapper(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
