#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

from dataclasses import dataclass, field

from lerobot.robots.openarm_ros2_follower.config_openarm_ros2_follower import (
    OPENARM_LEFT_JOINT_NAMES,
    OPENARM_RIGHT_JOINT_NAMES,
)

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("pico_openarm_ik")
@dataclass
class PicoOpenArmIKTeleopConfig(TeleoperatorConfig):
    """Pico 4 / teleop_xr OpenArm IK teleoperator.

    This teleoperator subscribes to the `trajectory_msgs/JointTrajectory`
    produced by `python -m teleop_xr.ros2 --mode ik --robot-class openarm`
    and exposes those targets as LeRobot bimanual OpenArm actions.
    """

    id: str | None = "pico_openarm_ik"
    trajectory_topic: str = "/joint_trajectory"
    joint_state_topic: str = "/joint_states"
    output_arm: str = "both"

    left_ros_joint_names: list[str] = field(default_factory=lambda: list(OPENARM_LEFT_JOINT_NAMES))
    right_ros_joint_names: list[str] = field(default_factory=lambda: list(OPENARM_RIGHT_JOINT_NAMES))

    include_grippers: bool = True
    left_gripper_joint_name: str = "openarm_left_finger_joint1"
    right_gripper_joint_name: str = "openarm_right_finger_joint1"
    gripper_closed_degrees: float = 0.0
    gripper_open_degrees: float = -65.0
    gripper_closed_position_m: float = 0.0
    gripper_open_position_m: float = 0.044
    default_left_arm_degrees: list[float] = field(
        default_factory=lambda: [0.0, -45.84, 0.0, 68.75, 0.0, 0.0, 0.0]
    )
    default_right_arm_degrees: list[float] = field(
        default_factory=lambda: [0.0, 45.84, 0.0, 68.75, 0.0, 0.0, 0.0]
    )
    subtract_default_arm_degrees: bool = False
    relative_to_first_trajectory: bool = False
    relative_scale: float = 1.0
    relative_left_joint_scales: list[float] = field(default_factory=lambda: [1.0] * 7)
    relative_right_joint_scales: list[float] = field(default_factory=lambda: [1.0] * 7)
    relative_left_home_degrees: list[float] = field(default_factory=lambda: [0.0] * 7)
    relative_right_home_degrees: list[float] = field(default_factory=lambda: [0.0] * 7)

    enable_trigger_grippers: bool = True
    left_joy_topic: str = "/xr/controller_left/joy"
    right_joy_topic: str = "/xr/controller_right/joy"
    trigger_button_index: int = 0
    left_trigger_axis_index: int = -1
    right_trigger_axis_index: int = -1
    trigger_pressed_threshold: float = 0.2

    require_joint_states: bool = True
    connect_timeout_s: float = 5.0
