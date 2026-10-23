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

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from ..config import RobotConfig

OPENARM_FEATURE_NAMES = (
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
    "joint_7",
)

OPENARM_SINGLE_JOINT_NAMES = (
    "openarm_joint1",
    "openarm_joint2",
    "openarm_joint3",
    "openarm_joint4",
    "openarm_joint5",
    "openarm_joint6",
    "openarm_joint7",
)

OPENARM_LEFT_JOINT_NAMES = (
    "openarm_left_joint1",
    "openarm_left_joint2",
    "openarm_left_joint3",
    "openarm_left_joint4",
    "openarm_left_joint5",
    "openarm_left_joint6",
    "openarm_left_joint7",
)

OPENARM_RIGHT_JOINT_NAMES = (
    "openarm_right_joint1",
    "openarm_right_joint2",
    "openarm_right_joint3",
    "openarm_right_joint4",
    "openarm_right_joint5",
    "openarm_right_joint6",
    "openarm_right_joint7",
)

DEFAULT_JOINT_LIMITS_DEGREES: dict[str, tuple[float, float]] = {
    "joint_1": (-75.0, 75.0),
    "joint_2": (-90.0, 90.0),
    "joint_3": (-85.0, 85.0),
    "joint_4": (0.0, 135.0),
    "joint_5": (-85.0, 85.0),
    "joint_6": (-40.0, 40.0),
    "joint_7": (-80.0, 80.0),
    "gripper": (-65.0, 0.0),
}


@dataclass
class OpenArmROS2FollowerConfigBase:
    """ROS2 controller bridge for an OpenArm arm.

    LeRobot actions/observations are exposed in the same units as the existing
    OpenArm CAN follower: arm joints in degrees and gripper in degrees
    (0 closed, -65 open). ROS2 topics use radians for arm joints and meters for
    the gripper finger joint.
    """

    namespace: str = ""
    joint_state_topic: str = "joint_states"
    command_topic: str = "forward_position_controller/commands"
    gripper_action_name: str = "gripper_controller/gripper_cmd"

    ros_joint_names: list[str] = field(default_factory=lambda: list(OPENARM_SINGLE_JOINT_NAMES))
    feature_names: list[str] = field(default_factory=lambda: list(OPENARM_FEATURE_NAMES))
    feature_prefix: str = ""

    include_gripper: bool = True
    ros_gripper_joint_name: str = "openarm_finger_joint1"
    gripper_feature_name: str = "gripper"
    gripper_closed_degrees: float = 0.0
    gripper_open_degrees: float = -65.0
    gripper_closed_position_m: float = 0.0
    gripper_open_position_m: float = 0.044
    gripper_max_effort: float = 0.0

    joint_limits: dict[str, tuple[float, float]] = field(
        default_factory=lambda: dict(DEFAULT_JOINT_LIMITS_DEGREES)
    )
    max_relative_target: float | dict[str, float] | None = None

    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    connect_timeout_s: float = 5.0
    gripper_server_timeout_s: float = 0.2


@RobotConfig.register_subclass("openarm_ros2_follower")
@dataclass
class OpenArmROS2FollowerConfig(RobotConfig, OpenArmROS2FollowerConfigBase):
    id: str | None = "openarm_ros2_follower"


@RobotConfig.register_subclass("bi_openarm_ros2_follower")
@dataclass(kw_only=True)
class BiOpenArmROS2FollowerConfig(RobotConfig):
    """Bimanual OpenArm ROS2 follower.

    The default topics match `openarm.bimanual.launch.py` with
    `robot_controller:=forward_position_controller`.
    """

    id: str | None = "bi_openarm_ros2_follower"
    namespace: str = ""
    joint_state_topic: str = "joint_states"

    left_arm_config: OpenArmROS2FollowerConfigBase = field(
        default_factory=lambda: OpenArmROS2FollowerConfigBase(
            command_topic="left_forward_position_controller/commands",
            gripper_action_name="left_gripper_controller/gripper_cmd",
            ros_joint_names=list(OPENARM_LEFT_JOINT_NAMES),
            ros_gripper_joint_name="openarm_left_finger_joint1",
            feature_prefix="left_",
        )
    )
    right_arm_config: OpenArmROS2FollowerConfigBase = field(
        default_factory=lambda: OpenArmROS2FollowerConfigBase(
            command_topic="right_forward_position_controller/commands",
            gripper_action_name="right_gripper_controller/gripper_cmd",
            ros_joint_names=list(OPENARM_RIGHT_JOINT_NAMES),
            ros_gripper_joint_name="openarm_right_finger_joint1",
            feature_prefix="right_",
        )
    )
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
