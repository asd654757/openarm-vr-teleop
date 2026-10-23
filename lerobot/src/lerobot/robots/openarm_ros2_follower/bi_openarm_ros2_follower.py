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

from dataclasses import asdict
from functools import cached_property

from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..robot import Robot
from .config_openarm_ros2_follower import BiOpenArmROS2FollowerConfig, OpenArmROS2FollowerConfig
from .openarm_ros2_follower import OpenArmROS2Follower


class BiOpenArmROS2Follower(Robot):
    """Bimanual OpenArm ROS2 follower for ROS2 mock/simulation and hardware."""

    config_class = BiOpenArmROS2FollowerConfig
    name = "bi_openarm_ros2_follower"

    def __init__(self, config: BiOpenArmROS2FollowerConfig):
        super().__init__(config)
        self.config = config

        left_cameras = config.cameras or config.left_arm_config.cameras
        right_cameras = {} if config.cameras else config.right_arm_config.cameras

        self.left_arm = OpenArmROS2Follower(
            self._make_arm_config(
                config.left_arm_config,
                id_=f"{config.id}_left" if config.id else None,
                namespace=config.namespace,
                joint_state_topic=config.joint_state_topic,
                cameras=left_cameras,
            )
        )
        self.right_arm = OpenArmROS2Follower(
            self._make_arm_config(
                config.right_arm_config,
                id_=f"{config.id}_right" if config.id else None,
                namespace=config.namespace,
                joint_state_topic=config.joint_state_topic,
                cameras=right_cameras,
            )
        )
        self.cameras = {**self.left_arm.cameras, **self.right_arm.cameras}

    def _make_arm_config(
        self,
        arm_config,
        id_: str | None,
        namespace: str,
        joint_state_topic: str,
        cameras,
    ) -> OpenArmROS2FollowerConfig:
        data = asdict(arm_config)
        data["id"] = id_
        data["calibration_dir"] = self.config.calibration_dir
        data["namespace"] = arm_config.namespace or namespace
        data["joint_state_topic"] = joint_state_topic
        data["cameras"] = cameras
        return OpenArmROS2FollowerConfig(**data)

    @cached_property
    def observation_features(self) -> dict:
        return {**self.right_arm.observation_features, **self.left_arm.observation_features}

    @cached_property
    def action_features(self) -> dict:
        return {**self.right_arm.action_features, **self.left_arm.action_features}

    @property
    def is_connected(self) -> bool:
        return self.left_arm.is_connected and self.right_arm.is_connected

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        self.left_arm.connect(calibrate)
        self.right_arm.connect(calibrate)

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        return None

    def configure(self) -> None:
        return None

    def setup_motors(self) -> None:
        raise NotImplementedError("BiOpenArmROS2Follower uses ROS2 controllers; motor setup is external.")

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        return {**self.right_arm.get_observation(), **self.left_arm.get_observation()}

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        sent_right = self.right_arm.send_action(action)
        sent_left = self.left_arm.send_action(action)
        return {**sent_right, **sent_left}

    @check_if_not_connected
    def disconnect(self) -> None:
        self.right_arm.disconnect()
        self.left_arm.disconnect()
