#!/usr/bin/env python

from dataclasses import dataclass

from lerobot.cameras.configs import CameraConfig


@CameraConfig.register_subclass("openarm_sim")
@dataclass(kw_only=True)
class OpenArmSimCameraConfig(CameraConfig):
    """Lightweight rendered OpenArm grasp-task camera.

    This is a simulation camera for the OpenArm ROS2 fake-hardware path. It
    renders a simple table-top grasp scene from `/joint_states` so LeRobot can
    collect image observations without a physical camera.
    """

    fps: int | None = 15
    width: int | None = 424
    height: int | None = 240
    joint_state_topic: str = "/joint_states"
    warmup_s: float = 0.0
    arm: str = "right"
