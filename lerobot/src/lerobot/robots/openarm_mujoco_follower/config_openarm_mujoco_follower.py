#!/usr/bin/env python

from dataclasses import dataclass, field

from ..config import RobotConfig


DEFAULT_MUJOCO_CAMERA_VIEWS = {
    "front": "camera_head",
    "ceiling": "camera_ceiling",
    "right_wrist": "camera_wrist_right",
}


@RobotConfig.register_subclass("openarm_mujoco_follower")
@dataclass(kw_only=True)
class OpenArmMujocoFollowerConfig(RobotConfig):
    """OpenArm v2 MuJoCo simulation robot using official MJCF assets."""

    id: str | None = "openarm_mujoco_follower"
    xml_path: str = "~/OPENARM/openarm_mujoco/v2/openarm_mujoco_v2/demo.xml"
    keyframe: str | None = "home"
    fps: int = 15
    width: int = 424
    height: int = 240
    camera_views: dict[str, str] = field(default_factory=lambda: dict(DEFAULT_MUJOCO_CAMERA_VIEWS))
    n_substeps: int = 8
    max_relative_target: float | None = None
    reset_cube_on_connect: bool = True
