#!/usr/bin/env python

from __future__ import annotations

import logging
import math
import os
from pathlib import Path
from typing import Any

import numpy as np

from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_openarm_mujoco_follower import OpenArmMujocoFollowerConfig

logger = logging.getLogger(__name__)

ACTION_NAMES = (
    [f"right_joint_{idx}.pos" for idx in range(1, 8)]
    + ["right_gripper.pos"]
    + [f"left_joint_{idx}.pos" for idx in range(1, 8)]
    + ["left_gripper.pos"]
)

RIGHT_ACTUATORS = [f"right_joint{idx}_ctrl" for idx in range(1, 8)] + ["right_finger1_ctrl"]
LEFT_ACTUATORS = [f"left_joint{idx}_ctrl" for idx in range(1, 8)] + ["left_finger1_ctrl"]
RIGHT_JOINTS = [f"openarm_right_joint{idx}" for idx in range(1, 8)] + ["openarm_right_finger_joint1"]
LEFT_JOINTS = [f"openarm_left_joint{idx}" for idx in range(1, 8)] + ["openarm_left_finger_joint1"]


class OpenArmMujocoFollower(Robot):
    """LeRobot robot adapter for the official `enactic/openarm_mujoco` MJCF."""

    config_class = OpenArmMujocoFollowerConfig
    name = "openarm_mujoco_follower"

    def __init__(self, config: OpenArmMujocoFollowerConfig):
        super().__init__(config)
        self.config = config
        self._mujoco = None
        self._model = None
        self._data = None
        self._renderer = None
        self._actuator_ids: dict[str, int] = {}
        self._joint_qpos_addr: dict[str, int] = {}
        self._connected = False
        self.cameras = {key: self for key in config.camera_views}

    @property
    def observation_features(self) -> dict[str, type | tuple]:
        motor_features = {name: float for name in ACTION_NAMES}
        camera_features = {
            key: (self.config.height, self.config.width, 3) for key in self.config.camera_views
        }
        return {**motor_features, **camera_features}

    @property
    def action_features(self) -> dict[str, type]:
        return {name: float for name in ACTION_NAMES}

    @property
    def is_connected(self) -> bool:
        return self._connected

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        os.environ.setdefault("MUJOCO_GL", "egl")
        try:
            import mujoco
        except ImportError as e:
            raise ImportError("Install MuJoCo first: `python -m pip install mujoco`.") from e

        self._mujoco = mujoco
        xml_path = Path(self.config.xml_path).expanduser()
        if not xml_path.exists():
            raise FileNotFoundError(f"OpenArm MuJoCo XML does not exist: {xml_path}")

        self._model = mujoco.MjModel.from_xml_path(str(xml_path))
        self._data = mujoco.MjData(self._model)
        self._renderer = mujoco.Renderer(self._model, self.config.height, self.config.width)
        self._resolve_names()
        self._reset()
        self._connected = True
        logger.info("%s connected to MuJoCo model %s.", self, xml_path)

    def _resolve_names(self) -> None:
        assert self._mujoco is not None and self._model is not None
        for name in [*RIGHT_ACTUATORS, *LEFT_ACTUATORS]:
            idx = self._mujoco.mj_name2id(self._model, self._mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if idx < 0:
                raise ValueError(f"Missing actuator in MuJoCo model: {name}")
            self._actuator_ids[name] = int(idx)

        for name in [*RIGHT_JOINTS, *LEFT_JOINTS]:
            idx = self._mujoco.mj_name2id(self._model, self._mujoco.mjtObj.mjOBJ_JOINT, name)
            if idx < 0:
                raise ValueError(f"Missing joint in MuJoCo model: {name}")
            self._joint_qpos_addr[name] = int(self._model.jnt_qposadr[idx])

        for camera_name in self.config.camera_views.values():
            idx = self._mujoco.mj_name2id(self._model, self._mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
            if idx < 0:
                raise ValueError(f"Missing camera in MuJoCo model: {camera_name}")

    def _reset(self) -> None:
        assert self._mujoco is not None and self._model is not None and self._data is not None
        if self.config.keyframe:
            key_id = self._mujoco.mj_name2id(
                self._model, self._mujoco.mjtObj.mjOBJ_KEY, self.config.keyframe
            )
            if key_id >= 0:
                self._mujoco.mj_resetDataKeyframe(self._model, self._data, key_id)
            else:
                self._mujoco.mj_resetData(self._model, self._data)
        else:
            self._mujoco.mj_resetData(self._model, self._data)

        if self.config.reset_cube_on_connect:
            self._reset_body_pose("orange_cube", [0.45, 0.0, 1.05], [1.0, 0.0, 0.0, 0.0])

        self._sync_ctrl_to_qpos()
        self._mujoco.mj_forward(self._model, self._data)

    def _reset_body_pose(self, body_name: str, pos: list[float], quat: list[float]) -> None:
        assert self._mujoco is not None and self._model is not None and self._data is not None
        body_id = self._mujoco.mj_name2id(self._model, self._mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id < 0:
            return
        joint_id = self._model.body_jntadr[body_id]
        if joint_id < 0:
            return
        qpos_addr = self._model.jnt_qposadr[joint_id]
        self._data.qpos[qpos_addr : qpos_addr + 3] = pos
        self._data.qpos[qpos_addr + 3 : qpos_addr + 7] = quat

    def _sync_ctrl_to_qpos(self) -> None:
        assert self._model is not None and self._data is not None
        for actuator_idx in range(self._model.nu):
            joint_id = self._model.actuator_trnid[actuator_idx, 0]
            if joint_id >= 0:
                self._data.ctrl[actuator_idx] = self._data.qpos[self._model.jnt_qposadr[joint_id]]

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        return None

    def configure(self) -> None:
        return None

    def setup_motors(self) -> None:
        return None

    @check_if_not_connected
    def reset(self) -> None:
        """Reset the MuJoCo scene to the configured keyframe and object start pose."""
        self._reset()

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        obs: dict[str, Any] = self._read_state_degrees()
        for key, camera_name in self.config.camera_views.items():
            obs[key] = self._render(camera_name)
        return obs

    def _read_state_degrees(self) -> dict[str, float]:
        assert self._data is not None
        obs: dict[str, float] = {}
        for idx, joint_name in enumerate(RIGHT_JOINTS[:7]):
            obs[ACTION_NAMES[idx]] = math.degrees(float(self._data.qpos[self._joint_qpos_addr[joint_name]]))
        obs["right_gripper.pos"] = _right_finger_rad_to_degrees(
            float(self._data.qpos[self._joint_qpos_addr["openarm_right_finger_joint1"]])
        )
        for idx, joint_name in enumerate(LEFT_JOINTS[:7]):
            obs[ACTION_NAMES[8 + idx]] = math.degrees(
                float(self._data.qpos[self._joint_qpos_addr[joint_name]])
            )
        obs["left_gripper.pos"] = _left_finger_rad_to_degrees(
            float(self._data.qpos[self._joint_qpos_addr["openarm_left_finger_joint1"]])
        )
        return obs

    def _render(self, camera_name: str) -> np.ndarray:
        assert self._mujoco is not None and self._data is not None and self._renderer is not None
        self._renderer.update_scene(self._data, camera=camera_name)
        return self._renderer.render().copy()

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        assert self._mujoco is not None and self._model is not None and self._data is not None
        goal = {key: float(action[key]) for key in ACTION_NAMES if key in action}
        if self.config.max_relative_target is not None:
            obs = self._read_state_degrees()
            goal_present_pos = {key: (target, obs[key]) for key, target in goal.items()}
            goal = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        for idx, actuator_name in enumerate(RIGHT_ACTUATORS[:7]):
            key = ACTION_NAMES[idx]
            if key in goal:
                self._data.ctrl[self._actuator_ids[actuator_name]] = math.radians(goal[key])
        if "right_gripper.pos" in goal:
            self._data.ctrl[self._actuator_ids["right_finger1_ctrl"]] = _right_gripper_degrees_to_rad(
                goal["right_gripper.pos"]
            )

        for idx, actuator_name in enumerate(LEFT_ACTUATORS[:7]):
            key = ACTION_NAMES[8 + idx]
            if key in goal:
                self._data.ctrl[self._actuator_ids[actuator_name]] = math.radians(goal[key])
        if "left_gripper.pos" in goal:
            self._data.ctrl[self._actuator_ids["left_finger1_ctrl"]] = _left_gripper_degrees_to_rad(
                goal["left_gripper.pos"]
            )

        for _ in range(max(1, self.config.n_substeps)):
            self._mujoco.mj_step(self._model, self._data)
        return goal

    @check_if_not_connected
    def disconnect(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
        self._renderer = None
        self._data = None
        self._model = None
        self._mujoco = None
        self._actuator_ids = {}
        self._joint_qpos_addr = {}
        self._connected = False
        logger.info("%s disconnected.", self)


def _right_gripper_degrees_to_rad(degrees: float) -> float:
    return float(np.interp(degrees, [-65.0, 0.0], [-0.7854, 0.0]))


def _left_gripper_degrees_to_rad(degrees: float) -> float:
    return float(np.interp(degrees, [-65.0, 0.0], [0.7854, 0.0]))


def _right_finger_rad_to_degrees(rad: float) -> float:
    return float(np.interp(rad, [-0.7854, 0.0], [-65.0, 0.0]))


def _left_finger_rad_to_degrees(rad: float) -> float:
    return float(np.interp(rad, [0.7854, 0.0], [-65.0, 0.0]))
