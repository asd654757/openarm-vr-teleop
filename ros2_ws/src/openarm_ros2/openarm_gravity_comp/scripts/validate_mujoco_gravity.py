#!/usr/bin/env python3

"""Validate OpenArm gravity feedforward in a CAN-free MuJoCo simulation."""

import argparse
import math
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np


ARM_JOINTS = {
    side: [f"openarm_{side}_joint{index}" for index in range(1, 8)]
    for side in ("left", "right")
}
GRAVITY = 9.80665
POSITION_KP = np.array([70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0] * 2)
POSITION_KD = np.array([2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5] * 2)


def rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def axis_angle_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    one_c = 1.0 - c
    return np.array(
        [
            [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
            [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
            [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
        ]
    )


def parse_vector(element: ET.Element | None, attribute: str, default: str) -> np.ndarray:
    text = default if element is None else element.get(attribute, default)
    return np.fromstring(text, sep=" ", dtype=float)


def transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


class UrdfPotential:
    def __init__(self, root: ET.Element):
        self.links = {link.get("name"): link for link in root.findall("link")}
        self.children: dict[str, list[ET.Element]] = {}
        child_links = set()
        for joint in root.findall("joint"):
            child = joint.find("child").get("link")
            parent = joint.find("parent").get("link")
            self.children.setdefault(parent, []).append(joint)
            child_links.add(child)
        roots = set(self.links) - child_links
        if len(roots) != 1:
            raise RuntimeError(f"Expected one URDF root, found {sorted(roots)}")
        self.root_link = roots.pop()

    def potential(self, positions: dict[str, float]) -> float:
        total = 0.0
        pending = [(self.root_link, np.eye(4))]
        while pending:
            link_name, world_from_link = pending.pop()
            link = self.links[link_name]
            inertial = link.find("inertial")
            if inertial is not None:
                origin = inertial.find("origin")
                center = parse_vector(origin, "xyz", "0 0 0")
                mass = float(inertial.find("mass").get("value"))
                center_world = world_from_link @ np.append(center, 1.0)
                total += mass * GRAVITY * center_world[2]

            for joint in self.children.get(link_name, []):
                origin = joint.find("origin")
                joint_tf = transform(
                    rpy_matrix(parse_vector(origin, "rpy", "0 0 0")),
                    parse_vector(origin, "xyz", "0 0 0"),
                )
                if joint.get("type") in ("revolute", "continuous"):
                    axis = parse_vector(joint.find("axis"), "xyz", "1 0 0")
                    motion = transform(
                        axis_angle_matrix(axis, positions.get(joint.get("name"), 0.0)),
                        np.zeros(3),
                    )
                elif joint.get("type") == "prismatic":
                    axis = parse_vector(joint.find("axis"), "xyz", "1 0 0")
                    motion = transform(
                        np.eye(3), axis * positions.get(joint.get("name"), 0.0)
                    )
                else:
                    motion = np.eye(4)
                child = joint.find("child").get("link")
                pending.append((child, world_from_link @ joint_tf @ motion))
        return total

    def gravity_effort(self, names: list[str], q: np.ndarray) -> np.ndarray:
        positions = dict(zip(names, q, strict=True))
        epsilon = 1e-6
        efforts = np.zeros(len(names))
        for index, name in enumerate(names):
            positions[name] += epsilon
            plus = self.potential(positions)
            positions[name] -= 2.0 * epsilon
            minus = self.potential(positions)
            positions[name] += epsilon
            efforts[index] = (plus - minus) / (2.0 * epsilon)
        return efforts


def prepare_validation_urdf(source: Path, destination: Path) -> ET.Element:
    tree = ET.parse(source)
    root = tree.getroot()
    for link in root.findall("link"):
        for child in list(link):
            if child.tag in ("visual", "collision"):
                link.remove(child)
    # ArmDynamics ends at hand_tcp. Remove the two movable finger branches so
    # both engines evaluate exactly the same seven-axis chain and hand load.
    for element in list(root):
        if "finger" in element.get("name", ""):
            root.remove(element)
    # MuJoCo's URDF static-body fusion changes the transformed inertias around
    # the fixed arm mounts and hand joint. Preserve those bodies so the loaded
    # model remains dynamically equivalent to the KDL tree.
    mujoco_options = ET.SubElement(root, "mujoco")
    ET.SubElement(
        mujoco_options,
        "compiler",
        {"discardvisual": "true", "fusestatic": "false"},
    )
    tree.write(destination, encoding="unicode")
    return root


def calculate_kdl(
    calculator: Path, urdf: Path, side: str, q: np.ndarray
) -> np.ndarray:
    command = [str(calculator), str(urdf), side, *(f"{value:.17g}" for value in q)]
    output = subprocess.check_output(command, text=True).strip()
    return np.fromstring(output, sep=",")


def set_pose(model: mujoco.MjModel, data: mujoco.MjData, pose: dict[str, np.ndarray]):
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    for side, q in pose.items():
        for name, value in zip(ARM_JOINTS[side], q, strict=True):
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            data.qpos[model.jnt_qposadr[joint_id]] = value
    mujoco.mj_forward(model, data)


def effort_vector(model: mujoco.MjModel, efforts: dict[str, np.ndarray]) -> np.ndarray:
    result = np.zeros(model.nv)
    for side, tau in efforts.items():
        for name, value in zip(ARM_JOINTS[side], tau, strict=True):
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            result[model.jnt_dofadr[joint_id]] = value
    return result


def simulate(
    model: mujoco.MjModel,
    pose: dict[str, np.ndarray],
    applied: np.ndarray,
    duration_s: float,
) -> np.ndarray:
    data = mujoco.MjData(model)
    set_pose(model, data, pose)
    initial = data.qpos.copy()
    steps = round(duration_s / model.opt.timestep)
    for _ in range(steps):
        data.qfrc_applied[:] = applied
        mujoco.mj_step(model, data)
    return data.qpos - initial


def simulate_position_hold(
    model: mujoco.MjModel,
    pose: dict[str, np.ndarray],
    calculator: Path,
    urdf: Path,
    duration_s: float,
    control_rate_hz: float,
    gravity_feedforward: bool,
) -> np.ndarray:
    data = mujoco.MjData(model)
    set_pose(model, data, pose)
    initial = data.qpos.copy()
    control_steps = max(1, round(1.0 / (control_rate_hz * model.opt.timestep)))
    gravity_effort = np.zeros(model.nv)
    steps = round(duration_s / model.opt.timestep)
    for step in range(steps):
        if gravity_feedforward and step % control_steps == 0:
            efforts = {}
            for side in ("left", "right"):
                q = np.array(
                    [
                        data.qpos[
                            model.jnt_qposadr[
                                mujoco.mj_name2id(
                                    model, mujoco.mjtObj.mjOBJ_JOINT, name
                                )
                            ]
                        ]
                        for name in ARM_JOINTS[side]
                    ]
                )
                efforts[side] = calculate_kdl(calculator, urdf, side, q)
            gravity_effort = effort_vector(model, efforts)
        position_effort = POSITION_KP * (initial - data.qpos) - POSITION_KD * data.qvel
        data.qfrc_applied[:] = position_effort + gravity_effort
        mujoco.mj_step(model, data)
    return data.qpos - initial


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--calculator", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=0.25)
    parser.add_argument("--hold-duration", type=float, default=2.0)
    parser.add_argument("--control-rate", type=float, default=100.0)
    args = parser.parse_args()

    poses = [
        {
            "left": np.array([0.0, -0.8, 0.0, 1.2, 0.0, 0.0, 0.0]),
            "right": np.array([0.0, 0.8, 0.0, 1.2, 0.0, 0.0, 0.0]),
        },
        {
            "left": np.array([0.25, -1.0, 0.3, 1.35, -0.2, 0.25, 0.15]),
            "right": np.array([-0.25, 1.0, -0.3, 1.35, 0.2, -0.25, -0.15]),
        },
        {
            "left": np.array([-0.3, -0.55, -0.35, 0.9, 0.25, -0.2, -0.2]),
            "right": np.array([0.3, 0.55, 0.35, 0.9, -0.25, 0.2, 0.2]),
        },
    ]

    with tempfile.TemporaryDirectory(prefix="openarm_gravity_") as directory:
        validation_urdf = Path(directory) / "openarm_dynamics.urdf"
        root = prepare_validation_urdf(args.urdf, validation_urdf)
        potential = UrdfPotential(root)
        model = mujoco.MjModel.from_xml_path(str(validation_urdf))
        model.opt.timestep = 0.001

        static_errors = []
        mujoco_errors = []
        drift_without = []
        drift_with = []
        for pose_index, pose in enumerate(poses, start=1):
            data = mujoco.MjData(model)
            set_pose(model, data, pose)
            kdl = {
                side: calculate_kdl(args.calculator, args.urdf, side, q)
                for side, q in pose.items()
            }
            numeric = {
                side: potential.gravity_effort(ARM_JOINTS[side], q)
                for side, q in pose.items()
            }
            kdl_all = effort_vector(model, kdl)
            numeric_all = effort_vector(model, numeric)
            error = np.linalg.norm(kdl_all - numeric_all) / max(
                np.linalg.norm(numeric_all), 1e-12
            )
            static_errors.append(error)
            mujoco_error = np.linalg.norm(kdl_all - data.qfrc_bias) / max(
                np.linalg.norm(data.qfrc_bias), 1e-12
            )
            mujoco_errors.append(mujoco_error)

            no_comp = simulate(model, pose, np.zeros(model.nv), args.duration)
            with_comp = simulate(model, pose, kdl_all, args.duration)
            no_rms = float(np.sqrt(np.mean(np.square(no_comp))))
            comp_rms = float(np.sqrt(np.mean(np.square(with_comp))))
            drift_without.append(no_rms)
            drift_with.append(comp_rms)
            print(
                f"pose={pose_index} static_error={error:.6f} "
                f"mujoco_error={mujoco_error:.6f} "
                f"drift_no_comp={no_rms:.6f}rad "
                f"drift_with_comp={comp_rms:.6f}rad"
            )

        mean_without = float(np.mean(drift_without))
        mean_with = float(np.mean(drift_with))
        improvement = 100.0 * (1.0 - mean_with / mean_without)
        print(f"max_static_error={max(static_errors):.6f}")
        print(f"max_mujoco_error={max(mujoco_errors):.6f}")
        print(f"mean_drift_no_comp={mean_without:.6f}rad")
        print(f"mean_drift_with_comp={mean_with:.6f}rad")
        print(f"drift_reduction={improvement:.2f}%")

        hold_without = simulate_position_hold(
            model,
            poses[0],
            args.calculator,
            args.urdf,
            args.hold_duration,
            args.control_rate,
            False,
        )
        hold_with = simulate_position_hold(
            model,
            poses[0],
            args.calculator,
            args.urdf,
            args.hold_duration,
            args.control_rate,
            True,
        )
        hold_without_rms = float(np.sqrt(np.mean(np.square(hold_without))))
        hold_with_rms = float(np.sqrt(np.mean(np.square(hold_with))))
        hold_with_max = float(np.max(np.abs(hold_with)))
        hold_reduction = 100.0 * (1.0 - hold_with_rms / hold_without_rms)
        print(
            f"position_hold={args.hold_duration:.2f}s@{args.control_rate:.1f}Hz "
            f"pd_only_error={hold_without_rms:.6f}rad "
            f"pd_plus_gravity_error={hold_with_rms:.6f}rad "
            f"max_error={hold_with_max:.6f}rad "
            f"error_reduction={hold_reduction:.2f}%"
        )

        passed = (
            max(static_errors) < 0.01
            and max(mujoco_errors) < 0.01
            and improvement > 80.0
            and hold_reduction > 80.0
            and hold_with_rms < 0.01
            and hold_with_max < 0.03
        )
        print(f"result={'PASS' if passed else 'FAIL'}")
        return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
