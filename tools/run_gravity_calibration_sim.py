#!/usr/bin/env python3
"""Offline MuJoCo gravity identification and fixed-target hold experiments.

No ROS initialization, sockets, CAN interfaces, or hardware command paths.
All observations and errors are explicitly simulation data.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timedelta, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
NAMES = [f"openarm_{side}_joint{j}" for side in ("left", "right") for j in range(1, 8)]
LIMITS = np.array([2., 2., 2., 2., .5, .5, .5] * 2)
ALPHAS = [0., .2, .4, .5, .6, .8, 1., 1.2]
GRAVITY = 9.80665


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def load_validator():
    path = ROOT / "ros2_ws/src/openarm_ros2/openarm_gravity_comp/scripts/validate_mujoco_gravity.py"
    spec = importlib.util.spec_from_file_location("offline_gravity_validator", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def gains(urdf):
    controls = ET.parse(urdf).getroot().findall("ros2_control")
    configured, wire = {}, {}
    for side in ("left", "right"):
        matches = [c for c in controls if f"_{side}_" in c.get("name", "")]
        if len(matches) != 1:
            raise ValueError(f"Need one ros2_control entry for {side}")
        values = {p.get("name"): float(p.text) for p in matches[0].findall("hardware/param")
                  if p.get("name", "").startswith(("kp", "kd"))}
        for kind, maximum in (("kp", 500.), ("kd", 5.)):
            v = np.array([values[f"{kind}{j}"] for j in range(1, 8)])
            configured[f"{side}_{kind}"] = v.tolist()
            wire[f"{side}_{kind}"] = (np.floor(np.clip(v, 0, maximum) / maximum * 4095)
                                       / 4095 * maximum).tolist()
    return np.array(wire["left_kp"] + wire["right_kp"]), np.array(wire["left_kd"] + wire["right_kd"]), configured, wire


def indices(model):
    ids = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in NAMES])
    if np.any(ids < 0) or model.nq != 14 or model.nv != 14:
        raise ValueError("Expected exactly fourteen arm joints")
    return model.jnt_qposadr[ids].copy(), model.jnt_dofadr[ids].copy(), ids


def model_from(path):
    model = mujoco.MjModel.from_xml_path(str(path))
    model.opt.gravity[:] = [0, 0, -GRAVITY]
    model.opt.timestep = .001
    model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    model.dof_frictionloss[:] = 0
    model.dof_damping[:] = 0
    return model


def gravity(model, data, q, qi, vi):
    data.qpos[qi] = q
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)
    return data.qfrc_bias[vi].copy()


def plant_file(nominal, target, mass_scale, payload):
    tree = ET.parse(nominal)
    root = tree.getroot()
    if mass_scale != 1:
        for element in root.findall("link/inertial/mass"):
            element.set("value", str(float(element.get("value")) * mass_scale))
        for element in root.findall("link/inertial/inertia"):
            for name, value in list(element.attrib.items()):
                element.set(name, str(float(value) * mass_scale))
    if payload:
        link = ET.SubElement(root, "link", name="simulation_only_right_payload")
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "origin", xyz="0 0 0", rpy="0 0 0")
        ET.SubElement(inertial, "mass", value=str(payload))
        ET.SubElement(inertial, "inertia", ixx="0.0001", iyy="0.0001", izz="0.0001",
                      ixy="0", ixz="0", iyz="0")
        joint = ET.SubElement(root, "joint", name="simulation_only_payload_mount", type="fixed")
        ET.SubElement(joint, "parent", link="openarm_right_hand_tcp")
        ET.SubElement(joint, "child", link="simulation_only_right_payload")
        ET.SubElement(joint, "origin", xyz="0 0 0.10", rpy="0 0 0")
    tree.write(target, encoding="unicode")


def make_record(session, directory, pose, repeat, split, tag, alpha, limits, q, dq,
                torque, ff, target, urdf_sha, approach="fixed"):
    directory = session / "recordings" / directory
    directory.mkdir(parents=True)
    summary = {}
    rows = []
    for j, name in enumerate(NAMES):
        summary[name] = {"n": len(q), "q": float(q[:, j].mean()),
                         "tau": float(torque[:, j].mean()),
                         "tau_std": float(torque[:, j].std(ddof=1)),
                         "ff_mean": float(ff[:, j].mean()),
                         "qd": float(target[j]), "q_std_rad": float(q[:, j].std(ddof=1)),
                         "q_span_rad": float(np.ptp(q[:, j])),
                         "max_speed_rad_s": float(np.max(np.abs(dq[:, j]))),
                         "tracking_rmse_rad": float(np.sqrt(np.mean((q[:, j] - target[j])**2)))}
        for k in range(len(q)):
            rows.append([k*.01, 0, 0, name, q[k, j], dq[k, j], torque[k, j], ff[k, j], target[j]])
    with (directory / "raw.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["elapsed_s", "stamp_sec", "stamp_nanosec", "joint", "q_rad",
                         "dq_rad_s", "tau_feedback_nm", "ff_preview_nm", "q_des_rad"])
        writer.writerows(rows)
    reasons = []
    if split == "deployment":
        if np.max(np.abs(dq)) > .03:
            reasons.append("End window exceeds 0.03 rad/s")
        if np.max(np.ptp(q, axis=0)) > np.deg2rad(.5):
            reasons.append("End window angle span exceeds 0.5 degrees")
    save(directory / "record.json", {
        "pose": pose, "repeat": repeat, "split": split, "tag": tag, "approach": approach,
        "simulation_only": True, "accepted": not reasons, "reasons": reasons,
        "summary": summary, "raw_sha256": sha(directory / "raw.csv"), "urdf_sha256": urdf_sha,
        "gravity_parameters": {"gravity_scale": alpha, "torque_limits": limits.tolist(),
            "enabled": True, "apply_effort": True, "enable_left": True, "enable_right": True,
            "ramp_duration_s": 10., "state_timeout_s": .1, "publish_rate_hz": 100.}})
    return not reasons


def simulate_hold(plant, nominal, q_target, qi, vi, nqi, nvi, kp, kd, alpha, limits,
                  repeat, pose_index, duration):
    data, nominal_data = mujoco.MjData(plant), mujoco.MjData(nominal)
    rng = np.random.default_rng(30000 + pose_index*10 + repeat)
    data.qpos[qi] = q_target + rng.normal(0, .002, 14)
    data.qvel[:] = 0
    mujoco.mj_forward(plant, data)
    # An implicit viscous term implements -Kd*qdot algebraically, avoiding
    # numerical artifacts from explicitly discretizing a stiff derivative law.
    plant.dof_damping[vi] = kd
    steps = round(duration / plant.opt.timestep)
    interval = round(.01 / plant.opt.timestep)
    ff = np.zeros(14)
    records = []
    for step in range(steps):
        q = data.qpos[qi].copy()
        dq = data.qvel[vi].copy()
        if step % interval == 0:
            g = gravity(nominal, nominal_data, q, nqi, nvi)
            effective = alpha * min(step*plant.opt.timestep / 10., 1.)
            ff = np.clip(effective*g, -limits, limits)
        position = kp*(q_target-q)
        data.qfrc_applied[:] = 0
        data.qfrc_applied[vi] = position + ff
        if step >= steps - 1000 and step % interval == 0:
            records.append((q, dq, position-kd*dq+ff, ff.copy()))
        mujoco.mj_step(plant, data)
        if not np.isfinite(data.qpos).all() or np.max(np.abs(data.qvel)) > 100:
            raise ValueError("Simulation became unstable")
    arrays = [np.array([row[i] for row in records]) for i in range(4)]
    return arrays


def call_tool(session, command, extra):
    log = session / f"{command}_cli.log"
    result = subprocess.run([sys.executable, str(ROOT/"tools/gravity_calibration.py"), command,
                             "--session", str(session), *extra], text=True, capture_output=True)
    log.write_text(result.stdout + "\n" + result.stderr)
    if result.returncode:
        raise ValueError(f"{command} failed; inspect {log}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--calculator", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--hold-seconds", type=float, default=13.)
    args = parser.parse_args()
    if args.hold_seconds < 12:
        parser.error("Allow the 10-second ramp to finish and settle; hold >=12 seconds")
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ["MPLCONFIGDIR"] = str(args.output / ".mplconfig")
    validator = load_validator()
    nominal_file = args.output / "nominal_dynamics.urdf"
    validator.prepare_validation_urdf(args.urdf, nominal_file)
    nominal = model_from(nominal_file)
    nqi, nvi, joint_ids = indices(nominal)
    kp, kd, configured, wire = gains(args.urdf)
    rng = np.random.default_rng(20261023)
    center = np.array([0., -.65, 0., .9, 0., 0., 0., 0., .65, 0., .9, 0., 0., 0.])
    spread = np.tile([.45, .35, .55, .3, .45, .4, .5], 2)
    poses = center + rng.uniform(-1, 1, (20, 14))*spread
    for j, joint in enumerate(joint_ids):
        if nominal.jnt_limited[joint]:
            low, high = nominal.jnt_range[joint]
            poses[:, j] = np.clip(poses[:, j], low+.15, high-.15)
    # Independent KDL vs MuJoCo consistency check before using MuJoCo nominal
    # gravity to avoid a subprocess at every 100-Hz control update.
    cross_errors = []
    nd = mujoco.MjData(nominal)
    for q in poses:
        kdl = np.concatenate([validator.calculate_kdl(args.calculator, args.urdf, side, q[i:i+7])
                              for side, i in [("left", 0), ("right", 7)]])
        mg = gravity(nominal, nd, q, nqi, nvi)
        cross_errors.append(float(np.linalg.norm(kdl-mg) / max(np.linalg.norm(kdl), 1e-12)))
    if max(cross_errors) > .001:
        raise ValueError(f"KDL/MuJoCo mismatch: {max(cross_errors)}")
    scenarios = [
        ("nominal_unclipped", 1., 0., np.full(14, 1000.), 1.),
        ("mass_0_8_unclipped", .8, 0., np.full(14, 1000.), .8),
        ("nominal_current_limits", 1., 0., LIMITS, 1.),
        ("right_payload_0_2kg_current_limits", 1., .2, LIMITS, None),
    ]
    results, sweep = [], []
    started = time.monotonic()
    for case, mass_scale, payload, limits, expected in scenarios:
        print(f"START {case}", flush=True)
        session = args.output / case
        session.mkdir()
        (session / "robot.urdf").write_bytes(args.urdf.read_bytes())
        model_sha = sha(session / "robot.urdf")
        plant_path = session / "plant_dynamics.urdf"
        plant_file(nominal_file, plant_path, mass_scale, payload)
        plant = model_from(plant_path)
        qi, vi, _ = indices(plant)
        save(session / "session.json", {"schema": 1, "simulation_only": True,
            "urdf_sha256": model_sha, "warnings": ["SIMULATION ONLY: NOT A PHYSICAL CALIBRATION"],
            "torque_measurement_note": "MuJoCo static inverse dynamics + known synthetic measurement bias/noise",
            "known_mass_scale": mass_scale, "known_right_payload_kg": payload})
        generator = np.random.default_rng(65536)
        pd = mujoco.MjData(plant)
        bias = np.linspace(-.02, .02, 14)
        for p, base in enumerate(poses):
            for repeat in range(1, 5):
                q_true = base + generator.normal(0, .001, 14)
                true_tau = gravity(plant, pd, q_true, qi, vi)
                nominal_tau = gravity(nominal, nd, q_true, nqi, nvi)
                q = q_true + generator.normal(0, .0001, (50, 14))
                torque = true_tau + bias + generator.normal(0, .02, (50, 14))
                ff = np.tile(np.clip(.2*nominal_tau, -limits, limits), (50, 1))
                make_record(session, f"calibration_p{p+1:02d}_r{repeat:02d}", f"p{p+1:02d}",
                    repeat, "train" if p < 14 else "validation", "calibration", .2, limits,
                    q, np.zeros_like(q), torque, ff, q_true, model_sha, "A" if repeat%2 else "B")
        call_tool(session, "fit", ["--calculator", str(args.calculator.resolve()),
                  "--output", str(session/"fit"), "--bootstrap", "300"])
        fitted = json.loads((session / "fit/fit_report.json").read_text())
        if expected is not None and abs(fitted["alpha"] - expected) > .01:
            raise ValueError(f"Failed known-slope recovery in {case}")
        print(f"IDENTIFIED {case}: alpha={fitted['alpha']:.6f}", flush=True)
        accepted = 0
        for p in range(14, 20):
            for alpha in ALPHAS:
                tag = f"alpha_{alpha:.2f}".replace(".", "_")
                pose_errors, saturation, speeds = [], [], []
                for repeat in range(1, 4):
                    q, dq, tau, ff = simulate_hold(plant, nominal, poses[p], qi, vi, nqi, nvi,
                        kp, kd, alpha, limits, repeat, p, args.hold_seconds)
                    good = make_record(session, f"{tag}_p{p+1:02d}_r{repeat:02d}",
                        f"p{p+1:02d}", repeat, "deployment", tag, alpha, limits,
                        q, dq, tau, ff, poses[p], model_sha)
                    accepted += int(good)
                    pose_errors.append(float(np.sqrt(np.mean((q-poses[p])**2))))
                    saturation.append(float(np.mean(np.abs(ff) >= limits*.999)))
                    speeds.append(float(np.max(np.abs(dq))))
                sweep.append({"scenario": case, "pose": p+1, "alpha": alpha,
                    "hold_rmse_deg": float(np.rad2deg(np.sqrt(np.mean(np.square(pose_errors))))),
                    "ff_saturation_fraction": float(np.mean(saturation)),
                    "max_speed_rad_s": max(speeds)})
            print(f"HOLD {case}: pose {p+1}/20 complete", flush=True)
        call_tool(session, "compare", ["--output", str(session/"compare")])
        comparison = json.loads((session / "compare/hold_comparison.json").read_text())
        best = comparison["candidates_sorted_by_hold_error"][0]
        results.append({"scenario": case, "expected_scalar": expected,
            "fitted_alpha": fitted["alpha"], "ci95": fitted["alpha_pose_bootstrap_95_ci"],
            "per_arm_alpha": fitted["per_arm_alpha_diagnostic"],
            "held_out_model_rmse_nm": fitted["held_out_model_rmse_nm_with_offsets"],
            "best_tested_alpha": best["alpha"], "best_hold_rmse_deg": best["hold_rmse_deg"],
            "accepted_dynamic_records": accepted,
            "rejected_dynamic_records": comparison["rejected_deployment_records"]})
        print(f"DONE {case}: best tested alpha={best['alpha']}, error={best['hold_rmse_deg']:.6g} deg", flush=True)
    with (args.output / "hold_sweep.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(sweep[0]))
        writer.writeheader(); writer.writerows(sweep)
    save(args.output / "results.json", {"simulation_only": True, "mujoco_version": mujoco.__version__,
        "source_urdf_sha256": sha(args.urdf), "calculator_sha256": sha(args.calculator),
        "max_kdl_mujoco_relative_error": max(cross_errors), "gravity_m_s2": GRAVITY,
        "configured_gains": configured, "encoded_gains": wire,
        "control_hz": 100, "physics_hz": 1000, "ramp_seconds": 10,
        "hold_seconds": args.hold_seconds, "candidate_alphas": ALPHAS,
        "wall_seconds": time.monotonic()-started, "cases": results,
        "limitations": ["Known simulated plants; no evidence of the real arm's optimal scale.",
          "No CAN, real actuator/current limits, friction, temperature, contacts or network latency.",
          "Moving finger branches are excluded to match the existing KDL chain.",
          "Static identification uses inverse-dynamics observations with explicitly injected sensor noise.",
          "Hold comparison uses actual forward dynamics, not a formula assigned to the error.",
          "Unclipped scenarios are theoretical controls; they do not authorize changing real torque limits.",
          "The best tested alpha need not be a unique or continuous global optimum."]})
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), constrained_layout=True)
    for ax, result in zip(axes.flat, results):
        for alpha in ALPHAS:
            cells = [r for r in sweep if r["scenario"] == result["scenario"] and r["alpha"] == alpha]
            mean = np.sqrt(np.mean([r["hold_rmse_deg"]**2 for r in cells]))
            ax.scatter(alpha, mean, color="#1769aa")
        means = [np.sqrt(np.mean([r["hold_rmse_deg"]**2 for r in sweep
                 if r["scenario"] == result["scenario"] and r["alpha"] == a])) for a in ALPHAS]
        ax.plot(ALPHAS, means, color="#1769aa")
        ax.axvline(result["fitted_alpha"], color="#e08020", ls="--", label="identified slope")
        ax.set(title=result["scenario"].replace("_", " "), xlabel="gravity scale alpha", ylabel="hold RMSE (deg)")
        ax.grid(alpha=.2); ax.legend(fontsize=8)
    fig.suptitle("SIMULATION ONLY - actual MuJoCo fixed-target hold")
    fig.savefig(args.output / "hold_sweep.png", dpi=180)
    fig.savefig(args.output / "hold_sweep.pdf")
    plt.close(fig)
    table = "\n".join(f"| {r['scenario']} | {r['fitted_alpha']:.5f} | {r['held_out_model_rmse_nm']:.5f} | {r['best_tested_alpha']:.2f} | {r['best_hold_rmse_deg']:.5f} |" for r in results)
    (args.output / "REPORT.md").write_text(
        "# 无实机重力补偿仿真实验\n\n**所有数据均为仿真，不是实机标定结果。**\n\n"
        "每场景采用 20 姿态、每姿态 4 次合成力矩观测，14 姿态拟合、6 姿态留出；"
        "再在 6 个留出位置目标上测试 8 个候选、各 3 次真实 MuJoCo 前向动力学保持。"
        "每次保持 13 秒，含 10 秒前馈 ramp；统计最后 1 秒。PD 增益按当前 URDF 及 CAN 量化/限幅计算，"
        "其中关节 4 配置 kd=40 被编码限制到 5。\n\n"
        "| 仿真场景 | 模型比例估计 | 留出力矩 RMSE (Nm) | 最好被测比例 | 保持 RMSE (deg) |\n"
        "| --- | ---: | ---: | ---: | ---: |\n" + table + "\n\n"
        "![候选比例对照](hold_sweep.png)\n\n"
        "未限幅场景仅为理论对照，不代表应放宽实机限幅。额外右侧负载场景没有唯一已知全局比例；"
        "它用于检查单一比例模型的局限。当前限幅场景需要同时查看饱和比例。\n\n"
        "本实验不含真实电机摩擦、温度、驱动器反馈误差、总电流限制、手指分支和接触；"
        "它验证计算链路和指定模型中的选参方法，不能据此将实机参数直接设为仿真最优值。\n\n"
        "完整数值与运行配置见 `results.json`，逐姿态保持结果见 `hold_sweep.csv`，"
        "每场景的原始模拟数据、拟合区间及对照表保存在相应子目录。\n\n"
        "参考：[MuJoCo 动力学计算文档](https://mujoco.readthedocs.io/en/stable/computation/)。\n")
    print("COMPLETE", args.output, flush=True)


if __name__ == "__main__":
    main()
