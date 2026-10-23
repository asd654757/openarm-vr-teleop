#!/usr/bin/env python3
"""Read-only ROS static recording and offline gravity-scale identification.

Never publishes robot commands or changes ROS parameters. The fitted slope is
a model estimate, not an automatically approved controller setting.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

SIDES = ("left", "right")
PARAMS = ("robot_description", "enabled", "apply_effort", "enable_left",
          "enable_right", "gravity_scale", "ramp_duration_s", "torque_limits",
          "state_timeout_s", "publish_rate_hz")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2,
                                    allow_nan=False) + "\n")


def ros_node():
    import rclpy
    from rclpy.node import Node
    rclpy.init(args=[])
    return rclpy, Node("gravity_calibration_reader")


def read_parameters(rclpy, node, target):
    from rcl_interfaces.srv import GetParameters
    from rclpy.parameter import parameter_value_to_python
    client = node.create_client(GetParameters, target.rstrip("/") + "/get_parameters")
    try:
        if not client.wait_for_service(timeout_sec=5):
            raise ValueError(f"Parameter service unavailable: {target}")
        request = GetParameters.Request()
        request.names = list(PARAMS)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if not future.done() or future.result() is None:
            raise ValueError("Timed out reading gravity parameters")
        return {name: parameter_value_to_python(value)
                for name, value in zip(PARAMS, future.result().values)}
    finally:
        node.destroy_client(client)


def snapshot(args):
    rclpy, node = ros_node()
    try:
        params = read_parameters(rclpy, node, args.node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    urdf = params.pop("robot_description")
    if not urdf:
        raise ValueError("The live gravity node has no robot_description")
    controls, warnings = [], []
    for control in ET.fromstring(urdf).findall("ros2_control"):
        hardware = control.find("hardware")
        if hardware is None:
            continue
        gains = {p.get("name"): p.text for p in hardware.findall("param")}
        plugin = hardware.findtext("plugin", "")
        controls.append({"name": control.get("name"), "plugin": plugin, "parameters": gains})
        for name, raw in gains.items():
            if re.fullmatch(r"k[dp][1-7]", name):
                limit = 5.0 if name.startswith("kd") else 500.0
                if not 0 <= float(raw) <= limit:
                    warnings.append(f"{control.get('name')} {name}={raw}: current CAN encoder clamps to [0,{limit}]. Verify the running driver.")
        if "mock" in plugin.lower() or "fake" in plugin.lower():
            warnings.append("Fake hardware detected: this session cannot establish a physical calibration.")
    args.session.mkdir(parents=True, exist_ok=True)
    if (args.session / "session.json").exists():
        raise ValueError("Session already exists; use a new directory")
    (args.session / "robot.urdf").write_text(urdf)
    value = {"schema": 1, "created_unix": time.time(), "gravity_node": args.node,
             "payload": args.payload, "torque_measurement_note": args.torque_note,
             "urdf_sha256": digest(args.session / "robot.urdf"),
             "gravity_parameters": params, "hardware": controls, "warnings": warnings}
    write_json(args.session / "session.json", value)
    print(json.dumps(value, ensure_ascii=False, indent=2))


def record(args):
    import numpy as np
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray
    from rclpy.qos import qos_profile_sensor_data
    session = json.loads((args.session / "session.json").read_text())
    if digest(args.session / "robot.urdf") != session["urdf_sha256"]:
        raise ValueError("Session URDF changed")
    for label in (args.pose, args.tag):
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]*", label):
            raise ValueError("Pose/tag must use letters, numbers, underscores or hyphens")
    sides = SIDES if args.arm == "both" else (args.arm,)
    names = [f"openarm_{side}_joint{j}" for side in sides for j in range(1, 8)]
    output = args.session / "recordings" / f"{args.tag}_{args.pose}_r{args.repeat:02d}"
    output.mkdir(parents=True, exist_ok=False)
    rclpy, node = ros_node()
    state = {"capture": False, "rows": [], "error": None, "ff": {}, "qd": {}, "start": 0.0}
    if args.target_record:
        seed = json.loads((args.target_record / "record.json").read_text())
        if (seed["pose"] != args.pose or not seed["accepted"]
                or seed["urdf_sha256"] != session["urdf_sha256"]):
            raise ValueError("Target record must be an accepted capture of this pose")
        for side in sides:
            targets = [seed["summary"].get(f"openarm_{side}_joint{j}", {}).get("qd")
                       for j in range(1, 8)]
            if all(v is not None for v in targets):
                state["qd"][side] = targets

    def receive_ff(msg, side):
        if len(msg.data) == 7 and all(math.isfinite(x) for x in msg.data):
            state["ff"][side] = (time.monotonic(), list(msg.data))

    def receive_command(msg, side):
        if len(msg.data) >= 7 and all(math.isfinite(x) for x in msg.data[:7]):
            state["qd"][side] = list(msg.data[:7])

    def receive(msg):
        if not state["capture"]:
            return
        indices = {name: i for i, name in enumerate(msg.name)}
        if not all(name in indices for name in names):
            state["error"] = "Missing arm joints on /joint_states"
            return
        now = time.monotonic()
        for name in names:
            index = indices[name]
            if any(index >= len(values) for values in (msg.position, msg.velocity, msg.effort)):
                state["error"] = "joint_states must include position, velocity AND effort"
                return
        frame = []
        for name in names:
            i = indices[name]
            values = [float(msg.position[i]), float(msg.velocity[i]), float(msg.effort[i])]
            if not all(math.isfinite(v) for v in values):
                state["error"] = "Non-finite joint state"
                return
            side = "left" if "_left_" in name else "right"
            ff = state["ff"].get(side)
            applied = ff[1][int(name[-1]) - 1] if ff and now - ff[0] < 0.2 else ""
            target = state["qd"].get(side)
            frame.append([now - state["start"], msg.header.stamp.sec,
                          msg.header.stamp.nanosec, name, *values, applied,
                          target[int(name[-1]) - 1] if target else ""])
        state["rows"].extend(frame)

    node.create_subscription(JointState, args.topic, receive, qos_profile_sensor_data)
    for side in sides:
        node.create_subscription(Float64MultiArray, f"/gravity_compensation/{side}_effort",
                                 lambda msg, side=side: receive_ff(msg, side), 10)
        node.create_subscription(Float64MultiArray, f"/{side}_forward_position_controller/commands",
                                 lambda msg, side=side: receive_command(msg, side), 10)
    try:
        before = read_parameters(rclpy, node, session["gravity_node"])
        if hashlib.sha256(before.pop("robot_description").encode()).hexdigest() != session["urdf_sha256"]:
            raise ValueError("Live URDF differs from session snapshot")
        print(f"Settle for {args.settle:g}s; no external support/contact during measurement.", flush=True)
        end = time.monotonic() + args.settle
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        state["start"] = time.monotonic()
        state["capture"] = True
        print(f"Recording {args.seconds:g}s. Robot commands are NOT published by this tool.", flush=True)
        while time.monotonic() - state["start"] < args.seconds and not state["error"]:
            rclpy.spin_once(node, timeout_sec=0.02)
        state["capture"] = False
        after = read_parameters(rclpy, node, session["gravity_node"])
        after_urdf = after.pop("robot_description")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    reasons = [state["error"]] if state["error"] else []
    if before != after:
        reasons.append("Gravity parameters changed during recording")
    if hashlib.sha256(after_urdf.encode()).hexdigest() != session["urdf_sha256"]:
        reasons.append("Live URDF changed during recording")
    summary = {}
    for name in names:
        rows = [r for r in state["rows"] if r[3] == name]
        if len(rows) < max(30, args.seconds * 20):
            reasons.append(f"{name}: too few samples ({len(rows)})")
            continue
        a = np.array([r[4:7] for r in rows])
        span = float(np.ptp(a[:, 0]))
        speed = float(np.max(np.abs(a[:, 1])))
        if span > math.radians(args.max_span_deg) or speed > args.max_speed:
            reasons.append(f"{name}: not static (q span={math.degrees(span):.3f}deg, max speed={speed:.4f}rad/s)")
        ffs = [float(r[7]) for r in rows if r[7] != ""]
        if len(ffs) < 0.9 * len(rows):
            reasons.append(f"{name}: gravity preview missing/stale")
        targets = [float(r[8]) for r in rows if r[8] != ""]
        if args.split == "deployment" and (len(targets) != len(rows) or np.ptp(targets) > math.radians(0.1)):
            reasons.append(f"{name}: missing or moving hold target; seed from an earlier capture")
        summary[name] = {"n": len(rows), "q": float(a[:, 0].mean()),
                         "tau": float(a[:, 2].mean()), "tau_std": float(a[:, 2].std(ddof=1)),
                         "q_span_rad": span, "max_speed_rad_s": speed,
                         "ff_mean": float(np.mean(ffs)) if ffs else None,
                         "qd": float(np.mean(targets)) if targets else None,
                         "q_std_rad": float(a[:, 0].std(ddof=1)),
                         "tracking_rmse_rad": float(np.sqrt(np.mean((a[:, 0] - np.array(targets))**2)))
                         if len(targets) == len(rows) else None}
    with (output / "raw.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["elapsed_s", "stamp_sec", "stamp_nanosec", "joint", "q_rad",
                         "dq_rad_s", "tau_feedback_nm", "ff_preview_nm", "q_des_rad"])
        writer.writerows(state["rows"])
    metadata = {"pose": args.pose, "repeat": args.repeat, "split": args.split,
                "tag": args.tag, "approach": args.approach, "note": args.note,
                "created_unix": time.time(), "urdf_sha256": session["urdf_sha256"],
                "target_seed": str(args.target_record) if args.target_record else None,
                "gravity_parameters": before, "accepted": not reasons,
                "reasons": reasons, "summary": summary, "raw_sha256": digest(output / "raw.csv")}
    write_json(output / "record.json", metadata)
    print("ACCEPTED" if not reasons else "REJECTED", output)
    for reason in reasons:
        print(" -", reason)
    return 0 if not reasons else 2


def solve(rows, eligible=None):
    import numpy as np
    joints = sorted({r["joint"] for r in rows}) if eligible is None else eligible
    numerator = denominator = 0.0
    means = {}
    for joint in joints:
        r = [r for r in rows if r["joint"] == joint]
        if len(r) < 2:
            continue
        g, t, w = (np.array([x[key] for x in r]) for key in ("g", "tau", "weight"))
        gm, tm = np.average(g, weights=w), np.average(t, weights=w)
        if eligible is None and np.sqrt(np.average((g - gm)**2, weights=w)) < 0.05:
            continue
        numerator += float(np.sum(w * (g - gm) * (t - tm)))
        denominator += float(np.sum(w * (g - gm)**2))
        means[joint] = (float(gm), float(tm))
    if denominator <= 1e-10:
        raise ValueError("Insufficient gravity excitation; cannot identify a slope")
    alpha = numerator / denominator
    return alpha, {joint: tm - alpha * gm for joint, (gm, tm) in means.items()}


def fit(args):
    import numpy as np
    session = json.loads((args.session / "session.json").read_text())
    urdf = args.session / "robot.urdf"
    if digest(urdf) != session["urdf_sha256"]:
        raise ValueError("Session URDF changed")
    args.output.mkdir(parents=True, exist_ok=False)
    rows, rejected, splits, configs = [], [], {}, set()
    for path in sorted((args.session / "recordings").glob("*/record.json")):
        meta = json.loads(path.read_text())
        if meta["split"] == "deployment":
            continue
        if not meta["accepted"]:
            rejected.append(str(path))
            continue
        if meta["urdf_sha256"] != session["urdf_sha256"] or digest(path.parent / "raw.csv") != meta["raw_sha256"]:
            raise ValueError(f"Changed model or raw data: {path}")
        prior = splits.setdefault(meta["pose"], meta["split"])
        if prior != meta["split"]:
            raise ValueError("The same pose appears in both train and validation")
        configs.add(json.dumps(meta["gravity_parameters"], sort_keys=True))
        for side in SIDES:
            names = [f"openarm_{side}_joint{j}" for j in range(1, 8)]
            if not all(name in meta["summary"] for name in names):
                continue
            q = [meta["summary"][name]["q"] for name in names]
            result = subprocess.run([str(args.calculator.resolve()), str(urdf.resolve()), side,
                                     *(f"{v:.17g}" for v in q)], text=True, capture_output=True, check=True)
            g = np.fromstring(result.stdout.strip(), sep=",")
            if len(g) != 7 or not np.isfinite(g).all():
                raise ValueError("Invalid output from the offline KDL calculator")
            limits = meta["gravity_parameters"]["torque_limits"]
            for j, name in enumerate(names):
                s = meta["summary"][name]
                rows.append({"record": path.parent.name, "pose": meta["pose"],
                             "split": meta["split"], "approach": meta["approach"],
                             "joint": name, "g": float(g[j]), "tau": s["tau"],
                             "weight": 1 / max(s["tau_std"], args.noise_floor)**2,
                             "limit": limits[j], "ff_mean": s["ff_mean"]})
    if len(configs) != 1:
        raise ValueError("Use one fixed gravity configuration for training and validation")
    train = [r for r in rows if r["split"] == "train"]
    validation = [r for r in rows if r["split"] == "validation"]
    if len({r["pose"] for r in train}) < 8 or len({r["pose"] for r in validation}) < 4:
        raise ValueError("Need at least 8 distinct training poses and 4 held-out poses")
    alpha, offsets = solve(train)
    available = [r for r in validation if r["joint"] in offsets]
    residuals = [r["tau"] - alpha*r["g"] - offsets[r["joint"]] for r in available]
    if not residuals:
        raise ValueError("No identifiable joints in validation")
    rng = np.random.default_rng(20261023)
    poses = sorted({r["pose"] for r in train})
    draws = []
    for _ in range(args.bootstrap):
        boot = [r for pose in rng.choice(poses, len(poses), replace=True)
                for r in train if r["pose"] == pose]
        try:
            a, _ = solve(boot, sorted(offsets))
            draws.append(a)
        except ValueError:
            pass
    if len(draws) < 0.9 * args.bootstrap:
        raise ValueError("Bootstrap is rank deficient; collect more varied poses")
    per_arm = {}
    for side in SIDES:
        arm_rows = [r for r in train if f"_{side}_" in r["joint"]]
        try:
            a, _ = solve(arm_rows)
            per_arm[side] = a
        except ValueError:
            per_arm[side] = None
    per_approach = {}
    for approach in sorted({r["approach"] for r in train}):
        try:
            a, _ = solve([r for r in train if r["approach"] == approach])
            per_approach[approach] = a
        except ValueError:
            per_approach[approach] = None
    predictions = {}
    for name, candidate in [("fitted", alpha), ("previous_0_5", 0.5)]:
        res = [r["tau"] - float(np.clip(candidate*r["g"], -r["limit"], r["limit"]))
               for r in available]
        predictions[name] = {"alpha": candidate,
                             "predicted_residual_rmse_nm": float(np.sqrt(np.mean(np.square(res)))),
                             "predicted_saturation_fraction": float(np.mean([abs(candidate*r["g"]) >= r["limit"] for r in available]))}
    report = {"status": "offline_identification_only_not_a_controller_recommendation",
              "alpha": alpha, "alpha_pose_bootstrap_95_ci": np.quantile(draws, [0.025, 0.975]).tolist(),
              "per_arm_alpha_diagnostic": per_arm, "offsets_nm": offsets,
              "per_approach_alpha_diagnostic": per_approach,
              "held_out_model_rmse_nm_with_offsets": float(np.sqrt(np.mean(np.square(residuals)))),
              "deployment_predictions_no_offset": predictions,
              "train_poses": sorted({r["pose"] for r in train}),
              "validation_poses": sorted({r["pose"] for r in validation}),
              "excluded_gravity_insensitive_joints": sorted({r["joint"] for r in train} - set(offsets)),
              "rejected_recordings": rejected, "session_warnings": session["warnings"],
              "torque_measurement_note": session["torque_measurement_note"],
              "calculator_sha256": digest(args.calculator),
              "limitations": ["Static friction is not automatically removed.",
                              "Driver torque calibration and unknown contacts can bias the estimate.",
                              "Confidence interval does not include systematic model/sensor errors.",
                              "Bias offsets are fitted for diagnosis; current controller does not apply them.",
                              "Different-pose holdouts are mandatory; nearby poses may still be correlated."]}
    write_json(args.output / "fit_report.json", report)
    with (args.output / "pose_summary.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(json.dumps(report, ensure_ascii=False, indent=2))


def compare(args):
    """Compare measured static hold performance under matched targets."""
    import numpy as np
    session = json.loads((args.session / "session.json").read_text())
    groups, targets, models, conditions, rejected = {}, {}, set(), set(), []
    for path in sorted((args.session / "recordings").glob("*/record.json")):
        meta = json.loads(path.read_text())
        if meta["split"] != "deployment":
            continue
        if not meta["accepted"]:
            rejected.append({"record": path.parent.name, "reasons": meta.get("reasons", [])})
            continue
        if meta["urdf_sha256"] != session["urdf_sha256"]:
            raise ValueError("Deployment model differs from session model")
        if digest(path.parent / "raw.csv") != meta["raw_sha256"]:
            raise ValueError(f"Raw data changed: {path}")
        models.add(meta["urdf_sha256"])
        params = dict(meta["gravity_parameters"])
        alpha = params.pop("gravity_scale")
        conditions.add(json.dumps(params, sort_keys=True))
        group = groups.setdefault(meta["tag"], {"alpha": alpha, "poses": {}})
        if group["alpha"] != alpha:
            raise ValueError("Use a separate tag for each tested gravity_scale")
        for joint, s in meta["summary"].items():
            if s.get("tracking_rmse_rad") is None or s.get("qd") is None:
                raise ValueError("Deployment comparison requires recorded fixed position targets")
            key = (meta["pose"], joint)
            old = targets.setdefault(key, s["qd"])
            if abs(old - s["qd"]) > math.radians(0.1):
                raise ValueError("Different position targets across candidates; repeat matched trials")
        group["poses"].setdefault(meta["pose"], []).append(meta["summary"])
    if len(groups) < 2 or len(models) != 1 or len(conditions) != 1:
        raise ValueError("Need >=2 candidates with the same model and non-scale settings")
    sets = [set(group["poses"]) for group in groups.values()]
    if any(poses != sets[0] for poses in sets) or len(sets[0]) < 4:
        raise ValueError("Every candidate needs the same >=4 validation poses")
    rows = []
    reference_joints = None
    for tag, group in groups.items():
        per_pose, jitter, spans, counts = [], [], [], []
        for pose, repeats in group["poses"].items():
            counts.append(len(repeats))
            if len(repeats) < 3:
                raise ValueError("Each candidate/pose needs >=3 accepted repeats")
            for repeat in repeats:
                joints = set(repeat)
                reference_joints = joints if reference_joints is None else reference_joints
                if joints != reference_joints:
                    raise ValueError("Do not mix single-arm and bimanual captures")
            per_pose.append(float(np.mean([s["tracking_rmse_rad"]**2
                                           for rep in repeats for s in rep.values()])))
            jitter.extend(s["q_std_rad"] for rep in repeats for s in rep.values())
            spans.extend(s["q_span_rad"] for rep in repeats for s in rep.values())
        rows.append({"tag": tag, "alpha": group["alpha"], "poses": len(per_pose),
                     "min_repeats_per_pose": min(counts),
                     "hold_rmse_deg": math.degrees(math.sqrt(float(np.mean(per_pose)))),
                     "max_joint_std_deg": math.degrees(max(jitter)),
                     "max_joint_span_deg": math.degrees(max(spans))})
    rows.sort(key=lambda r: r["hold_rmse_deg"])
    args.output.mkdir(parents=True, exist_ok=False)
    with (args.output / "hold_comparison.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    write_json(args.output / "hold_comparison.json", {
        "status": "measured_static_comparison_requires_jitter_thermal_and_task_review",
        "candidates_sorted_by_hold_error": rows,
        "rejected_deployment_records": rejected,
        "limitations": ["Only accepted static windows are compared; separately document aborted/unstable candidates.",
                        "Temperature and dynamic performance are not measured by this tool.",
                        "Do not claim a unique optimum when differences are comparable to repeatability noise."]})
    print(json.dumps(rows, ensure_ascii=False, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    s = sub.add_parser("snapshot", help="Read the live gravity model/parameters; no writes to ROS")
    s.add_argument("--session", type=Path, required=True)
    s.add_argument("--node", default="/openarm_gravity_comp")
    s.add_argument("--payload", required=True)
    s.add_argument("--torque-note", required=True)
    s.set_defaults(func=snapshot)
    r = sub.add_parser("record", help="Read-only recording of a manually established static pose")
    r.add_argument("--session", type=Path, required=True)
    r.add_argument("--pose", required=True)
    r.add_argument("--repeat", type=int, required=True)
    r.add_argument("--split", choices=["train", "validation", "deployment"], required=True)
    r.add_argument("--arm", choices=["left", "right", "both"], default="both")
    r.add_argument("--tag", default="calibration")
    r.add_argument("--approach", required=True)
    r.add_argument("--note", default="")
    r.add_argument("--target-record", type=Path,
                   help="Reuse a previously observed hold target only if no position command has changed")
    r.add_argument("--topic", default="/joint_states")
    r.add_argument("--seconds", type=float, default=5)
    r.add_argument("--settle", type=float, default=5)
    r.add_argument("--max-speed", type=float, default=0.03, help="rad/s; initial threshold, inspect sensor noise")
    r.add_argument("--max-span-deg", type=float, default=0.5)
    r.set_defaults(func=record)
    f = sub.add_parser("fit", help="Offline model identification; never changes controller settings")
    f.add_argument("--session", type=Path, required=True)
    f.add_argument("--calculator", type=Path, required=True)
    f.add_argument("--output", type=Path, required=True)
    f.add_argument("--noise-floor", type=float, default=0.05, help="Nm; weighting floor, not an accuracy claim")
    f.add_argument("--bootstrap", type=int, default=300)
    f.set_defaults(func=fit)
    c = sub.add_parser("compare", help="Offline comparison of measured fixed-target hold errors")
    c.add_argument("--session", type=Path, required=True)
    c.add_argument("--output", type=Path, required=True)
    c.set_defaults(func=compare)
    args = parser.parse_args()
    for name in ("seconds", "max_speed", "max_span_deg", "noise_floor"):
        if hasattr(args, name) and (not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0):
            parser.error(f"{name} must be positive and finite")
    if hasattr(args, "settle") and (not math.isfinite(args.settle) or args.settle < 0):
        parser.error("settle must be nonnegative and finite")
    if hasattr(args, "repeat") and args.repeat < 1:
        parser.error("repeat must be positive")
    if hasattr(args, "bootstrap") and args.bootstrap < 100:
        parser.error("bootstrap must be at least 100")
    try:
        return args.func(args) or 0
    except (ValueError, OSError, subprocess.SubprocessError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
