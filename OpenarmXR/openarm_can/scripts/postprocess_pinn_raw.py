#!/usr/bin/env python3
"""Post-process OpenArm PINN raw logs into per-joint training rows."""

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(
        description="Convert openarm-pinn-data-logger raw CSV to training-ready CSV."
    )
    parser.add_argument("raw_csv", type=Path)
    parser.add_argument("--output", type=Path, default=Path("openarm_pinn_training.csv"))
    parser.add_argument("--plot-dir", type=Path, default=Path("openarm_pinn_plots"))
    parser.add_argument("--window", type=int, default=11, help="Odd smoothing window length.")
    parser.add_argument("--polyorder", type=int, default=3, help="Savitzky-Golay polynomial order.")
    parser.add_argument("--no-plots", action="store_true")
    return parser.parse_args()


def to_float(value):
    try:
        out = float(value)
    except (TypeError, ValueError):
        return math.nan
    return out if math.isfinite(out) else math.nan


def load_rows(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def smooth_signal(y, window, polyorder):
    y = np.asarray(y, dtype=float)
    if len(y) < 3:
        return y.copy()

    window = max(3, int(window))
    if window % 2 == 0:
        window += 1
    if window > len(y):
        window = len(y) if len(y) % 2 == 1 else len(y) - 1
    if window < 3:
        return y.copy()

    try:
        from scipy.signal import savgol_filter

        order = min(polyorder, window - 1)
        return savgol_filter(y, window_length=window, polyorder=order, mode="interp")
    except Exception:
        kernel = np.ones(window, dtype=float) / float(window)
        pad = window // 2
        padded = np.pad(y, (pad, pad), mode="edge")
        return np.convolve(padded, kernel, mode="valid")


def gradient(signal, time):
    if len(signal) < 2:
        return np.full_like(signal, np.nan, dtype=float)
    return np.gradient(signal, time, edge_order=1)


def write_training_csv(groups, output, window, polyorder):
    fieldnames = [
        "time",
        "chunk",
        "joint_name",
        "motor_id",
        "motor_type",
        "theta",
        "omega",
        "s",
        "ds",
        "dds",
        "tau_feedback",
        "tau_rnea",
        "tau_friction_label",
        "t_mos",
        "t_rotor",
    ]
    with output.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for _, rows in sorted(groups.items()):
            rows.sort(key=lambda r: to_float(r["time"]))
            time = np.array([to_float(r["time"]) for r in rows], dtype=float)
            p_m = np.array([to_float(r["p_m"]) for r in rows], dtype=float)
            xout = np.array([to_float(r["xout"]) for r in rows], dtype=float)
            tau = np.array([to_float(r["feedback_torque"]) for r in rows], dtype=float)

            valid = np.isfinite(time) & np.isfinite(p_m) & np.isfinite(xout) & np.isfinite(tau)
            if np.count_nonzero(valid) < 3:
                continue

            rows_valid = [row for row, keep in zip(rows, valid) if keep]
            time = time[valid]
            p_m = p_m[valid]
            xout = xout[valid]
            tau = tau[valid]

            order = np.argsort(time)
            time = time[order]
            p_m = p_m[order]
            xout = xout[order]
            tau = tau[order]
            rows_valid = [rows_valid[i] for i in order]

            unique = np.concatenate(([True], np.diff(time) > 0.0))
            time = time[unique]
            p_m = p_m[unique]
            xout = xout[unique]
            tau = tau[unique]
            rows_valid = [row for row, keep in zip(rows_valid, unique) if keep]
            if len(time) < 3:
                continue

            theta = smooth_signal(p_m, window, polyorder)
            s = smooth_signal(xout, window, polyorder)
            omega = smooth_signal(gradient(theta, time), window, polyorder)
            ds = smooth_signal(gradient(s, time), window, polyorder)
            dds = smooth_signal(gradient(ds, time), window, polyorder)

            for i, row in enumerate(rows_valid):
                writer.writerow(
                    {
                        "time": f"{time[i]:.12g}",
                        "chunk": row["chunk"],
                        "joint_name": row["joint_name"],
                        "motor_id": row["motor_id"],
                        "motor_type": row["motor_type"],
                        "theta": f"{theta[i]:.12g}",
                        "omega": f"{omega[i]:.12g}",
                        "s": f"{s[i]:.12g}",
                        "ds": f"{ds[i]:.12g}",
                        "dds": f"{dds[i]:.12g}",
                        "tau_feedback": f"{tau[i]:.12g}",
                        "tau_rnea": "nan",
                        "tau_friction_label": "nan",
                        "t_mos": row["t_mos"],
                        "t_rotor": row["t_rotor"],
                    }
                )


def make_plots(groups, plot_dir, window, polyorder):
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        print(f"Skipping plots: matplotlib unavailable ({exc})")
        return

    plot_dir.mkdir(parents=True, exist_ok=True)
    for key, rows in sorted(groups.items()):
        rows.sort(key=lambda r: to_float(r["time"]))
        time = np.array([to_float(r["time"]) for r in rows], dtype=float)
        feedback_pos = np.array([to_float(r["feedback_pos"]) for r in rows], dtype=float)
        p_m = np.array([to_float(r["p_m"]) for r in rows], dtype=float)
        xout = np.array([to_float(r["xout"]) for r in rows], dtype=float)
        gr = np.array([to_float(r["gear_ratio"]) for r in rows], dtype=float)
        valid = np.isfinite(time) & np.isfinite(feedback_pos) & np.isfinite(p_m) & np.isfinite(xout)
        if np.count_nonzero(valid) < 3:
            continue

        time = time[valid]
        feedback_pos = feedback_pos[valid]
        p_m = smooth_signal(p_m[valid], window, polyorder)
        xout = smooth_signal(xout[valid], window, polyorder)
        gr = gr[valid]
        gr_value = np.nanmedian(gr) if np.any(np.isfinite(gr)) else math.nan

        omega = gradient(p_m, time)
        ds = gradient(xout, time)

        fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
        axes[0, 0].plot(time, feedback_pos, label="feedback_pos")
        axes[0, 0].plot(time, p_m, label="p_m")
        axes[0, 0].set_title("feedback_pos vs p_m")
        axes[0, 0].legend()

        axes[0, 1].plot(time, feedback_pos, label="feedback_pos")
        axes[0, 1].plot(time, xout, label="xout")
        axes[0, 1].set_title("feedback_pos vs xout")
        axes[0, 1].legend()

        axes[1, 0].plot(time, p_m, label="p_m")
        if math.isfinite(gr_value):
            axes[1, 0].plot(time, gr_value * xout, label="Gr*xout")
        axes[1, 0].set_title("p_m vs Gr*xout")
        axes[1, 0].legend()

        axes[1, 1].plot(time, omega, label="omega=d(p_m)/dt")
        if math.isfinite(gr_value):
            axes[1, 1].plot(time, gr_value * ds, label="Gr*ds")
        axes[1, 1].set_title("omega vs Gr*ds")
        axes[1, 1].legend()

        safe_key = str(key).replace("/", "_").replace(" ", "_")
        fig.savefig(plot_dir / f"{safe_key}.png", dpi=150)
        plt.close(fig)


def main():
    args = parse_args()
    rows = load_rows(args.raw_csv)
    groups = defaultdict(list)
    for row in rows:
        key = row.get("joint_name") or row.get("motor_id")
        groups[key].append(row)

    write_training_csv(groups, args.output, args.window, args.polyorder)
    if not args.no_plots:
        make_plots(groups, args.plot_dir, args.window, args.polyorder)
    print(f"Wrote {args.output}")
    print("tau_rnea and tau_friction_label are NaN until an RNEA pass is added.")


if __name__ == "__main__":
    main()
