#!/usr/bin/env python3
"""Validate Damiao feedback_pos, p_m, and xout relationships from raw CSV logs."""

import argparse
import csv
import json
import math
import warnings
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


DEFAULT_MIN_VALID_RATIO = 0.8
DEFAULT_CORRELATION_THRESHOLD = 0.98
DEFAULT_R2_THRESHOLD = 0.95
DEFAULT_NORMALIZED_RMSE_THRESHOLD = 0.05
DEFAULT_MIN_RATE_HZ = 10.0
DEFAULT_SMOOTHING_WINDOW = 11
DEFAULT_POLYORDER = 3

SUMMARY_KEYS = [
    "input_csv",
    "num_samples",
    "duration_sec",
    "estimated_rate_hz",
    "feedback_position_interpretation",
    "pm_xout_interpretation",
    "feature_usability",
    "recommended_theta_feature",
    "recommended_s_feature",
    "recommended_extra_features",
    "warnings",
]

METRIC_FIELDS = [
    "domain",
    "variant",
    "comparison",
    "valid_count",
    "rmse",
    "mae",
    "max_abs_error",
    "correlation",
    "linear_fit_slope",
    "linear_fit_intercept",
    "linear_fit_r2",
    "scale_ratio_median",
    "scale_ratio_mean",
    "normalized_rmse",
]

SAFETY_WARNING = """This validation is read-only logging and analysis.
Do not enable motors for this validation unless the operator intentionally decides to do so.
Do not send motion commands.
Do not send torque commands.
Do not write motor parameters.
If manually moving the joint, ensure it is mechanically safe and gravity-supported."""


@dataclass(frozen=True)
class Thresholds:
    min_valid_ratio: float
    correlation: float
    r2: float
    normalized_rmse: float
    min_rate_hz: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyze feedback_pos, p_m, xout, and gear ratio validation metrics."
    )
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--joint-name", default=None)
    parser.add_argument("--unwrap", dest="unwrap", action="store_true", default=True)
    parser.add_argument("--no-unwrap", dest="unwrap", action="store_false")
    parser.add_argument("--min-valid-ratio", type=float, default=DEFAULT_MIN_VALID_RATIO)
    parser.add_argument("--plot", action="store_true")
    parser.add_argument("--report", action="store_true")
    parser.add_argument("--correlation-threshold", type=float, default=DEFAULT_CORRELATION_THRESHOLD)
    parser.add_argument("--r2-threshold", type=float, default=DEFAULT_R2_THRESHOLD)
    parser.add_argument(
        "--normalized-rmse-threshold",
        type=float,
        default=DEFAULT_NORMALIZED_RMSE_THRESHOLD,
    )
    parser.add_argument("--min-rate-hz", type=float, default=DEFAULT_MIN_RATE_HZ)
    parser.add_argument("--smoothing-window", type=int, default=DEFAULT_SMOOTHING_WINDOW)
    parser.add_argument("--polyorder", type=int, default=DEFAULT_POLYORDER)
    return parser.parse_args()


def to_float(value: object) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return math.nan
    return out if math.isfinite(out) else math.nan


def load_rows(path: Path, joint_name: Optional[str]) -> List[Dict[str, str]]:
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    if joint_name is not None:
        rows = [row for row in rows if row.get("joint_name") == joint_name]
    if not rows:
        raise ValueError("no rows available after applying input filters")
    return rows


def find_gear_column(fieldnames: Sequence[str]) -> str:
    if "gear_ratio" in fieldnames:
        return "gear_ratio"
    if "Gr" in fieldnames:
        return "Gr"
    raise ValueError("CSV must include gear_ratio or Gr column")


def require_columns(fieldnames: Sequence[str], gear_column: str) -> None:
    required = ["time", "feedback_pos", "feedback_vel", "p_m", "xout", gear_column]
    missing = [name for name in required if name not in fieldnames]
    if missing:
        raise ValueError("CSV is missing required columns: " + ", ".join(missing))


def original_column_array(rows: Sequence[Dict[str, str]], name: str) -> np.ndarray:
    return np.array([to_float(row.get(name)) for row in rows], dtype=float)


def analysis_column_array(rows: Sequence[Dict[str, str]], name: str, missing_minus_one: bool) -> np.ndarray:
    values = original_column_array(rows, name)
    if missing_minus_one:
        values = values.copy()
        values[values == -1.0] = np.nan
    return values


def validity_stats(values_original: np.ndarray, values_for_analysis: np.ndarray) -> Dict[str, float]:
    finite = np.isfinite(values_for_analysis)
    finite_values = values_for_analysis[finite]
    count = int(np.count_nonzero(finite))
    return {
        "num_samples": int(values_original.size),
        "valid_count": count,
        "valid_ratio": float(count / values_original.size) if values_original.size else 0.0,
        "nan_count": int(np.count_nonzero(~np.isfinite(values_original))),
        "minus_one_count": int(np.count_nonzero(values_original == -1.0)),
        "min": float(np.nanmin(finite_values)) if finite_values.size else math.nan,
        "max": float(np.nanmax(finite_values)) if finite_values.size else math.nan,
        "mean": float(np.nanmean(finite_values)) if finite_values.size else math.nan,
        "std": float(np.nanstd(finite_values)) if finite_values.size else math.nan,
        "range": float(np.nanmax(finite_values) - np.nanmin(finite_values))
        if finite_values.size
        else math.nan,
    }


def sampling_stats(time: np.ndarray) -> Dict[str, float]:
    valid_time = time[np.isfinite(time)]
    if valid_time.size < 2:
        return {
            "duration_sec": 0.0,
            "mean_dt": math.nan,
            "median_dt": math.nan,
            "min_dt": math.nan,
            "max_dt": math.nan,
            "estimated_rate_hz": 0.0,
            "jitter_std": math.nan,
        }
    valid_time = np.sort(valid_time)
    dt = np.diff(valid_time)
    dt = dt[dt > 0.0]
    duration = float(valid_time[-1] - valid_time[0])
    mean_dt = float(np.mean(dt)) if dt.size else math.nan
    return {
        "duration_sec": duration,
        "mean_dt": mean_dt,
        "median_dt": float(np.median(dt)) if dt.size else math.nan,
        "min_dt": float(np.min(dt)) if dt.size else math.nan,
        "max_dt": float(np.max(dt)) if dt.size else math.nan,
        "estimated_rate_hz": float(1.0 / mean_dt)
        if math.isfinite(mean_dt) and mean_dt > 0.0
        else 0.0,
        "jitter_std": float(np.std(dt)) if dt.size else math.nan,
    }


def pair_metrics(name: str, x: np.ndarray, y: np.ndarray) -> Dict[str, float]:
    valid = np.isfinite(x) & np.isfinite(y)
    xv = x[valid]
    yv = y[valid]
    result: Dict[str, float] = {"comparison": name, "valid_count": int(xv.size)}
    if xv.size < 3:
        result.update(
            {
                "rmse": math.nan,
                "mae": math.nan,
                "max_abs_error": math.nan,
                "correlation": math.nan,
                "linear_fit_slope": math.nan,
                "linear_fit_intercept": math.nan,
                "linear_fit_r2": math.nan,
                "scale_ratio_median": math.nan,
                "scale_ratio_mean": math.nan,
                "normalized_rmse": math.nan,
            }
        )
        return result

    err = yv - xv
    rmse = float(np.sqrt(np.mean(err * err)))
    denom_range = float(np.nanmax(yv) - np.nanmin(yv))
    if denom_range <= 0.0:
        denom_range = float(np.nanmax(np.abs(yv)))
    if denom_range <= 0.0:
        denom_range = 1.0
    corr = (
        float(np.corrcoef(xv, yv)[0, 1])
        if np.std(xv) > 0.0 and np.std(yv) > 0.0
        else math.nan
    )
    rank_warning = getattr(np, "RankWarning", None)
    if rank_warning is None and hasattr(np, "exceptions"):
        rank_warning = getattr(np.exceptions, "RankWarning", Warning)
    if rank_warning is None:
        rank_warning = Warning
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", rank_warning)
        slope, intercept = np.polyfit(xv, yv, 1)
    fit = slope * xv + intercept
    ss_res = float(np.sum((yv - fit) ** 2))
    ss_tot = float(np.sum((yv - np.mean(yv)) ** 2))
    ratios = np.divide(yv, xv, out=np.full_like(yv, np.nan), where=np.abs(xv) > 1e-12)
    result.update(
        {
            "rmse": rmse,
            "mae": float(np.mean(np.abs(err))),
            "max_abs_error": float(np.max(np.abs(err))),
            "correlation": corr,
            "linear_fit_slope": float(slope),
            "linear_fit_intercept": float(intercept),
            "linear_fit_r2": float(1.0 - ss_res / ss_tot) if ss_tot > 0.0 else math.nan,
            "scale_ratio_median": float(np.nanmedian(ratios))
            if np.any(np.isfinite(ratios))
            else math.nan,
            "scale_ratio_mean": float(np.nanmean(ratios))
            if np.any(np.isfinite(ratios))
            else math.nan,
            "normalized_rmse": float(rmse / denom_range),
        }
    )
    return result


def normalize_window(window: int, length: int, polyorder: int) -> int:
    window = max(3, int(window))
    if window % 2 == 0:
        window += 1
    if window > length:
        window = length if length % 2 == 1 else length - 1
    if window <= polyorder:
        window = polyorder + 2 if (polyorder + 2) % 2 == 1 else polyorder + 3
    return window if window <= length and window >= 3 else 0


def smooth_signal(y: np.ndarray, window: int, polyorder: int) -> np.ndarray:
    y = np.asarray(y, dtype=float)
    valid = np.isfinite(y)
    if y.size < 3 or np.count_nonzero(valid) < 3:
        return y.copy()
    filled = y.copy()
    if not np.all(valid):
        idx = np.arange(y.size)
        filled[~valid] = np.interp(idx[~valid], idx[valid], y[valid])
    win = normalize_window(window, y.size, polyorder)
    if win == 0:
        return filled
    try:
        from scipy.signal import savgol_filter

        return savgol_filter(filled, window_length=win, polyorder=min(polyorder, win - 1), mode="interp")
    except Exception:
        kernel = np.ones(win, dtype=float) / float(win)
        pad = win // 2
        padded = np.pad(filled, (pad, pad), mode="edge")
        return np.convolve(padded, kernel, mode="valid")


def derivative(signal: np.ndarray, time: np.ndarray, window: int, polyorder: int) -> np.ndarray:
    valid = np.isfinite(signal) & np.isfinite(time)
    out = np.full_like(signal, np.nan, dtype=float)
    if np.count_nonzero(valid) < 3:
        return out
    idx = np.where(valid)[0]
    tv = time[valid]
    sv = smooth_signal(signal[valid], window, polyorder)
    order = np.argsort(tv)
    grad = np.gradient(sv[order], tv[order], edge_order=1)
    restored = np.empty_like(grad)
    restored[order] = grad
    out[idx] = smooth_signal(restored, window, polyorder)
    return out


def unwrap_angle(values: np.ndarray) -> np.ndarray:
    out = values.copy()
    valid = np.isfinite(out)
    if np.count_nonzero(valid) >= 2:
        out[valid] = np.unwrap(out[valid])
    return out


def safe_divide(numerator: np.ndarray, denominator: np.ndarray) -> np.ndarray:
    return np.divide(
        numerator,
        denominator,
        out=np.full_like(numerator, np.nan, dtype=float),
        where=np.isfinite(denominator) & (np.abs(denominator) > 1e-12),
    )


def metric_passes(metric: Dict[str, float], thresholds: Thresholds) -> bool:
    corr = abs(metric.get("correlation", math.nan))
    r2 = metric.get("linear_fit_r2", math.nan)
    nrmse = metric.get("normalized_rmse", math.nan)
    return (
        math.isfinite(corr)
        and corr >= thresholds.correlation
        and math.isfinite(r2)
        and r2 >= thresholds.r2
        and math.isfinite(nrmse)
        and nrmse <= thresholds.normalized_rmse
    )


def choose_feedback_interpretation(metrics: Dict[str, Dict[str, float]], thresholds: Thresholds) -> str:
    candidates = [
        ("feedback_vs_pm", "feedback_pos_is_motor_side"),
        ("feedback_vs_xout", "feedback_pos_is_output_side"),
        ("feedback_vs_pm_over_gr", "feedback_pos_is_motor_side_divided_by_gear_ratio"),
        ("feedback_vs_gr_xout", "feedback_pos_is_output_side_scaled_by_gear_ratio"),
    ]
    passing: List[Tuple[float, int, str]] = []
    for priority, (key, label) in enumerate(candidates):
        metric = metrics.get(key)
        if metric and metric_passes(metric, thresholds):
            passing.append((metric.get("normalized_rmse", math.inf), priority, label))
    return sorted(passing)[0][2] if passing else "inconclusive"


def choose_pm_xout_interpretation(
    metrics: Dict[str, Dict[str, float]],
    column_stats: Dict[str, Dict[str, float]],
    sample: Dict[str, float],
    thresholds: Thresholds,
) -> str:
    pm_ok = column_stats["p_m"]["valid_ratio"] >= thresholds.min_valid_ratio
    xout_ok = column_stats["xout"]["valid_ratio"] >= thresholds.min_valid_ratio
    gr_ok = column_stats["Gr"]["valid_ratio"] >= thresholds.min_valid_ratio
    if not (pm_ok and xout_ok and gr_ok):
        return "pm_xout_missing_or_unstable"
    pair_ok = any(
        metric_passes(metrics[key], thresholds)
        for key in ("pm_vs_gr_xout", "pm_over_gr_vs_xout")
        if key in metrics
    )
    if not pair_ok:
        return "pm_xout_inconclusive"
    if sample["estimated_rate_hz"] < thresholds.min_rate_hz:
        return "pm_xout_valid_but_low_rate"
    return "pm_xout_valid_motor_output_pair"


def choose_feature_usability(feedback_label: str, pm_label: str) -> str:
    if pm_label == "pm_xout_valid_motor_output_pair":
        return "CAN_EXTENDED_FEATURES_USABLE_FOR_PINN"
    if pm_label == "pm_xout_valid_but_low_rate":
        return "CAN_EXTENDED_FEATURES_DIAGNOSTIC_ONLY"
    if pm_label == "pm_xout_missing_or_unstable":
        return "CAN_EXTENDED_FEATURES_NOT_USABLE"
    if feedback_label == "inconclusive":
        return "INCONCLUSIVE_NEED_MORE_DATA"
    return "INCONCLUSIVE_NEED_MORE_DATA"


def json_safe(value: object) -> object:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, list):
        return [json_safe(v) for v in value]
    return value


def fmt(value: object) -> str:
    if isinstance(value, float):
        return "nan" if not math.isfinite(value) else f"{value:.6g}"
    return str(value)


def write_metrics_csv(path: Path, metric_rows: Sequence[Dict[str, object]]) -> None:
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=METRIC_FIELDS)
        writer.writeheader()
        for row in metric_rows:
            writer.writerow({field: json_safe(row.get(field, math.nan)) for field in METRIC_FIELDS})


def write_summary_json(path: Path, summary: Dict[str, object]) -> None:
    ordered = {key: summary[key] for key in SUMMARY_KEYS}
    with path.open("w") as f:
        json.dump(json_safe(ordered), f, indent=2, sort_keys=False)
        f.write("\n")


def report_table(headers: Sequence[str], rows: Iterable[Sequence[object]]) -> str:
    out = ["| " + " | ".join(headers) + " |", "| " + " | ".join(["---"] * len(headers)) + " |"]
    for row in rows:
        out.append("| " + " | ".join(fmt(item) for item in row) + " |")
    return "\n".join(out)


def hardware_validation_commands() -> str:
    return """### Single ID 1 logging

```bash
colcon build --packages-select openarm_can
source install/setup.bash

openarm-pinn-data-logger \\
  --interface can0 \\
  --ids 1 \\
  --types DM8009 \\
  --rate 20 \\
  --duration 30 \\
  --output /tmp/openarm_id1_raw.csv
```

### Single ID 1 analysis

```bash
python3 scripts/analyze_pm_xout_validation.py \\
  --input /tmp/openarm_id1_raw.csv \\
  --output-dir /tmp/openarm_id1_pm_xout_validation \\
  --unwrap \\
  --plot \\
  --report
```

### Full V10 arm logging

```bash
openarm-pinn-data-logger \\
  --interface can0 \\
  --ids 1,2,3,4,5,6,7 \\
  --types DM8009,DM8009,DM4340,DM4340,DM4310,DM4310,DM4310 \\
  --rate 20 \\
  --duration 30 \\
  --output /tmp/openarm_arm_raw.csv
```

### Full V10 arm analysis

```bash
python3 scripts/analyze_pm_xout_validation.py \\
  --input /tmp/openarm_arm_raw.csv \\
  --output-dir /tmp/openarm_arm_pm_xout_validation \\
  --unwrap \\
  --plot \\
  --report
```

### Passive logging

```bash
openarm-pinn-data-logger \\
  --interface can0 \\
  --ids 1 \\
  --types DM8009 \\
  --rate 20 \\
  --duration 30 \\
  --passive \\
  --output /tmp/openarm_id1_raw_passive.csv
```

Passive mode may not refresh parameter values reliably, so missing `p_m` and `xout`
values do not by themselves prove the parameters are unavailable.
"""


def write_report(
    path: Path,
    summary: Dict[str, object],
    sample: Dict[str, float],
    column_stats: Dict[str, Dict[str, float]],
    metric_rows: Sequence[Dict[str, object]],
    warnings: Sequence[str],
) -> None:
    position_rows = [row for row in metric_rows if row["domain"] == "position"]
    velocity_rows = [row for row in metric_rows if row["domain"] == "velocity"]
    column_rows = [
        (
            name,
            stats["num_samples"],
            stats["valid_count"],
            stats["valid_ratio"],
            stats["nan_count"],
            stats["minus_one_count"],
            stats["min"],
            stats["max"],
            stats["mean"],
            stats["std"],
            stats["range"],
        )
        for name, stats in column_stats.items()
    ]
    metric_table_headers = [
        "variant",
        "comparison",
        "valid",
        "rmse",
        "mae",
        "corr",
        "slope",
        "intercept",
        "r2",
        "norm_rmse",
    ]
    content = [
        "# PM/XOUT Validation Report",
        "",
        "## Summary",
        "",
        report_table(
            ["key", "value"],
            [(key, summary[key]) for key in SUMMARY_KEYS if key in summary],
        ),
        "",
        "## Safety Warning",
        "",
        "```text",
        SAFETY_WARNING,
        "```",
        "",
        "## Hardware Validation Commands",
        "",
        hardware_validation_commands(),
        "",
        "## Sampling",
        "",
        report_table(["metric", "value"], sample.items()),
        "",
        "## Column Validity",
        "",
        report_table(
            [
                "column",
                "num",
                "valid",
                "valid_ratio",
                "nan",
                "minus_one",
                "min",
                "max",
                "mean",
                "std",
                "range",
            ],
            column_rows,
        ),
        "",
        "## Position Metrics",
        "",
        report_table(
            metric_table_headers,
            [
                (
                    row["variant"],
                    row["comparison"],
                    row["valid_count"],
                    row["rmse"],
                    row["mae"],
                    row["correlation"],
                    row["linear_fit_slope"],
                    row["linear_fit_intercept"],
                    row["linear_fit_r2"],
                    row["normalized_rmse"],
                )
                for row in position_rows
            ],
        ),
        "",
        "## Velocity Metrics",
        "",
        report_table(
            metric_table_headers,
            [
                (
                    row["variant"],
                    row["comparison"],
                    row["valid_count"],
                    row["rmse"],
                    row["mae"],
                    row["correlation"],
                    row["linear_fit_slope"],
                    row["linear_fit_intercept"],
                    row["linear_fit_r2"],
                    row["normalized_rmse"],
                )
                for row in velocity_rows
            ],
        ),
        "",
        "## Interpretation",
        "",
        f"- feedback_position_interpretation: `{summary['feedback_position_interpretation']}`",
        f"- pm_xout_interpretation: `{summary['pm_xout_interpretation']}`",
        f"- feature_usability: `{summary['feature_usability']}`",
        f"- recommended_theta_feature: `{summary['recommended_theta_feature']}`",
        f"- recommended_s_feature: `{summary['recommended_s_feature']}`",
        "",
    ]
    if warnings:
        content.extend(["## Warnings", ""])
        content.extend(f"- {warning}" for warning in warnings)
        content.append("")
    path.write_text("\n".join(content))


def add_metric_row(
    rows: List[Dict[str, object]], domain: str, variant: str, metric: Dict[str, float]
) -> None:
    row: Dict[str, object] = {"domain": domain, "variant": variant}
    row.update(metric)
    rows.append(row)


def compute_position_metrics(
    variant: str,
    feedback_pos: np.ndarray,
    p_m: np.ndarray,
    xout: np.ndarray,
    gr: np.ndarray,
) -> Dict[str, Dict[str, float]]:
    pm_over_gr = safe_divide(p_m, gr)
    gr_xout = gr * xout
    return {
        "feedback_vs_pm": pair_metrics("feedback_vs_pm", p_m, feedback_pos),
        "feedback_vs_xout": pair_metrics("feedback_vs_xout", xout, feedback_pos),
        "feedback_vs_pm_over_gr": pair_metrics("feedback_vs_pm_over_gr", pm_over_gr, feedback_pos),
        "feedback_vs_gr_xout": pair_metrics("feedback_vs_gr_xout", gr_xout, feedback_pos),
        "pm_vs_gr_xout": pair_metrics("pm_vs_gr_xout", gr_xout, p_m),
        "pm_over_gr_vs_xout": pair_metrics("pm_over_gr_vs_xout", xout, pm_over_gr),
    }


def plot_outputs(
    output_dir: Path,
    time: np.ndarray,
    feedback_pos: np.ndarray,
    feedback_vel: np.ndarray,
    p_m: np.ndarray,
    xout: np.ndarray,
    gr: np.ndarray,
    d_feedback: np.ndarray,
    d_pm: np.ndarray,
    d_xout: np.ndarray,
    original: Dict[str, np.ndarray],
) -> Optional[str]:
    try:
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", message="Unable to import Axes3D.*")
            import matplotlib.pyplot as plt
    except Exception as exc:
        return f"matplotlib unavailable; skipped plots: {exc}"

    pm_over_gr = safe_divide(p_m, gr)
    gr_xout = gr * xout

    def save_scatter(name: str, x: np.ndarray, y: np.ndarray, xlabel: str, ylabel: str) -> None:
        fig, ax = plt.subplots(figsize=(7, 5), constrained_layout=True)
        ax.scatter(x, y, s=10, alpha=0.7)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        fig.savefig(output_dir / name, dpi=150)
        plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 5), constrained_layout=True)
    ax.plot(time, feedback_pos, label="feedback_pos")
    ax.plot(time, p_m, label="p_m")
    ax.plot(time, xout, label="xout")
    ax.plot(time, pm_over_gr, label="p_m / Gr")
    ax.plot(time, gr_xout, label="Gr * xout")
    ax.set_xlabel("time")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.savefig(output_dir / "time_feedback_pm_xout.png", dpi=150)
    plt.close(fig)

    save_scatter("feedback_vs_pm.png", p_m, feedback_pos, "p_m", "feedback_pos")
    save_scatter("feedback_vs_xout.png", xout, feedback_pos, "xout", "feedback_pos")
    save_scatter("feedback_vs_pm_over_gr.png", pm_over_gr, feedback_pos, "p_m / Gr", "feedback_pos")
    save_scatter("pm_vs_gr_xout.png", gr_xout, p_m, "Gr * xout", "p_m")
    save_scatter("pm_over_gr_vs_xout.png", xout, pm_over_gr, "xout", "p_m / Gr")

    fig, ax = plt.subplots(figsize=(10, 5), constrained_layout=True)
    ax.plot(time, feedback_vel, label="feedback_vel")
    ax.plot(time, d_feedback, label="d(feedback_pos)/dt")
    ax.plot(time, d_pm, label="d(p_m)/dt")
    ax.plot(time, safe_divide(d_pm, gr), label="d(p_m)/dt / Gr")
    ax.plot(time, d_xout, label="d(xout)/dt")
    ax.set_xlabel("time")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.savefig(output_dir / "velocity_consistency.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 4), constrained_layout=True)
    for idx, name in enumerate(("feedback_pos", "feedback_vel", "p_m", "xout", "Gr")):
        vals = original[name]
        missing = (~np.isfinite(vals)) | (vals == -1.0 if name in ("p_m", "xout", "Gr") else False)
        ax.scatter(time[missing], np.full(np.count_nonzero(missing), idx), s=12, label=name)
    ax.set_yticks(range(5), ["feedback_pos", "feedback_vel", "p_m", "xout", "Gr"])
    ax.set_xlabel("time")
    ax.set_title("Missing values timeline")
    ax.grid(True, axis="x", alpha=0.3)
    fig.savefig(output_dir / "missing_values_timeline.png", dpi=150)
    plt.close(fig)
    return None


def build_warnings(
    column_stats: Dict[str, Dict[str, float]],
    sample: Dict[str, float],
    thresholds: Thresholds,
) -> List[str]:
    warnings: List[str] = []
    for name in ("p_m", "xout", "Gr"):
        stats = column_stats[name]
        if stats["minus_one_count"] > 0:
            warnings.append(f"{name} contains {stats['minus_one_count']} -1 missing-sentinel candidates.")
        if stats["valid_ratio"] < thresholds.min_valid_ratio:
            warnings.append(
                f"{name} valid ratio {stats['valid_ratio']:.3g} is below min_valid_ratio "
                f"{thresholds.min_valid_ratio:.3g}."
            )
    if sample["estimated_rate_hz"] < thresholds.min_rate_hz:
        warnings.append(
            f"estimated_rate_hz {sample['estimated_rate_hz']:.3g} is below min_rate_hz "
            f"{thresholds.min_rate_hz:.3g}; high-rate PINN features may be unsuitable."
        )
    return warnings


def run_analysis(args: argparse.Namespace) -> None:
    rows = load_rows(args.input, args.joint_name)
    fieldnames = list(rows[0].keys())
    gear_column = find_gear_column(fieldnames)
    require_columns(fieldnames, gear_column)
    thresholds = Thresholds(
        min_valid_ratio=args.min_valid_ratio,
        correlation=args.correlation_threshold,
        r2=args.r2_threshold,
        normalized_rmse=args.normalized_rmse_threshold,
        min_rate_hz=args.min_rate_hz,
    )

    time = original_column_array(rows, "time")
    feedback_pos_original = original_column_array(rows, "feedback_pos")
    feedback_vel_original = original_column_array(rows, "feedback_vel")
    p_m_original = original_column_array(rows, "p_m")
    xout_original = original_column_array(rows, "xout")
    gr_original = original_column_array(rows, gear_column)

    feedback_pos = analysis_column_array(rows, "feedback_pos", False)
    feedback_vel = analysis_column_array(rows, "feedback_vel", False)
    p_m = analysis_column_array(rows, "p_m", True)
    xout = analysis_column_array(rows, "xout", True)
    gr = analysis_column_array(rows, gear_column, True)

    order = np.argsort(np.where(np.isfinite(time), time, np.inf))
    time = time[order]
    feedback_pos_original = feedback_pos_original[order]
    feedback_vel_original = feedback_vel_original[order]
    p_m_original = p_m_original[order]
    xout_original = xout_original[order]
    gr_original = gr_original[order]
    feedback_pos = feedback_pos[order]
    feedback_vel = feedback_vel[order]
    p_m = p_m[order]
    xout = xout[order]
    gr = gr[order]

    column_stats = {
        "time": validity_stats(time, time),
        "feedback_pos": validity_stats(feedback_pos_original, feedback_pos),
        "feedback_vel": validity_stats(feedback_vel_original, feedback_vel),
        "p_m": validity_stats(p_m_original, p_m),
        "xout": validity_stats(xout_original, xout),
        "Gr": validity_stats(gr_original, gr),
    }
    sample = sampling_stats(time)

    metric_rows: List[Dict[str, object]] = []
    raw_position = compute_position_metrics("raw", feedback_pos, p_m, xout, gr)
    for metric in raw_position.values():
        add_metric_row(metric_rows, "position", "raw", metric)

    if args.unwrap:
        feedback_for_positions = unwrap_angle(feedback_pos)
        pm_for_positions = unwrap_angle(p_m)
        xout_for_positions = unwrap_angle(xout)
        interp_position = compute_position_metrics(
            "unwrapped", feedback_for_positions, pm_for_positions, xout_for_positions, gr
        )
        for metric in interp_position.values():
            add_metric_row(metric_rows, "position", "unwrapped", metric)
    else:
        feedback_for_positions = feedback_pos
        pm_for_positions = p_m
        xout_for_positions = xout
        interp_position = raw_position

    d_feedback = derivative(feedback_for_positions, time, args.smoothing_window, args.polyorder)
    d_pm = derivative(pm_for_positions, time, args.smoothing_window, args.polyorder)
    d_xout = derivative(xout_for_positions, time, args.smoothing_window, args.polyorder)
    velocity_metrics = {
        "feedback_vel_vs_d_feedback_pos": pair_metrics(
            "feedback_vel_vs_d_feedback_pos", d_feedback, feedback_vel
        ),
        "feedback_vel_vs_dxout": pair_metrics("feedback_vel_vs_dxout", d_xout, feedback_vel),
        "feedback_vel_vs_dpm": pair_metrics("feedback_vel_vs_dpm", d_pm, feedback_vel),
        "feedback_vel_vs_dpm_over_gr": pair_metrics(
            "feedback_vel_vs_dpm_over_gr", safe_divide(d_pm, gr), feedback_vel
        ),
    }
    for metric in velocity_metrics.values():
        add_metric_row(metric_rows, "velocity", "unwrapped" if args.unwrap else "raw", metric)

    feedback_label = choose_feedback_interpretation(interp_position, thresholds)
    pm_label = choose_pm_xout_interpretation(interp_position, column_stats, sample, thresholds)
    usability = choose_feature_usability(feedback_label, pm_label)
    warnings = build_warnings(column_stats, sample, thresholds)

    if pm_label in ("pm_xout_valid_motor_output_pair", "pm_xout_valid_but_low_rate"):
        theta = "p_m"
        s_feature = "xout"
        extras = ["p_m / Gr - xout", "p_m - Gr * xout"]
    else:
        theta = ""
        s_feature = ""
        extras = []

    summary: Dict[str, object] = {
        "input_csv": str(args.input),
        "num_samples": int(len(rows)),
        "duration_sec": sample["duration_sec"],
        "estimated_rate_hz": sample["estimated_rate_hz"],
        "feedback_position_interpretation": feedback_label,
        "pm_xout_interpretation": pm_label,
        "feature_usability": usability,
        "recommended_theta_feature": theta,
        "recommended_s_feature": s_feature,
        "recommended_extra_features": extras,
        "warnings": warnings,
    }

    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_summary_json(args.output_dir / "pm_xout_validation_summary.json", summary)
    write_metrics_csv(args.output_dir / "pm_xout_validation_metrics.csv", metric_rows)
    if args.report:
        write_report(
            args.output_dir / "pm_xout_validation_report.md",
            summary,
            sample,
            column_stats,
            metric_rows,
            warnings,
        )
    else:
        (args.output_dir / "pm_xout_validation_report.md").write_text(
            "# PM/XOUT Validation Report\n\nRun with `--report` for the full report.\n"
        )

    if args.plot:
        warning = plot_outputs(
            args.output_dir,
            time,
            feedback_for_positions,
            feedback_vel,
            pm_for_positions,
            xout_for_positions,
            gr,
            d_feedback,
            d_pm,
            d_xout,
            {
                "feedback_pos": feedback_pos_original,
                "feedback_vel": feedback_vel_original,
                "p_m": p_m_original,
                "xout": xout_original,
                "Gr": gr_original,
            },
        )
        if warning:
            warnings.append(warning)
            summary["warnings"] = warnings
            write_summary_json(args.output_dir / "pm_xout_validation_summary.json", summary)
            if args.report:
                write_report(
                    args.output_dir / "pm_xout_validation_report.md",
                    summary,
                    sample,
                    column_stats,
                    metric_rows,
                    warnings,
                )

    print(f"feedback_position_interpretation={feedback_label}")
    print(f"pm_xout_interpretation={pm_label}")
    print(f"feature_usability={usability}")
    print(f"summary={args.output_dir / 'pm_xout_validation_summary.json'}")


def main() -> None:
    args = parse_args()
    run_analysis(args)


if __name__ == "__main__":
    main()
