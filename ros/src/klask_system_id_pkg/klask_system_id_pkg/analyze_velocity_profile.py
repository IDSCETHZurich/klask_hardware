"""Analyze velocity_profile_node bag files from klask robot system identification.

Loads ROS2 bag files recorded by the velocity_profile_node across several
trajectory patterns (lines, circles) and multiple commanded speeds per pattern,
produces per-bag diagnostic figures (reusing the acceleration-test plots),
and adds pattern-aware aggregates: speed tracking, all-speeds XY overlay per
pattern, error-vs-speed curves, and a global cross-pattern summary.
"""

import argparse
import os
import re
import sys
from collections import defaultdict

import matplotlib

import matplotlib.pyplot as plt
import numpy as np

from klask_system_id_pkg.bag_analysis_common import (
    extract_noise_rms_summary,
    make_time_relative,
    plot_motor_commands,
    plot_noise_estimation,
    plot_odrive_current,
    plot_odrive_system,
    plot_odrive_temperature,
    plot_odrive_torque,
    plot_position,
    plot_velocity_tracking,
    read_bag,
)

matplotlib.use("Agg")


BAG_NAME_RE = re.compile(r"^vel_profile_(?P<pattern>.+)_(?P<ts>\d{8}_\d{6})_run(?P<run>\d+)$")


def discover_vel_profile_bags(bag_dir, patterns=None, runs=None, splits=None):
    """Find velocity-profile bag directories.

    Returns list of dicts with keys: path, pattern, run, ts, split.
    Scans <bag_dir>/train/ and <bag_dir>/validation/ if either exists; otherwise
    falls back to a flat scan of <bag_dir> (legacy layout, tagged as "train").
    Filters by pattern name, run index, and split if provided.
    """
    search_dirs = []
    for split_name in ("train", "validation"):
        split_path = os.path.join(bag_dir, split_name)
        if os.path.isdir(split_path):
            search_dirs.append((split_name, split_path))
    if not search_dirs:
        search_dirs = [("train", bag_dir)]

    bags = []
    for split_name, search_dir in search_dirs:
        if splits is not None and split_name not in splits:
            continue
        for entry in sorted(os.listdir(search_dir)):
            full_path = os.path.join(search_dir, entry)
            if not os.path.isdir(full_path):
                continue
            m = BAG_NAME_RE.match(entry)
            if not m:
                continue
            pattern = m.group("pattern")
            run = int(m.group("run"))
            ts = m.group("ts")
            if patterns is not None and pattern not in patterns:
                continue
            if runs is not None and run not in runs:
                continue
            bags.append(
                {
                    "path": full_path,
                    "pattern": pattern,
                    "run": run,
                    "ts": ts,
                    "split": split_name,
                }
            )
    bags.sort(key=lambda b: (b["split"], b["pattern"], b["run"]))
    return bags


def derive_nominal_speed(data, threshold=0.01):
    """Return the scalar commanded speed for a run, derived from /cmd_vel.

    Uses median(|cmd_vel|) over samples where |cmd_vel| > threshold, which
    excludes settle/homing/idle portions.
    """
    cv = data["cmd_vel"]
    if len(cv["t"]) == 0:
        return float("nan")
    mag = np.hypot(cv["vx"], cv["vy"])
    moving = mag > threshold
    if not np.any(moving):
        return float("nan")
    return float(np.median(mag[moving]))


# ---------------------------------------------------------------------------
# Per-bag plots (velocity-profile specific)
# ---------------------------------------------------------------------------


def plot_speed_tracking(data, pattern_name, run_index, nominal_speed):
    """Figure: scalar speed |v|(t) commanded vs measured, with nominal reference."""
    fig, ax = plt.subplots(1, 1, figsize=(14, 5))
    fig.suptitle(
        f"Speed Tracking  —  {pattern_name} run{run_index}  —  v ≈ {nominal_speed:.2f} m/s",
        fontsize=14,
    )

    cv = data["cmd_vel"]
    bs = data["board_state"]
    if len(cv["t"]) > 0:
        cmd_mag = np.hypot(cv["vx"], cv["vy"])
        ax.plot(cv["t"], cmd_mag, label="|Commanded|", color="C0", linewidth=1, alpha=0.85)
    if len(bs["t"]) > 0:
        meas_mag = np.hypot(bs["vx"], bs["vy"])
        ax.plot(bs["t"], meas_mag, label="|Measured|", color="C1", linewidth=1, alpha=0.85)

    ax.axhline(nominal_speed, linestyle="--", color="C3", alpha=0.7, label=f"Nominal {nominal_speed:.2f} m/s")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Speed |v| (m/s)")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# Per-pattern aggregate plots
# ---------------------------------------------------------------------------


def plot_xy_all_speeds(pattern_name, pattern_runs):
    """Overlay all runs' actual XY trajectories for one pattern, colored by speed."""
    fig, ax = plt.subplots(1, 1, figsize=(9, 9))
    fig.suptitle(f"XY Trajectories — {pattern_name}", fontsize=14)

    speeds = np.array([r["speed"] for r in pattern_runs])
    if len(speeds) == 0:
        ax.set_title("no runs")
        return fig

    vmin, vmax = float(np.nanmin(speeds)), float(np.nanmax(speeds))
    if vmin == vmax:
        vmax = vmin + 1e-6
    cmap = plt.get_cmap("viridis")
    norm = plt.Normalize(vmin=vmin, vmax=vmax)

    for r in pattern_runs:
        bs = r["data"]["board_state"]
        if len(bs["t"]) == 0:
            continue
        ax.plot(bs["px"], bs["py"], color=cmap(norm(r["speed"])), linewidth=0.8, alpha=0.8)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    sm = plt.cm.ScalarMappable(norm=norm, cmap=cmap)
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label("Commanded speed (m/s)")

    fig.tight_layout()
    return fig


def plot_error_vs_speed(pattern_name, pattern_runs):
    """Velocity-tracking RMS (x and y) vs commanded speed, one point per run."""
    fig, ax = plt.subplots(1, 1, figsize=(10, 6))
    fig.suptitle(f"Velocity Tracking Error vs Speed — {pattern_name}", fontsize=14)

    pts = sorted(
        [(r["speed"], r["vel_err_rms_x"], r["vel_err_rms_y"]) for r in pattern_runs],
        key=lambda p: p[0],
    )
    if not pts:
        ax.set_title("no runs")
        return fig
    speeds, rms_x, rms_y = zip(*pts)
    ax.plot(speeds, rms_x, "o-", color="C0", label="RMS(vx − cmd_vx)")
    ax.plot(speeds, rms_y, "s-", color="C1", label="RMS(vy − cmd_vy)")
    ax.set_xlabel("Commanded speed (m/s)")
    ax.set_ylabel("Velocity error RMS (m/s)")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_noise_vs_speed(pattern_name, pattern_runs):
    """Per-pattern: state-estimator noise RMS vs commanded speed (HPF-based)."""
    fig, (ax_pos, ax_vel) = plt.subplots(2, 1, sharex=True, figsize=(10, 9))
    fig.suptitle(f"State Estimator Noise vs Speed — {pattern_name}", fontsize=14)

    def collect(key):
        pts = sorted(
            [(r["speed"], r[key]) for r in pattern_runs if np.isfinite(r.get(key, np.nan))],
            key=lambda p: p[0],
        )
        return zip(*pts) if pts else ((), ())

    s_px, n_px = collect("noise_rms_position_x")
    s_py, n_py = collect("noise_rms_position_y")
    s_vx, n_vx = collect("noise_rms_velocity_x")
    s_vy, n_vy = collect("noise_rms_velocity_y")

    if s_px:
        ax_pos.plot(s_px, n_px, "o-", color="C0", label="Position X")
    if s_py:
        ax_pos.plot(s_py, n_py, "s-", color="C1", label="Position Y")
    ax_pos.set_ylabel("Position noise RMS (m)")
    ax_pos.legend()
    ax_pos.grid(True, alpha=0.3)

    if s_vx:
        ax_vel.plot(s_vx, n_vx, "o-", color="C0", label="Velocity X")
    if s_vy:
        ax_vel.plot(s_vy, n_vy, "s-", color="C1", label="Velocity Y")
    ax_vel.set_xlabel("Commanded speed (m/s)")
    ax_vel.set_ylabel("Velocity noise RMS (m/s)")
    ax_vel.legend()
    ax_vel.grid(True, alpha=0.3)

    fig.tight_layout()
    return fig


def plot_summary_all_patterns(per_pattern_runs):
    """Global figure overlaying total-velocity-error RMS vs speed for all patterns."""
    fig, ax = plt.subplots(1, 1, figsize=(11, 6))
    fig.suptitle("Velocity Tracking Error vs Speed — All Patterns", fontsize=14)

    for pattern_name, pattern_runs in sorted(per_pattern_runs.items()):
        pts = sorted(
            [
                (r["speed"], np.hypot(r["vel_err_rms_x"], r["vel_err_rms_y"]))
                for r in pattern_runs
                if np.isfinite(r["vel_err_rms_x"]) and np.isfinite(r["vel_err_rms_y"])
            ],
            key=lambda p: p[0],
        )
        if not pts:
            continue
        speeds, rms_total = zip(*pts)
        ax.plot(speeds, rms_total, "o-", label=pattern_name, linewidth=1.2)

    ax.set_xlabel("Commanded speed (m/s)")
    ax.set_ylabel("Velocity error RMS  √(RMS_x² + RMS_y²)  (m/s)")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_noise_summary_all_patterns(per_pattern_runs):
    """Cross-pattern: combined noise RMS  √(rms_x² + rms_y²)  vs speed.

    Two stacked subplots — position (m) and velocity (m/s) — one line per pattern.
    """
    fig, (ax_pos, ax_vel) = plt.subplots(2, 1, sharex=True, figsize=(11, 9))
    fig.suptitle("State Estimator Noise vs Speed — All Patterns", fontsize=14)

    def collect(runs, kx, ky):
        pts = sorted(
            [
                (r["speed"], np.hypot(r[kx], r[ky]))
                for r in runs
                if np.isfinite(r.get(kx, np.nan)) and np.isfinite(r.get(ky, np.nan))
            ],
            key=lambda p: p[0],
        )
        return zip(*pts) if pts else ((), ())

    for pattern_name, pattern_runs in sorted(per_pattern_runs.items()):
        s_p, n_p = collect(pattern_runs, "noise_rms_position_x", "noise_rms_position_y")
        if s_p:
            ax_pos.plot(s_p, n_p, "o-", label=pattern_name, linewidth=1.2)
        s_v, n_v = collect(pattern_runs, "noise_rms_velocity_x", "noise_rms_velocity_y")
        if s_v:
            ax_vel.plot(s_v, n_v, "o-", label=pattern_name, linewidth=1.2)

    ax_pos.set_ylabel("Position noise RMS  √(x² + y²)  (m)")
    ax_pos.legend()
    ax_pos.grid(True, alpha=0.3)

    ax_vel.set_xlabel("Commanded speed (m/s)")
    ax_vel.set_ylabel("Velocity noise RMS  √(x² + y²)  (m/s)")
    ax_vel.legend()
    ax_vel.grid(True, alpha=0.3)

    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# Per-bag summary metrics (subset of the acceleration-test version)
# ---------------------------------------------------------------------------


def compute_run_metrics(data):
    """Velocity-tracking RMS for a single run."""
    bs = data["board_state"]
    cv = data["cmd_vel"]
    if len(bs["t"]) == 0 or len(cv["t"]) == 0:
        return float("nan"), float("nan")
    cmd_vx_interp = np.interp(bs["t"], cv["t"], cv["vx"])
    cmd_vy_interp = np.interp(bs["t"], cv["t"], cv["vy"])
    rms_x = float(np.sqrt(np.mean((cmd_vx_interp - bs["vx"]) ** 2)))
    rms_y = float(np.sqrt(np.mean((cmd_vy_interp - bs["vy"]) ** 2)))
    return rms_x, rms_y


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def _save(fig, path, dpi):
    fig.savefig(path, dpi=dpi, bbox_inches="tight")
    print(f"  Saved {path}")
    plt.close(fig)


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description="Analyze Klask velocity-profile bag files")
    parser.add_argument(
        "--bag-dir",
        type=str,
        default=os.path.join(os.path.dirname(__file__), "..", "..", "..", "vel_profile_bags"),
        help="Directory containing velocity-profile bag subdirectories",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="vel_profile_analysis_output",
        help="Output directory for plots",
    )
    parser.add_argument(
        "--patterns",
        type=str,
        nargs="*",
        default=None,
        help="Pattern names to analyze (default: all discovered)",
    )
    parser.add_argument(
        "--runs",
        type=int,
        nargs="*",
        default=None,
        help="Run indices to analyze (default: all)",
    )
    parser.add_argument(
        "--splits",
        type=str,
        nargs="*",
        default=None,
        choices=["train", "validation"],
        help="Splits to analyze (default: all discovered)",
    )
    parser.add_argument("--dpi", type=int, default=150)
    parser.add_argument("--format", type=str, default="png", choices=["png", "pdf", "svg"])
    args = parser.parse_args()

    bag_dir = os.path.abspath(args.bag_dir)
    output_dir = os.path.abspath(args.output_dir)
    os.makedirs(output_dir, exist_ok=True)

    patterns_filter = set(args.patterns) if args.patterns else None
    runs_filter = set(args.runs) if args.runs else None
    splits_filter = set(args.splits) if args.splits else None

    bags = discover_vel_profile_bags(bag_dir, patterns_filter, runs_filter, splits_filter)
    if not bags:
        print(f"No velocity-profile bags found in {bag_dir}")
        sys.exit(1)

    print(f"Found {len(bags)} bag(s) in {bag_dir}")

    per_pattern_runs = defaultdict(list)

    for bag in bags:
        print(
            f"\nProcessing [{bag['split']}] {bag['pattern']} run{bag['run']}  " f"({os.path.basename(bag['path'])}) ..."
        )

        data = read_bag(bag["path"])
        data = make_time_relative(data)
        speed = derive_nominal_speed(data)

        pattern_dir = os.path.join(output_dir, bag["split"], bag["pattern"])
        os.makedirs(pattern_dir, exist_ok=True)

        prefix = f"run{bag['run']}_v{speed:.2f}"
        fmt = args.format

        _save(
            plot_speed_tracking(data, bag["pattern"], bag["run"], speed),
            os.path.join(pattern_dir, f"{prefix}_01_speed_tracking.{fmt}"),
            args.dpi,
        )
        _save(
            plot_velocity_tracking(data, speed),
            os.path.join(pattern_dir, f"{prefix}_02_velocity_tracking.{fmt}"),
            args.dpi,
        )
        _save(
            plot_position(data, speed),
            os.path.join(pattern_dir, f"{prefix}_03_position.{fmt}"),
            args.dpi,
        )
        _save(
            plot_odrive_system(data, speed),
            os.path.join(pattern_dir, f"{prefix}_04_odrive_system.{fmt}"),
            args.dpi,
        )
        _save(
            plot_odrive_torque(data, speed),
            os.path.join(pattern_dir, f"{prefix}_05_odrive_torque.{fmt}"),
            args.dpi,
        )
        _save(
            plot_odrive_current(data, speed),
            os.path.join(pattern_dir, f"{prefix}_06_odrive_current.{fmt}"),
            args.dpi,
        )
        _save(
            plot_odrive_temperature(data, speed),
            os.path.join(pattern_dir, f"{prefix}_07_odrive_temperature.{fmt}"),
            args.dpi,
        )
        _save(
            plot_motor_commands(data, speed),
            os.path.join(pattern_dir, f"{prefix}_08_motor_commands.{fmt}"),
            args.dpi,
        )

        noise_fig, noise_metrics = plot_noise_estimation(data, speed)
        if noise_fig is not None:
            _save(
                noise_fig,
                os.path.join(pattern_dir, f"{prefix}_09_noise_estimation.{fmt}"),
                args.dpi,
            )
        else:
            noise_metrics = {}

        for sig_name in ["Position X", "Position Y", "Velocity X", "Velocity Y"]:
            if sig_name in noise_metrics:
                m = noise_metrics[sig_name]
                print(f"    {sig_name} noise: RMS={m['rms']:.6f}, std={m['std']:.6f}")

        rms_x, rms_y = compute_run_metrics(data)
        per_pattern_runs[(bag["split"], bag["pattern"])].append(
            {
                "run": bag["run"],
                "speed": speed,
                "data": data,
                "vel_err_rms_x": rms_x,
                "vel_err_rms_y": rms_y,
                **extract_noise_rms_summary(noise_metrics),
            }
        )

    print("\nGenerating per-pattern aggregates ...")
    per_split_patterns = defaultdict(dict)
    for (split_name, pattern_name), runs in per_pattern_runs.items():
        pattern_dir = os.path.join(output_dir, split_name, pattern_name)
        _save(
            plot_xy_all_speeds(pattern_name, runs),
            os.path.join(pattern_dir, f"_pattern_xy_all_speeds.{args.format}"),
            args.dpi,
        )
        _save(
            plot_error_vs_speed(pattern_name, runs),
            os.path.join(pattern_dir, f"_pattern_error_vs_speed.{args.format}"),
            args.dpi,
        )
        _save(
            plot_noise_vs_speed(pattern_name, runs),
            os.path.join(pattern_dir, f"_pattern_noise_vs_speed.{args.format}"),
            args.dpi,
        )
        per_split_patterns[split_name][pattern_name] = runs

    for split_name, pattern_runs in per_split_patterns.items():
        if len(pattern_runs) > 1:
            print(f"\nGenerating cross-pattern summary for [{split_name}] ...")
            _save(
                plot_summary_all_patterns(pattern_runs),
                os.path.join(
                    output_dir,
                    split_name,
                    f"_summary_all_patterns_error_vs_speed.{args.format}",
                ),
                args.dpi,
            )
            _save(
                plot_noise_summary_all_patterns(pattern_runs),
                os.path.join(
                    output_dir,
                    split_name,
                    f"_summary_all_patterns_noise_vs_speed.{args.format}",
                ),
                args.dpi,
            )

    print(f"\nDone. {len(bags)} bag(s) analyzed. Plots saved to {output_dir}/")


if __name__ == "__main__":
    main()
