"""Analyze acceleration test bag files from klask robot system identification.

Loads ROS2 bag files recorded during triangle-pattern acceleration tests,
plots velocity tracking, ODrive telemetry, and estimates state estimator
noise via high-pass filtering.
"""

import argparse
import os
import re
import sys

import matplotlib

import matplotlib.pyplot as plt
import numpy as np

from klask_system_id.bag_analysis_common import (
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


def discover_bags(bag_dir, velocity_filter=None):
    """Find bag directories and extract velocity from name.

    Returns list of (bag_path, velocity) sorted by velocity.
    """
    bags = []
    pattern = re.compile(r"_v(\d+\.\d+)$")
    for entry in sorted(os.listdir(bag_dir)):
        full_path = os.path.join(bag_dir, entry)
        if not os.path.isdir(full_path):
            continue
        match = pattern.search(entry)
        if not match:
            continue
        vel = float(match.group(1))
        if velocity_filter is not None and vel not in velocity_filter:
            continue
        bags.append((full_path, vel))
    bags.sort(key=lambda x: x[1])
    return bags


# ---------------------------------------------------------------------------
# Summary across all velocities
# ---------------------------------------------------------------------------


def compute_summary_metrics(data, velocity, noise_metrics):
    """Compute summary statistics for a single bag."""
    summary = {"velocity": velocity}

    bs = data["board_state"]
    cv = data["cmd_vel"]
    if len(bs["t"]) > 0 and len(cv["t"]) > 0:
        cmd_vx_interp = np.interp(bs["t"], cv["t"], cv["vx"])
        cmd_vy_interp = np.interp(bs["t"], cv["t"], cv["vy"])
        summary["vel_err_rms_x"] = np.sqrt(np.mean((cmd_vx_interp - bs["vx"]) ** 2))
        summary["vel_err_rms_y"] = np.sqrt(np.mean((cmd_vy_interp - bs["vy"]) ** 2))
    else:
        summary["vel_err_rms_x"] = np.nan
        summary["vel_err_rms_y"] = np.nan

    for sig_name in ["Position X", "Position Y", "Velocity X", "Velocity Y"]:
        key = f"noise_rms_{sig_name.lower().replace(' ', '_')}"
        if sig_name in noise_metrics:
            summary[key] = noise_metrics[sig_name]["rms"]
        else:
            summary[key] = np.nan

    for axis_key in ["odrive2_ctrl", "odrive3_ctrl"]:
        d = data[axis_key]
        axis_label = axis_key.replace("_ctrl", "")
        if len(d["t"]) > 0:
            summary[f"{axis_label}_mean_torque"] = np.mean(np.abs(d["torque_estimate"]))
        else:
            summary[f"{axis_label}_mean_torque"] = np.nan

    for axis_key in ["odrive2_status", "odrive3_status"]:
        d = data[axis_key]
        axis_label = axis_key.replace("_status", "")
        if len(d["t"]) > 0:
            summary[f"{axis_label}_mean_bus_current"] = np.mean(d["bus_current"])
            valid_fet = d["fet_temperature"][np.isfinite(d["fet_temperature"])]
            summary[f"{axis_label}_max_fet_temp"] = np.max(valid_fet) if len(valid_fet) > 0 else np.nan
        else:
            summary[f"{axis_label}_mean_bus_current"] = np.nan
            summary[f"{axis_label}_max_fet_temp"] = np.nan

    return summary


def plot_summary(all_summaries, output_dir, fmt, dpi):
    """Create summary comparison plots across all velocities."""
    vels = np.array([s["velocity"] for s in all_summaries])

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle("Acceleration Test Summary — All Velocities", fontsize=14)

    ax = axes[0, 0]
    ax.plot(vels, [s["vel_err_rms_x"] for s in all_summaries], "o-", label="X", color="C0")
    ax.plot(vels, [s["vel_err_rms_y"] for s in all_summaries], "s-", label="Y", color="C1")
    ax.set_xlabel("Commanded Velocity (m/s)")
    ax.set_ylabel("Velocity Error RMS (m/s)")
    ax.set_title("Velocity Tracking Error")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.plot(vels, [s["noise_rms_position_x"] for s in all_summaries], "o-", label="X", color="C0")
    ax.plot(vels, [s["noise_rms_position_y"] for s in all_summaries], "s-", label="Y", color="C1")
    ax.set_xlabel("Commanded Velocity (m/s)")
    ax.set_ylabel("Position Noise RMS (m)")
    ax.set_title("Position Noise (HPF)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[0, 2]
    ax.plot(vels, [s["noise_rms_velocity_x"] for s in all_summaries], "o-", label="X", color="C0")
    ax.plot(vels, [s["noise_rms_velocity_y"] for s in all_summaries], "s-", label="Y", color="C1")
    ax.set_xlabel("Commanded Velocity (m/s)")
    ax.set_ylabel("Velocity Noise RMS (m/s)")
    ax.set_title("Velocity Noise (HPF)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[1, 0]
    ax.plot(vels, [s["odrive2_mean_bus_current"] for s in all_summaries], "o-", label="Axis 2", color="C0")
    ax.plot(vels, [s["odrive3_mean_bus_current"] for s in all_summaries], "s-", label="Axis 3", color="C1")
    ax.set_xlabel("Commanded Velocity (m/s)")
    ax.set_ylabel("Mean Bus Current (A)")
    ax.set_title("Bus Current")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    ax.plot(vels, [s["odrive2_mean_torque"] for s in all_summaries], "o-", label="Axis 2", color="C0")
    ax.plot(vels, [s["odrive3_mean_torque"] for s in all_summaries], "s-", label="Axis 3", color="C1")
    ax.set_xlabel("Commanded Velocity (m/s)")
    ax.set_ylabel("Mean |Torque| (Nm)")
    ax.set_title("Motor Torque")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[1, 2]
    ax.plot(vels, [s["odrive2_max_fet_temp"] for s in all_summaries], "o-", label="Axis 2", color="C0")
    ax.plot(vels, [s["odrive3_max_fet_temp"] for s in all_summaries], "s-", label="Axis 3", color="C1")
    ax.set_xlabel("Commanded Velocity (m/s)")
    ax.set_ylabel("Max FET Temp (C)")
    ax.set_title("FET Temperature")
    ax.legend()
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    out_path = os.path.join(output_dir, f"summary_comparison.{fmt}")
    fig.savefig(out_path, dpi=dpi, bbox_inches="tight")
    print(f"  Saved {out_path}")
    plt.close(fig)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description="Analyze Klask acceleration test bag files")
    parser.add_argument(
        "--bag-dir",
        type=str,
        default=os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "acceleration_test_bags"),
        help="Directory containing acceleration test bag subdirectories",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="accel_analysis_output",
        help="Output directory for plots",
    )
    parser.add_argument(
        "--velocities",
        type=float,
        nargs="*",
        default=None,
        help="Specific velocities to analyze (default: all)",
    )
    parser.add_argument("--dpi", type=int, default=150)
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Save only, do not display plots (default when using Agg backend)",
    )
    parser.add_argument("--format", type=str, default="png", choices=["png", "pdf", "svg"])
    args = parser.parse_args()

    bag_dir = os.path.abspath(args.bag_dir)
    output_dir = os.path.abspath(args.output_dir)
    os.makedirs(output_dir, exist_ok=True)

    bags = discover_bags(bag_dir, args.velocities)
    if not bags:
        print(f"No bag files found in {bag_dir}")
        sys.exit(1)

    print(f"Found {len(bags)} bag(s) in {bag_dir}")
    all_summaries = []

    for bag_path, velocity in bags:
        print(f"\nProcessing v={velocity:.2f} m/s  ({os.path.basename(bag_path)}) ...")

        data = read_bag(bag_path)
        data = make_time_relative(data)

        prefix = f"v{velocity:.2f}"

        fig = plot_velocity_tracking(data, velocity)
        path = os.path.join(output_dir, f"{prefix}_01_velocity_tracking.{args.format}")
        fig.savefig(path, dpi=args.dpi, bbox_inches="tight")
        print(f"  Saved {path}")
        plt.close(fig)

        fig = plot_position(data, velocity)
        path = os.path.join(output_dir, f"{prefix}_02_position.{args.format}")
        fig.savefig(path, dpi=args.dpi, bbox_inches="tight")
        print(f"  Saved {path}")
        plt.close(fig)

        fig = plot_odrive_system(data, velocity)
        path = os.path.join(output_dir, f"{prefix}_03_odrive_system.{args.format}")
        fig.savefig(path, dpi=args.dpi, bbox_inches="tight")
        print(f"  Saved {path}")
        plt.close(fig)

        fig = plot_odrive_torque(data, velocity)
        path = os.path.join(output_dir, f"{prefix}_04_odrive_torque.{args.format}")
        fig.savefig(path, dpi=args.dpi, bbox_inches="tight")
        print(f"  Saved {path}")
        plt.close(fig)

        fig = plot_odrive_current(data, velocity)
        path = os.path.join(output_dir, f"{prefix}_05_odrive_current.{args.format}")
        fig.savefig(path, dpi=args.dpi, bbox_inches="tight")
        print(f"  Saved {path}")
        plt.close(fig)

        fig = plot_odrive_temperature(data, velocity)
        path = os.path.join(output_dir, f"{prefix}_06_odrive_temperature.{args.format}")
        fig.savefig(path, dpi=args.dpi, bbox_inches="tight")
        print(f"  Saved {path}")
        plt.close(fig)

        fig = plot_motor_commands(data, velocity)
        path = os.path.join(output_dir, f"{prefix}_07_motor_commands.{args.format}")
        fig.savefig(path, dpi=args.dpi, bbox_inches="tight")
        print(f"  Saved {path}")
        plt.close(fig)

        fig, noise_metrics = plot_noise_estimation(data, velocity)
        if fig is not None:
            path = os.path.join(output_dir, f"{prefix}_08_noise_estimation.{args.format}")
            fig.savefig(path, dpi=args.dpi, bbox_inches="tight")
            print(f"  Saved {path}")
            plt.close(fig)
        else:
            noise_metrics = {}

        summary = compute_summary_metrics(data, velocity, noise_metrics)
        all_summaries.append(summary)

        for sig_name in ["Position X", "Position Y", "Velocity X", "Velocity Y"]:
            if sig_name in noise_metrics:
                m = noise_metrics[sig_name]
                print(f"    {sig_name} noise: RMS={m['rms']:.6f}, std={m['std']:.6f}")

    if len(all_summaries) > 1:
        print("\nGenerating summary comparison ...")
        plot_summary(all_summaries, output_dir, args.format, args.dpi)

    print(f"\nDone. {len(bags)} bag(s) analyzed. Plots saved to {output_dir}/")


if __name__ == "__main__":
    main()
