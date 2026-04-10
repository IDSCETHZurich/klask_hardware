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
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import butter, sosfiltfilt, welch

import rosbag2_py
from rclpy.serialization import deserialize_message

from geometry_msgs.msg import Twist
from klask_interfaces.msg import State
from odrive_can.msg import ControlMessage, ControllerStatus, ODriveStatus
from std_msgs.msg import Float32MultiArray


# Topics to read (skip images, rosout, parameter_events, heartbeat, etc.)
TOPICS_OF_INTEREST = [
    "/board_state",
    "/cmd_vel/left_player",
    "/odrive_axis2/odrive_status",
    "/odrive_axis3/odrive_status",
    "/odrive_axis2/controller_status",
    "/odrive_axis3/controller_status",
    "/odrive_axis2/control_message",
    "/odrive_axis3/control_message",
    "/left_player_magnet_position",
]

TOPIC_MSG_TYPE = {
    "/board_state": State,
    "/cmd_vel/left_player": Twist,
    "/odrive_axis2/odrive_status": ODriveStatus,
    "/odrive_axis3/odrive_status": ODriveStatus,
    "/odrive_axis2/controller_status": ControllerStatus,
    "/odrive_axis3/controller_status": ControllerStatus,
    "/odrive_axis2/control_message": ControlMessage,
    "/odrive_axis3/control_message": ControlMessage,
    "/left_player_magnet_position": Float32MultiArray,
}


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


def read_bag(bag_path):
    """Read a ROS2 bag and return extracted data as dict of numpy arrays."""
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader.open(storage_options, converter_options)

    # Filter to only topics of interest
    reader.set_filter(rosbag2_py.StorageFilter(topics=TOPICS_OF_INTEREST))

    # Accumulators
    acc = {
        "board_state": {"t": [], "px": [], "py": [], "vx": [], "vy": []},
        "cmd_vel": {"t": [], "vx": [], "vy": []},
        "odrive2_status": {
            "t": [], "bus_voltage": [], "bus_current": [],
            "fet_temperature": [], "motor_temperature": [],
        },
        "odrive3_status": {
            "t": [], "bus_voltage": [], "bus_current": [],
            "fet_temperature": [], "motor_temperature": [],
        },
        "odrive2_ctrl": {
            "t": [], "pos_estimate": [], "vel_estimate": [],
            "torque_target": [], "torque_estimate": [],
            "iq_setpoint": [], "iq_measured": [],
        },
        "odrive3_ctrl": {
            "t": [], "pos_estimate": [], "vel_estimate": [],
            "torque_target": [], "torque_estimate": [],
            "iq_setpoint": [], "iq_measured": [],
        },
        "odrive2_cmd": {"t": [], "input_pos": [], "input_vel": [], "input_torque": []},
        "odrive3_cmd": {"t": [], "input_pos": [], "input_vel": [], "input_torque": []},
        "magnet_pos": {"t": [], "x": [], "y": []},
    }

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        msg_type = TOPIC_MSG_TYPE.get(topic)
        if msg_type is None:
            continue
        msg = deserialize_message(data, msg_type)

        if topic == "/board_state":
            # Use header stamp for more accurate timing
            t = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            acc["board_state"]["t"].append(t)
            acc["board_state"]["px"].append(msg.left_peg.position.x)
            acc["board_state"]["py"].append(msg.left_peg.position.y)
            acc["board_state"]["vx"].append(msg.left_peg.velocity.x)
            acc["board_state"]["vy"].append(msg.left_peg.velocity.y)

        elif topic == "/cmd_vel/left_player":
            acc["cmd_vel"]["t"].append(timestamp_ns)
            acc["cmd_vel"]["vx"].append(msg.linear.x)
            acc["cmd_vel"]["vy"].append(msg.linear.y)

        elif topic == "/odrive_axis2/odrive_status":
            acc["odrive2_status"]["t"].append(timestamp_ns)
            acc["odrive2_status"]["bus_voltage"].append(msg.bus_voltage)
            acc["odrive2_status"]["bus_current"].append(msg.bus_current)
            acc["odrive2_status"]["fet_temperature"].append(msg.fet_temperature)
            acc["odrive2_status"]["motor_temperature"].append(msg.motor_temperature)

        elif topic == "/odrive_axis3/odrive_status":
            acc["odrive3_status"]["t"].append(timestamp_ns)
            acc["odrive3_status"]["bus_voltage"].append(msg.bus_voltage)
            acc["odrive3_status"]["bus_current"].append(msg.bus_current)
            acc["odrive3_status"]["fet_temperature"].append(msg.fet_temperature)
            acc["odrive3_status"]["motor_temperature"].append(msg.motor_temperature)

        elif topic == "/odrive_axis2/controller_status":
            acc["odrive2_ctrl"]["t"].append(timestamp_ns)
            acc["odrive2_ctrl"]["pos_estimate"].append(msg.pos_estimate)
            acc["odrive2_ctrl"]["vel_estimate"].append(msg.vel_estimate)
            acc["odrive2_ctrl"]["torque_target"].append(msg.torque_target)
            acc["odrive2_ctrl"]["torque_estimate"].append(msg.torque_estimate)
            acc["odrive2_ctrl"]["iq_setpoint"].append(msg.iq_setpoint)
            acc["odrive2_ctrl"]["iq_measured"].append(msg.iq_measured)

        elif topic == "/odrive_axis3/controller_status":
            acc["odrive3_ctrl"]["t"].append(timestamp_ns)
            acc["odrive3_ctrl"]["pos_estimate"].append(msg.pos_estimate)
            acc["odrive3_ctrl"]["vel_estimate"].append(msg.vel_estimate)
            acc["odrive3_ctrl"]["torque_target"].append(msg.torque_target)
            acc["odrive3_ctrl"]["torque_estimate"].append(msg.torque_estimate)
            acc["odrive3_ctrl"]["iq_setpoint"].append(msg.iq_setpoint)
            acc["odrive3_ctrl"]["iq_measured"].append(msg.iq_measured)

        elif topic == "/odrive_axis2/control_message":
            acc["odrive2_cmd"]["t"].append(timestamp_ns)
            acc["odrive2_cmd"]["input_pos"].append(msg.input_pos)
            acc["odrive2_cmd"]["input_vel"].append(msg.input_vel)
            acc["odrive2_cmd"]["input_torque"].append(msg.input_torque)

        elif topic == "/odrive_axis3/control_message":
            acc["odrive3_cmd"]["t"].append(timestamp_ns)
            acc["odrive3_cmd"]["input_pos"].append(msg.input_pos)
            acc["odrive3_cmd"]["input_vel"].append(msg.input_vel)
            acc["odrive3_cmd"]["input_torque"].append(msg.input_torque)

        elif topic == "/left_player_magnet_position":
            if len(msg.data) >= 2:
                acc["magnet_pos"]["t"].append(timestamp_ns)
                acc["magnet_pos"]["x"].append(msg.data[0])
                acc["magnet_pos"]["y"].append(msg.data[1])

    # Convert to numpy arrays
    data = {}
    for key, fields in acc.items():
        data[key] = {k: np.array(v, dtype=np.float64) for k, v in fields.items()}
    return data


def make_time_relative(data):
    """Normalize all timestamps to t=0 in seconds."""
    # Find global minimum timestamp
    t_mins = []
    for d in data.values():
        if len(d["t"]) > 0:
            t_mins.append(d["t"][0])
    if not t_mins:
        return data
    t_min = min(t_mins)
    for key in data:
        if len(data[key]["t"]) > 0:
            data[key]["t"] = (data[key]["t"] - t_min) * 1e-9
    return data


# ---------------------------------------------------------------------------
# High-pass filter noise estimation
# ---------------------------------------------------------------------------

def compute_noise_metrics(signal, fs, cutoff_hz=5.0, order=4):
    """Apply Butterworth high-pass filter and compute noise statistics."""
    if len(signal) < 20 or fs <= 0:
        return None
    nyq = fs / 2.0
    if cutoff_hz >= nyq:
        cutoff_hz = nyq * 0.8
    sos = butter(order, cutoff_hz, btype="high", fs=fs, output="sos")
    filtered = sosfiltfilt(sos, signal)
    # Skip first and last 0.5s for edge effects
    skip = int(0.5 * fs)
    if skip * 2 >= len(filtered):
        skip = 0
    trimmed = filtered[skip:-skip] if skip > 0 else filtered
    rms = np.sqrt(np.mean(trimmed**2))
    freqs, psd = welch(trimmed, fs=fs, nperseg=min(256, len(trimmed)))
    return {
        "filtered": filtered,
        "rms": rms,
        "std": np.std(trimmed),
        "psd_freqs": freqs,
        "psd_values": psd,
    }


# ---------------------------------------------------------------------------
# Plotting functions
# ---------------------------------------------------------------------------

def plot_velocity_tracking(data, velocity):
    """Figure 1: Commanded vs actual velocity."""
    fig, (ax_x, ax_y) = plt.subplots(2, 1, sharex=True, figsize=(14, 7))
    fig.suptitle(f"Velocity Tracking  —  v = {velocity:.2f} m/s", fontsize=14)

    ax_x.plot(data["cmd_vel"]["t"], data["cmd_vel"]["vx"],
              label="Commanded vx", color="C0", alpha=0.8, linewidth=1)
    ax_x.plot(data["board_state"]["t"], data["board_state"]["vx"],
              label="Actual vx (left peg)", color="C1", alpha=0.8, linewidth=1)
    ax_x.set_ylabel("Velocity X (m/s)")
    ax_x.legend(loc="upper right")
    ax_x.grid(True, alpha=0.3)

    ax_y.plot(data["cmd_vel"]["t"], data["cmd_vel"]["vy"],
              label="Commanded vy", color="C0", alpha=0.8, linewidth=1)
    ax_y.plot(data["board_state"]["t"], data["board_state"]["vy"],
              label="Actual vy (left peg)", color="C1", alpha=0.8, linewidth=1)
    ax_y.set_ylabel("Velocity Y (m/s)")
    ax_y.set_xlabel("Time (s)")
    ax_y.legend(loc="upper right")
    ax_y.grid(True, alpha=0.3)

    fig.tight_layout()
    return fig


def plot_position(data, velocity):
    """Figure 2: Left peg position over time + XY trajectory."""
    fig, (ax_x, ax_y, ax_xy) = plt.subplots(1, 3, figsize=(18, 5))
    fig.suptitle(f"Left Peg Position  —  v = {velocity:.2f} m/s", fontsize=14)

    t = data["board_state"]["t"]
    px = data["board_state"]["px"]
    py = data["board_state"]["py"]

    ax_x.plot(t, px, color="C0", linewidth=1)
    ax_x.set_xlabel("Time (s)")
    ax_x.set_ylabel("Position X (m)")
    ax_x.set_title("X(t)")
    ax_x.grid(True, alpha=0.3)

    ax_y.plot(t, py, color="C1", linewidth=1)
    ax_y.set_xlabel("Time (s)")
    ax_y.set_ylabel("Position Y (m)")
    ax_y.set_title("Y(t)")
    ax_y.grid(True, alpha=0.3)

    ax_xy.plot(px, py, color="C2", linewidth=0.5, alpha=0.7)
    ax_xy.set_xlabel("X (m)")
    ax_xy.set_ylabel("Y (m)")
    ax_xy.set_title("XY Trajectory")
    ax_xy.set_aspect("equal")
    ax_xy.grid(True, alpha=0.3)

    fig.tight_layout()
    return fig


def plot_odrive_system(data, velocity):
    """Figure 3: ODrive bus voltage and current."""
    fig, (ax_v, ax_i) = plt.subplots(2, 1, sharex=True, figsize=(14, 7))
    fig.suptitle(f"ODrive System Status  —  v = {velocity:.2f} m/s", fontsize=14)

    for label, key, color in [("Axis 2", "odrive2_status", "C0"),
                               ("Axis 3", "odrive3_status", "C1")]:
        d = data[key]
        if len(d["t"]) == 0:
            continue
        ax_v.plot(d["t"], d["bus_voltage"], label=label, color=color, linewidth=1, alpha=0.8)
        ax_i.plot(d["t"], d["bus_current"], label=label, color=color, linewidth=1, alpha=0.8)

    ax_v.set_ylabel("Bus Voltage (V)")
    ax_v.legend()
    ax_v.grid(True, alpha=0.3)

    ax_i.set_ylabel("Bus Current (A)")
    ax_i.set_xlabel("Time (s)")
    ax_i.legend()
    ax_i.grid(True, alpha=0.3)

    fig.tight_layout()
    return fig


def plot_odrive_torque(data, velocity):
    """Figure 4: Torque target vs estimate for each axis."""
    fig, (ax2, ax3) = plt.subplots(2, 1, sharex=True, figsize=(14, 7))
    fig.suptitle(f"ODrive Torque  —  v = {velocity:.2f} m/s", fontsize=14)

    for ax, key, name in [(ax2, "odrive2_ctrl", "Axis 2"),
                           (ax3, "odrive3_ctrl", "Axis 3")]:
        d = data[key]
        if len(d["t"]) == 0:
            continue
        ax.plot(d["t"], d["torque_target"], label="Target", color="C0", linewidth=1, alpha=0.8)
        ax.plot(d["t"], d["torque_estimate"], label="Estimate", color="C1", linewidth=1, alpha=0.8)
        ax.set_ylabel(f"{name} Torque (Nm)")
        ax.legend(loc="upper right")
        ax.grid(True, alpha=0.3)

    ax3.set_xlabel("Time (s)")
    fig.tight_layout()
    return fig


def plot_odrive_current(data, velocity):
    """Figure 5: Iq setpoint vs measured for each axis."""
    fig, (ax2, ax3) = plt.subplots(2, 1, sharex=True, figsize=(14, 7))
    fig.suptitle(f"ODrive Current (Iq)  —  v = {velocity:.2f} m/s", fontsize=14)

    for ax, key, name in [(ax2, "odrive2_ctrl", "Axis 2"),
                           (ax3, "odrive3_ctrl", "Axis 3")]:
        d = data[key]
        if len(d["t"]) == 0:
            continue
        ax.plot(d["t"], d["iq_setpoint"], label="Iq Setpoint", color="C0", linewidth=1, alpha=0.8)
        ax.plot(d["t"], d["iq_measured"], label="Iq Measured", color="C1", linewidth=1, alpha=0.8)
        ax.set_ylabel(f"{name} Current (A)")
        ax.legend(loc="upper right")
        ax.grid(True, alpha=0.3)

    ax3.set_xlabel("Time (s)")
    fig.tight_layout()
    return fig


def plot_odrive_temperature(data, velocity):
    """Figure 6: FET and motor temperature for each axis."""
    fig, (ax2, ax3) = plt.subplots(2, 1, sharex=True, figsize=(14, 7))
    fig.suptitle(f"ODrive Temperature  —  v = {velocity:.2f} m/s", fontsize=14)

    for ax, key, name in [(ax2, "odrive2_status", "Axis 2"),
                           (ax3, "odrive3_status", "Axis 3")]:
        d = data[key]
        if len(d["t"]) == 0:
            continue
        ax.plot(d["t"], d["fet_temperature"], label="FET Temp", color="C0", linewidth=1, alpha=0.8)
        # Motor temp may be NaN
        motor_t = d["motor_temperature"]
        valid = np.isfinite(motor_t)
        if np.any(valid):
            ax.plot(d["t"][valid], motor_t[valid], label="Motor Temp",
                    color="C1", linewidth=1, alpha=0.8)
        ax.set_ylabel(f"{name} Temperature (C)")
        ax.legend(loc="upper right")
        ax.grid(True, alpha=0.3)

    ax3.set_xlabel("Time (s)")
    fig.tight_layout()
    return fig


def plot_motor_commands(data, velocity):
    """Figure 7: Motor command inputs (pos, vel, torque) for each axis."""
    fig, axes = plt.subplots(3, 2, sharex=True, figsize=(16, 10))
    fig.suptitle(f"Motor Commands  —  v = {velocity:.2f} m/s", fontsize=14)

    for col, key, name in [(0, "odrive2_cmd", "Axis 2"),
                            (1, "odrive3_cmd", "Axis 3")]:
        d = data[key]
        if len(d["t"]) == 0:
            continue
        axes[0, col].plot(d["t"], d["input_pos"], color="C0", linewidth=0.8)
        axes[0, col].set_ylabel("Input Pos (rev)")
        axes[0, col].set_title(name)
        axes[0, col].grid(True, alpha=0.3)

        axes[1, col].plot(d["t"], d["input_vel"], color="C1", linewidth=0.8)
        axes[1, col].set_ylabel("Input Vel (rev/s)")
        axes[1, col].grid(True, alpha=0.3)

        axes[2, col].plot(d["t"], d["input_torque"], color="C2", linewidth=0.8)
        axes[2, col].set_ylabel("Input Torque (Nm)")
        axes[2, col].set_xlabel("Time (s)")
        axes[2, col].grid(True, alpha=0.3)

    fig.tight_layout()
    return fig


def plot_noise_estimation(data, velocity):
    """Figure 8: High-pass filtered position and velocity for noise estimation.

    Returns (fig, noise_metrics_dict).
    """
    bs = data["board_state"]
    if len(bs["t"]) < 20:
        return None, {}

    dt = np.median(np.diff(bs["t"]))
    fs = 1.0 / dt if dt > 0 else 48.0

    signals = {
        "Position X": bs["px"],
        "Position Y": bs["py"],
        "Velocity X": bs["vx"],
        "Velocity Y": bs["vy"],
    }

    fig, axes = plt.subplots(4, 3, figsize=(18, 14))
    fig.suptitle(
        f"State Estimator Noise (HPF cutoff=5 Hz)  —  v = {velocity:.2f} m/s",
        fontsize=14,
    )

    noise_metrics = {}
    for row, (sig_name, sig) in enumerate(signals.items()):
        metrics = compute_noise_metrics(sig, fs, cutoff_hz=5.0)
        if metrics is None:
            continue
        noise_metrics[sig_name] = metrics

        # Time-domain filtered signal
        axes[row, 0].plot(bs["t"], metrics["filtered"], color="C3", linewidth=0.5, alpha=0.8)
        axes[row, 0].set_ylabel(sig_name)
        axes[row, 0].set_title(f"HPF Signal  (RMS={metrics['rms']:.6f})" if row == 0
                                else f"RMS={metrics['rms']:.6f}")
        axes[row, 0].grid(True, alpha=0.3)
        if row == 3:
            axes[row, 0].set_xlabel("Time (s)")

        # Histogram
        axes[row, 1].hist(metrics["filtered"], bins=80, color="C4", alpha=0.7, density=True)
        axes[row, 1].set_title(f"std={metrics['std']:.6f}" if row > 0 else
                                f"Distribution  (std={metrics['std']:.6f})")
        axes[row, 1].grid(True, alpha=0.3)

        # PSD
        axes[row, 2].semilogy(metrics["psd_freqs"], metrics["psd_values"],
                               color="C5", linewidth=1)
        axes[row, 2].set_title("PSD" if row == 0 else "")
        axes[row, 2].set_ylabel("Power")
        axes[row, 2].grid(True, alpha=0.3)
        if row == 3:
            axes[row, 2].set_xlabel("Frequency (Hz)")

    fig.tight_layout()
    return fig, noise_metrics


# ---------------------------------------------------------------------------
# Summary across all velocities
# ---------------------------------------------------------------------------

def compute_summary_metrics(data, velocity, noise_metrics):
    """Compute summary statistics for a single bag."""
    summary = {"velocity": velocity}

    # Velocity tracking error RMS (interpolate cmd_vel to board_state timestamps)
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

    # Noise RMS
    for sig_name in ["Position X", "Position Y", "Velocity X", "Velocity Y"]:
        key = f"noise_rms_{sig_name.lower().replace(' ', '_')}"
        if sig_name in noise_metrics:
            summary[key] = noise_metrics[sig_name]["rms"]
        else:
            summary[key] = np.nan

    # ODrive metrics
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

    # (0,0) Velocity tracking error RMS
    ax = axes[0, 0]
    ax.plot(vels, [s["vel_err_rms_x"] for s in all_summaries], "o-", label="X", color="C0")
    ax.plot(vels, [s["vel_err_rms_y"] for s in all_summaries], "s-", label="Y", color="C1")
    ax.set_xlabel("Commanded Velocity (m/s)")
    ax.set_ylabel("Velocity Error RMS (m/s)")
    ax.set_title("Velocity Tracking Error")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # (0,1) Position noise RMS
    ax = axes[0, 1]
    ax.plot(vels, [s["noise_rms_position_x"] for s in all_summaries], "o-", label="X", color="C0")
    ax.plot(vels, [s["noise_rms_position_y"] for s in all_summaries], "s-", label="Y", color="C1")
    ax.set_xlabel("Commanded Velocity (m/s)")
    ax.set_ylabel("Position Noise RMS (m)")
    ax.set_title("Position Noise (HPF)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # (0,2) Velocity noise RMS
    ax = axes[0, 2]
    ax.plot(vels, [s["noise_rms_velocity_x"] for s in all_summaries], "o-", label="X", color="C0")
    ax.plot(vels, [s["noise_rms_velocity_y"] for s in all_summaries], "s-", label="Y", color="C1")
    ax.set_xlabel("Commanded Velocity (m/s)")
    ax.set_ylabel("Velocity Noise RMS (m/s)")
    ax.set_title("Velocity Noise (HPF)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # (1,0) Mean bus current
    ax = axes[1, 0]
    ax.plot(vels, [s["odrive2_mean_bus_current"] for s in all_summaries], "o-",
            label="Axis 2", color="C0")
    ax.plot(vels, [s["odrive3_mean_bus_current"] for s in all_summaries], "s-",
            label="Axis 3", color="C1")
    ax.set_xlabel("Commanded Velocity (m/s)")
    ax.set_ylabel("Mean Bus Current (A)")
    ax.set_title("Bus Current")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # (1,1) Mean torque
    ax = axes[1, 1]
    ax.plot(vels, [s["odrive2_mean_torque"] for s in all_summaries], "o-",
            label="Axis 2", color="C0")
    ax.plot(vels, [s["odrive3_mean_torque"] for s in all_summaries], "s-",
            label="Axis 3", color="C1")
    ax.set_xlabel("Commanded Velocity (m/s)")
    ax.set_ylabel("Mean |Torque| (Nm)")
    ax.set_title("Motor Torque")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # (1,2) Max FET temperature
    ax = axes[1, 2]
    ax.plot(vels, [s["odrive2_max_fet_temp"] for s in all_summaries], "o-",
            label="Axis 2", color="C0")
    ax.plot(vels, [s["odrive3_max_fet_temp"] for s in all_summaries], "s-",
            label="Axis 3", color="C1")
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
    parser = argparse.ArgumentParser(
        description="Analyze Klask acceleration test bag files"
    )
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
    parser.add_argument(
        "--format", type=str, default="png", choices=["png", "pdf", "svg"]
    )
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

        # Figure 1: Velocity Tracking
        fig = plot_velocity_tracking(data, velocity)
        path = os.path.join(output_dir, f"{prefix}_01_velocity_tracking.{args.format}")
        fig.savefig(path, dpi=args.dpi, bbox_inches="tight")
        print(f"  Saved {path}")
        plt.close(fig)

        # Figure 2: Position
        fig = plot_position(data, velocity)
        path = os.path.join(output_dir, f"{prefix}_02_position.{args.format}")
        fig.savefig(path, dpi=args.dpi, bbox_inches="tight")
        print(f"  Saved {path}")
        plt.close(fig)

        # Figure 3: ODrive System Status
        fig = plot_odrive_system(data, velocity)
        path = os.path.join(output_dir, f"{prefix}_03_odrive_system.{args.format}")
        fig.savefig(path, dpi=args.dpi, bbox_inches="tight")
        print(f"  Saved {path}")
        plt.close(fig)

        # Figure 4: ODrive Torque
        fig = plot_odrive_torque(data, velocity)
        path = os.path.join(output_dir, f"{prefix}_04_odrive_torque.{args.format}")
        fig.savefig(path, dpi=args.dpi, bbox_inches="tight")
        print(f"  Saved {path}")
        plt.close(fig)

        # Figure 5: ODrive Current
        fig = plot_odrive_current(data, velocity)
        path = os.path.join(output_dir, f"{prefix}_05_odrive_current.{args.format}")
        fig.savefig(path, dpi=args.dpi, bbox_inches="tight")
        print(f"  Saved {path}")
        plt.close(fig)

        # Figure 6: ODrive Temperature
        fig = plot_odrive_temperature(data, velocity)
        path = os.path.join(output_dir, f"{prefix}_06_odrive_temperature.{args.format}")
        fig.savefig(path, dpi=args.dpi, bbox_inches="tight")
        print(f"  Saved {path}")
        plt.close(fig)

        # Figure 7: Motor Commands
        fig = plot_motor_commands(data, velocity)
        path = os.path.join(output_dir, f"{prefix}_07_motor_commands.{args.format}")
        fig.savefig(path, dpi=args.dpi, bbox_inches="tight")
        print(f"  Saved {path}")
        plt.close(fig)

        # Figure 8: Noise Estimation
        fig, noise_metrics = plot_noise_estimation(data, velocity)
        if fig is not None:
            path = os.path.join(output_dir, f"{prefix}_08_noise_estimation.{args.format}")
            fig.savefig(path, dpi=args.dpi, bbox_inches="tight")
            print(f"  Saved {path}")
            plt.close(fig)
        else:
            noise_metrics = {}

        # Collect summary
        summary = compute_summary_metrics(data, velocity, noise_metrics)
        all_summaries.append(summary)

        # Print noise summary
        for sig_name in ["Position X", "Position Y", "Velocity X", "Velocity Y"]:
            if sig_name in noise_metrics:
                m = noise_metrics[sig_name]
                print(f"    {sig_name} noise: RMS={m['rms']:.6f}, std={m['std']:.6f}")

    # Summary comparison
    if len(all_summaries) > 1:
        print("\nGenerating summary comparison ...")
        plot_summary(all_summaries, output_dir, args.format, args.dpi)

    print(f"\nDone. {len(bags)} bag(s) analyzed. Plots saved to {output_dir}/")


if __name__ == "__main__":
    main()
