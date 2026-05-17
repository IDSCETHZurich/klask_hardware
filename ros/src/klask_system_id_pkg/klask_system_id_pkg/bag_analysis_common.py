"""Shared bag-reading and plotting utilities for klask system-id analyses.

Both the acceleration-test analyzer and the velocity-profile analyzer
record the same set of topics, so the bag reader and the per-bag
plotting primitives live here to avoid drift between the two scripts.
"""

import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import butter, sosfiltfilt, welch

import rosbag2_py
from rclpy.serialization import deserialize_message

from geometry_msgs.msg import Twist
from klask_interfaces.msg import State
from odrive_can.msg import ControlMessage, ControllerStatus, ODriveStatus
from std_msgs.msg import Float32MultiArray

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


def read_bag(bag_path):
    """Read a ROS2 bag and return extracted data as dict of numpy arrays."""
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(storage_options, converter_options)

    reader.set_filter(rosbag2_py.StorageFilter(topics=TOPICS_OF_INTEREST))

    acc = {
        "board_state": {"t": [], "px": [], "py": [], "vx": [], "vy": []},
        "cmd_vel": {"t": [], "vx": [], "vy": []},
        "odrive2_status": {
            "t": [],
            "bus_voltage": [],
            "bus_current": [],
            "fet_temperature": [],
            "motor_temperature": [],
        },
        "odrive3_status": {
            "t": [],
            "bus_voltage": [],
            "bus_current": [],
            "fet_temperature": [],
            "motor_temperature": [],
        },
        "odrive2_ctrl": {
            "t": [],
            "pos_estimate": [],
            "vel_estimate": [],
            "torque_target": [],
            "torque_estimate": [],
            "iq_setpoint": [],
            "iq_measured": [],
        },
        "odrive3_ctrl": {
            "t": [],
            "pos_estimate": [],
            "vel_estimate": [],
            "torque_target": [],
            "torque_estimate": [],
            "iq_setpoint": [],
            "iq_measured": [],
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

    data = {}
    for key, fields in acc.items():
        data[key] = {k: np.array(v, dtype=np.float64) for k, v in fields.items()}
    return data


def make_time_relative(data):
    """Normalize all timestamps to t=0 in seconds."""
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


_NOISE_SUMMARY_SIGNALS = ("Position X", "Position Y", "Velocity X", "Velocity Y")


def extract_noise_rms_summary(noise_metrics):
    """Flatten the noise_metrics dict into noise_rms_<signal> floats (NaN if absent)."""
    out = {}
    for sig_name in _NOISE_SUMMARY_SIGNALS:
        key = f"noise_rms_{sig_name.lower().replace(' ', '_')}"
        out[key] = noise_metrics[sig_name]["rms"] if sig_name in noise_metrics else np.nan
    return out


def compute_noise_metrics(signal, fs, cutoff_hz=5.0, order=4):
    """Apply Butterworth high-pass filter and compute noise statistics."""
    if len(signal) < 20 or fs <= 0:
        return None
    nyq = fs / 2.0
    if cutoff_hz >= nyq:
        cutoff_hz = nyq * 0.8
    sos = butter(order, cutoff_hz, btype="high", fs=fs, output="sos")
    filtered = sosfiltfilt(sos, signal)
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
# Per-bag plotting functions
# ---------------------------------------------------------------------------


def plot_velocity_tracking(data, velocity):
    """Figure: Commanded vs actual velocity."""
    fig, (ax_x, ax_y) = plt.subplots(2, 1, sharex=True, figsize=(14, 7))
    fig.suptitle(f"Velocity Tracking  —  v = {velocity:.2f} m/s", fontsize=14)

    ax_x.plot(data["cmd_vel"]["t"], data["cmd_vel"]["vx"], label="Commanded vx", color="C0", alpha=0.8, linewidth=1)
    ax_x.plot(
        data["board_state"]["t"],
        data["board_state"]["vx"],
        label="Actual vx (left peg)",
        color="C1",
        alpha=0.8,
        linewidth=1,
    )
    ax_x.set_ylabel("Velocity X (m/s)")
    ax_x.legend(loc="upper right")
    ax_x.grid(True, alpha=0.3)

    ax_y.plot(data["cmd_vel"]["t"], data["cmd_vel"]["vy"], label="Commanded vy", color="C0", alpha=0.8, linewidth=1)
    ax_y.plot(
        data["board_state"]["t"],
        data["board_state"]["vy"],
        label="Actual vy (left peg)",
        color="C1",
        alpha=0.8,
        linewidth=1,
    )
    ax_y.set_ylabel("Velocity Y (m/s)")
    ax_y.set_xlabel("Time (s)")
    ax_y.legend(loc="upper right")
    ax_y.grid(True, alpha=0.3)

    fig.tight_layout()
    return fig


def plot_position(data, velocity):
    """Figure: Left peg position over time + XY trajectory."""
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
    """Figure: ODrive bus voltage and current."""
    fig, (ax_v, ax_i) = plt.subplots(2, 1, sharex=True, figsize=(14, 7))
    fig.suptitle(f"ODrive System Status  —  v = {velocity:.2f} m/s", fontsize=14)

    for label, key, color in [("Axis 2", "odrive2_status", "C0"), ("Axis 3", "odrive3_status", "C1")]:
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
    """Figure: Torque target vs estimate for each axis."""
    fig, (ax2, ax3) = plt.subplots(2, 1, sharex=True, figsize=(14, 7))
    fig.suptitle(f"ODrive Torque  —  v = {velocity:.2f} m/s", fontsize=14)

    for ax, key, name in [(ax2, "odrive2_ctrl", "Axis 2"), (ax3, "odrive3_ctrl", "Axis 3")]:
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
    """Figure: Iq setpoint vs measured for each axis."""
    fig, (ax2, ax3) = plt.subplots(2, 1, sharex=True, figsize=(14, 7))
    fig.suptitle(f"ODrive Current (Iq)  —  v = {velocity:.2f} m/s", fontsize=14)

    for ax, key, name in [(ax2, "odrive2_ctrl", "Axis 2"), (ax3, "odrive3_ctrl", "Axis 3")]:
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
    """Figure: FET and motor temperature for each axis."""
    fig, (ax2, ax3) = plt.subplots(2, 1, sharex=True, figsize=(14, 7))
    fig.suptitle(f"ODrive Temperature  —  v = {velocity:.2f} m/s", fontsize=14)

    for ax, key, name in [(ax2, "odrive2_status", "Axis 2"), (ax3, "odrive3_status", "Axis 3")]:
        d = data[key]
        if len(d["t"]) == 0:
            continue
        ax.plot(d["t"], d["fet_temperature"], label="FET Temp", color="C0", linewidth=1, alpha=0.8)
        motor_t = d["motor_temperature"]
        valid = np.isfinite(motor_t)
        if np.any(valid):
            ax.plot(d["t"][valid], motor_t[valid], label="Motor Temp", color="C1", linewidth=1, alpha=0.8)
        ax.set_ylabel(f"{name} Temperature (C)")
        ax.legend(loc="upper right")
        ax.grid(True, alpha=0.3)

    ax3.set_xlabel("Time (s)")
    fig.tight_layout()
    return fig


def plot_motor_commands(data, velocity):
    """Figure: Motor command inputs (pos, vel, torque) for each axis."""
    fig, axes = plt.subplots(3, 2, sharex=True, figsize=(16, 10))
    fig.suptitle(f"Motor Commands  —  v = {velocity:.2f} m/s", fontsize=14)

    for col, key, name in [(0, "odrive2_cmd", "Axis 2"), (1, "odrive3_cmd", "Axis 3")]:
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
    """Figure: High-pass filtered position and velocity for noise estimation.

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

        axes[row, 0].plot(bs["t"], metrics["filtered"], color="C3", linewidth=0.5, alpha=0.8)
        axes[row, 0].set_ylabel(sig_name)
        axes[row, 0].set_title(f"HPF Signal  (RMS={metrics['rms']:.6f})" if row == 0 else f"RMS={metrics['rms']:.6f}")
        axes[row, 0].grid(True, alpha=0.3)
        if row == 3:
            axes[row, 0].set_xlabel("Time (s)")

        axes[row, 1].hist(metrics["filtered"], bins=80, color="C4", alpha=0.7, density=True)
        axes[row, 1].set_title(f"std={metrics['std']:.6f}" if row > 0 else f"Distribution  (std={metrics['std']:.6f})")
        axes[row, 1].grid(True, alpha=0.3)

        axes[row, 2].semilogy(metrics["psd_freqs"], metrics["psd_values"], color="C5", linewidth=1)
        axes[row, 2].set_title("PSD" if row == 0 else "")
        axes[row, 2].set_ylabel("Power")
        axes[row, 2].grid(True, alpha=0.3)
        if row == 3:
            axes[row, 2].set_xlabel("Frequency (Hz)")

    fig.tight_layout()
    return fig, noise_metrics
