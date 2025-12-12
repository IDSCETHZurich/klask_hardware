"""Utility functions for camera calibration and image processing."""

import os
import numpy as np


def load_calibration_data(
    filename: str = "calibration_data.npz",
) -> tuple[np.ndarray, np.ndarray]:
    """
    Load camera calibration data from file.

    Args:
        filename: Name of the calibration file

    Returns:
        Tuple of (camera_matrix, distortion_coefficients)

    Raises:
        FileNotFoundError: If calibration file or ros2_ws not found
    """
    current_dir = os.path.dirname(os.path.realpath(__file__))
    ros2_ws_index = current_dir.find("ros2_ws")

    if ros2_ws_index == -1:
        raise FileNotFoundError("ROS2 workspace not found in the path")

    ros2_ws_path = current_dir[: ros2_ws_index + len("ros2_ws")]
    filepath = os.path.join(
        ros2_ws_path,
        "src/klask_imaging_pkg/klask_imaging_pkg/data",
        filename,
    )

    if not os.path.exists(filepath):
        raise FileNotFoundError(f"Calibration file not found at: {filepath}")

    with np.load(filepath) as data:
        return data["mtx"], data["dist"]


def apply_ema_filter(
    current_value: np.ndarray, previous_value: np.ndarray | None, alpha: float = 0.1
) -> np.ndarray:
    """
    Apply Exponential Moving Average (EMA) filter for smoothing.

    Args:
        current_value: Current measurement
        previous_value: Previous filtered value (None on first call)
        alpha: Smoothing factor (0-1), higher = more responsive

    Returns:
        Filtered value
    """
    if previous_value is None:
        return current_value
    return alpha * current_value + (1 - alpha) * previous_value

