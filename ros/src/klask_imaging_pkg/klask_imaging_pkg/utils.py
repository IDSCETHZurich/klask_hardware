"""Utility functions for camera calibration and image processing."""

import os
import numpy as np


def load_calibration_data(
    filename: str = "klask_imaging_pkg/data/calibration_data.npz",
) -> tuple[np.ndarray, np.ndarray]:
    """
    Load camera calibration data from file.

    Args:
        filename: Name of the calibration file relative or absolute path

    Returns:
        Tuple of (camera_matrix, distortion_coefficients)

    Raises:
        FileNotFoundError: If calibration file or ros2_ws not found
    """
    # Check if filename is an absolute path
    if os.path.isabs(filename):
        filepath = filename
    else:
        # Relative path - relative to the package directory
        current_dir = os.path.dirname(os.path.realpath(__file__))
        package_dir = os.path.dirname(current_dir)
        filepath = os.path.join(package_dir, filename)

    if not os.path.exists(filepath):
        raise FileNotFoundError(f"Calibration file not found at: {filepath}")

    with np.load(filepath) as data:
        return data["mtx"], data["dist"]


def apply_ema_filter(current_value: np.ndarray, previous_value: np.ndarray | None, alpha: float = 0.1) -> np.ndarray:
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
