"""Utility functions for camera calibration and image processing."""

import os
import cv2
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
        "src/klask_state_estimation_pkg/klask_state_estimation_pkg",
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


def resize_with_aspect_ratio(
    image: np.ndarray, target_width: int, target_height: int
) -> tuple[np.ndarray, int, int]:
    """
    Resize image while maintaining aspect ratio to fit within target dimensions.

    Args:
        image: Input image
        target_width: Maximum width
        target_height: Maximum height

    Returns:
        Tuple of (resized_image, actual_width, actual_height)
    """
    h, w = image.shape[:2]
    aspect_ratio = w / h
    target_aspect_ratio = target_width / target_height

    # Determine which dimension is the limiting factor
    if aspect_ratio > target_aspect_ratio:
        # Width is limiting
        new_w = target_width
        new_h = int(target_width / aspect_ratio)
    else:
        # Height is limiting
        new_h = target_height
        new_w = int(target_height * aspect_ratio)

    resized_image = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_AREA)
    return resized_image, new_w, new_h
