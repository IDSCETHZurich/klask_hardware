"""Utility functions for camera calibration and image processing."""

import cv2
import numpy as np


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
