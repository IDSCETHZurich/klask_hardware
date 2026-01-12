"""
Debugging utilities for border segment analysis in imaging."""

import cv2
import pstats
import io
import numpy as np


def print_initial_debug_view(
    frame_rec: np.ndarray,
    h: np.ndarray,
    s: np.ndarray,
    v: np.ndarray,
    flood_seed: list[tuple[int, int]],
    flood_mask: np.ndarray,
    rotated_rect: cv2.RotatedRect,
    warped_v_channel: np.ndarray,
    goal_ellipses: tuple[cv2.RotatedRect, cv2.RotatedRect],
) -> None:
    """
    Display debug views for initial board analysis.

    Args:
        frame_rec (np.ndarray): The original rectified frame.
        h (np.ndarray): The H channel of the image.
        s (np.ndarray): The S channel of the image.
        v (np.ndarray): The V channel of the image.
        flood_mask (np.ndarray): The flood fill mask.
        rotated_rect (cv2.RotatedRect): The detected rotated rectangle.
    """
    # Display warped V channel with goal ellipses
    for goal_ellipse in goal_ellipses:
        cv2.ellipse(warped_v_channel, goal_ellipse, (0, 255, 0), 2)
    plot_single_channel(warped_v_channel, "Warped V Channel with Goal Seeds")

    # Draw the rotated rectangle on a copy of the original image
    frame_with_rect = frame_rec.copy()
    box = cv2.boxPoints(rotated_rect)
    box = np.intp(box)
    cv2.drawContours(frame_with_rect, [box], 0, (0, 255, 0), 2)

    for center_pt in flood_seed:
        cv2.circle(frame_with_rect, center_pt, 5, (0, 255, 0), -1)
    cv2.imshow("Initial Board Analysis - Flood Fill Rectangle", frame_with_rect)

    cv2.imshow("Initial Board Analysis - Flood Fill", flood_mask)

    plot_single_channel(v, "Initial Board Analysis - V Channel")
    plot_single_channel(s, "Initial Board Analysis - S Channel")
    plot_single_channel(h, "Initial Board Analysis - H Channel")
    cv2.imshow("Initial Board Analysis - Rect", frame_rec)


def print_segment_debug_view(
    frame_rec: np.ndarray,
    s_channel: np.ndarray,
    boarder_segment_masks: list[np.ndarray],
    seed_lines: list[tuple[np.ndarray, np.ndarray]],
    seed_line_samples: list[list[np.ndarray]],
    boarder_segment_flood_masks: list[np.ndarray],
    boarder_segment_flood_masks_aligned: list[np.ndarray],
    boarder_segment_edge_points: list[np.ndarray],
    boarder_segment_edge_lines: list[list[np.ndarray]],
    warped_from_corners: np.ndarray,
):
    """Display debug views for border segment analysis.

    Args:
        frame_rec (np.ndarray): The original rectified frame.
        s_channel (np.ndarray): The S channel of the image.
        boarder_segment_masks (list[np.ndarray]): List of masks for each border segment.
        seed_lines (list[tuple[np.ndarray, np.ndarray]]): List of seed lines for each segment.
        seed_line_samples (list[list[np.ndarray]]): List of sample points along each seed line.
        boarder_segment_flood_masks (list[np.ndarray]): List of flood fill masks for each segment.
        boarder_segment_flood_masks_aligned (list[np.ndarray]): List of aligned flood masks.
        boarder_segment_edge_points (list[np.ndarray]): List of edge points for each segment.
        boarder_segment_edge_lines (list[list[np.ndarray]]): List of fitted line parameters for each segment.
        warped_from_corners (np.ndarray): The warped image from fitted corners.
    """
    # Display warped image from fitted corners
    cv2.imshow("Initial Board Analysis - Warped from Fitted Corners", warped_from_corners)

    # Draw all border segment polygons on the original frame
    frame_with_polygons = frame_rec.copy()

    merged_mask = np.zeros_like(s_channel, dtype=np.uint8)
    merged_flood_mask = np.zeros_like(s_channel, dtype=np.uint8)
    for i, (
        mask,
        seed_line,
        seed_line_sample,
        flood_mask,
        aligned_mask,
        edge_points_original,
        line_params,
    ) in enumerate(
        zip(
            boarder_segment_masks,
            seed_lines,
            seed_line_samples,
            boarder_segment_flood_masks,
            boarder_segment_flood_masks_aligned,
            boarder_segment_edge_points,
            boarder_segment_edge_lines,
        )
    ):
        # Find contours from the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(frame_with_polygons, contours, -1, (255, 0, 0), 2)
        # Draw seed line
        cv2.line(
            frame_with_polygons,
            seed_line[:, 0],
            seed_line[:, 1],
            (255, 255, 0),
            2,
        )
        # Draw seed sample points
        for sample_point in seed_line_sample:
            cv2.circle(
                frame_with_polygons,
                tuple(sample_point),
                5,
                (0, 255, 255),
                -1,
            )
        # Merge the current mask into the combined mask
        merged_mask = cv2.bitwise_or(merged_mask, mask)
        merged_flood_mask = cv2.bitwise_or(merged_flood_mask, flood_mask)

        # Display aligned flood mask
        cv2.imshow(f"Initial Board Analysis - Aligned Segment {i}", aligned_mask)

    # Prepare frame with edge points and fitted lines
    frame_with_edges = _prepare_frame_with_lines(frame_rec, boarder_segment_edge_points, boarder_segment_edge_lines)
    cv2.imshow("Initial Board Analysis - Edge Points and Fitted Lines", frame_with_edges)
    cv2.imshow(
        "Initial Board Analysis - Border Segment Flood Mask",
        merged_flood_mask,
    )
    # Display border segment
    masked_image = cv2.bitwise_and(s_channel, s_channel, mask=merged_mask)
    plot_single_channel(masked_image, "Initial Board Analysis - Border Segments")
    cv2.imshow("Initial Board Analysis - Border Segments Overlay", frame_with_polygons)


def _prepare_frame_with_lines(frame_rec, boarder_segment_edge_points, boarder_segment_edge_lines):
    frame_with_edges = frame_rec.copy()
    for (
        edge_points_original,
        line_params,
    ) in zip(
        boarder_segment_edge_points,
        boarder_segment_edge_lines,
    ):
        # Draw edge points and fitted line on original image
        vx, vy, x0, y0 = line_params
        # Draw the fitted line across the image
        # Parametric line: p = p0 + t*v
        # Find intersections with image boundaries
        img_h, img_w = frame_rec.shape[:2]

        # Check if line is nearly vertical or horizontal
        if abs(vx[0]) < 1e-6:  # Nearly vertical line
            pt1 = (int(x0[0]), 0)
            pt2 = (int(x0[0]), img_h - 1)
        elif abs(vy[0]) < 1e-6:  # Nearly horizontal line
            pt1 = (0, int(y0[0]))
            pt2 = (img_w - 1, int(y0[0]))
        else:
            # General case: find intersections with image boundaries
            # Left edge (x=0): t = -x0/vx, y = y0 + t*vy
            t_left = -x0[0] / vx[0]
            y_left = int(y0[0] + t_left * vy[0])
            # Right edge (x=w-1): t = (w-1-x0)/vx, y = y0 + t*vy
            t_right = (img_w - 1 - x0[0]) / vx[0]
            y_right = int(y0[0] + t_right * vy[0])
            # Top edge (y=0): t = -y0/vy, x = x0 + t*vx
            t_top = -y0[0] / vy[0]
            x_top = int(x0[0] + t_top * vx[0])
            # Bottom edge (y=h-1): t = (h-1-y0)/vy, x = x0 + t*vx
            t_bottom = (img_h - 1 - y0[0]) / vy[0]
            x_bottom = int(x0[0] + t_bottom * vx[0])

            # Collect valid intersection points
            points = []
            if 0 <= y_left < img_h:
                points.append((0, y_left))
            if 0 <= y_right < img_h:
                points.append((img_w - 1, y_right))
            if 0 <= x_top < img_w:
                points.append((x_top, 0))
            if 0 <= x_bottom < img_w:
                points.append((x_bottom, img_h - 1))

            # Use first two valid points
            if len(points) >= 2:
                pt1, pt2 = points[:2]
            else:
                # Fallback to center point if no valid intersections
                pt1 = (int(x0[0]), int(y0[0]))
                pt2 = (int(x0[0] + vx[0] * 100), int(y0[0] + vy[0] * 100))

        cv2.line(
            frame_with_edges,
            pt1,
            pt2,
            (0, 255, 0),
            2,
        )
        edge_points_int = edge_points_original.astype(np.int32)
        frame_with_edges[edge_points_int[:, 1], edge_points_int[:, 0]] = [0, 0, 255]
    return frame_with_edges


def plot_single_channel(channel: np.ndarray, title: str) -> None:
    """Plot a single channel with color scale for visualization.

    Args:
        channel: Single channel image array (0-255 values).
        title: Window title for display.
    """

    # Apply colormap to the channel for better visualization
    channel_colored = cv2.applyColorMap(channel, cv2.COLORMAP_JET)

    # Create a color scale bar (0-255 range)
    scale_height = channel_colored.shape[0]
    scale_width = 50
    scale_bar = np.linspace(255, 0, scale_height, dtype=np.uint8).reshape(-1, 1)
    scale_bar = np.tile(scale_bar, (1, scale_width))
    scale_bar_colored = cv2.applyColorMap(scale_bar, cv2.COLORMAP_JET)

    # Add text labels to the scale
    for i in range(0, 256, 51):  # Labels at 0, 51, 102, 153, 204, 255
        y_pos = int((255 - i) / 255 * (scale_height - 1))
        cv2.putText(
            scale_bar_colored,
            str(i),
            (5, y_pos + 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

    # Concatenate the colored channel with the scale bar
    channel_with_scale = np.hstack([channel_colored, scale_bar_colored])

    cv2.imshow(title, channel_with_scale)


def show_online_boarders(
    frame_rec: np.ndarray,
    boarder_segment_edge_points,
    boarder_segment_edge_lines,
    fps_display,
    title,
) -> None:
    """Display board borders with edge detection lines overlay.

    Args:
        frame_rec: The original rectified frame.
        boarder_segment_edge_points: List of edge points for each border segment.
        boarder_segment_edge_lines: List of fitted line parameters for each segment.
        fps_display: Current frames per second for display.
        title: Window title for display.
    """
    frame_with_edges = _prepare_frame_with_lines(frame_rec, boarder_segment_edge_points, boarder_segment_edge_lines)

    show_final_output(frame_with_edges, fps_display, title)


def show_final_output(warped, fps_display, title, goal_ellipses=None) -> None:
    """Display the final warped board view with FPS overlay.

    Args:
        warped: The perspective-corrected board image.
        fps_display: Current frames per second for display.
        title: Window title for display.
        goal_ellipses: Optional tuple of goal ellipse parameters to overlay.
    """

    # Draw FPS on image
    display_image = warped.copy()
    fps_text = f"FPS: {fps_display}"
    cv2.putText(
        display_image,
        fps_text,
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    if goal_ellipses is not None:
        for goal_ellipse in goal_ellipses:
            cv2.ellipse(display_image, goal_ellipse, (0, 255, 0), 2)
            cv2.circle(
                display_image,
                (int(goal_ellipse[0][0]), int(goal_ellipse[0][1])),
                5,
                (0, 255, 0),
                -1,
            )

    cv2.imshow(title, display_image)
    cv2.waitKey(1)


def print_profiling_stats(profiler, top_functions: int) -> None:
    """Print cProfile statistics for performance analysis.

    Args:
        profiler: The cProfile profiler object.
        top_functions: Number of top functions to display in the output.
    """
    if profiler is None:
        return

    # Create a string buffer to capture the stats output
    s = io.StringIO()

    # Sort by total time (tottime) and print top functions
    ps = pstats.Stats(profiler, stream=s).sort_stats("tottime")

    print("\n" + "=" * 80)
    print("cProfile Statistics")
    print("=" * 80)

    # Print stats to string buffer
    ps.print_stats(top_functions)

    # Print the output
    print("\n" + s.getvalue())

    # Also print callers for more detailed analysis
    s = io.StringIO()
    ps = pstats.Stats(profiler, stream=s).sort_stats("tottime")
    ps.print_callers(20)

    print("\nTop Callers:")
    print(s.getvalue())
    print("=" * 80 + "\n")
