"""ROS2 node for camera image acquisition and perspective transformation."""

import itertools
import cv2
import time
import rclpy
import numpy as np
import cProfile
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Polygon, Point32
from klask_interfaces.msg import StampedPolygon
from cv_bridge import CvBridge

from .utils import load_calibration_data, apply_ema_filter
from .debug import (
    print_segment_debug_view,
    print_initial_debug_view,
    print_profiling_stats,
    show_final_output,
    show_online_boarders,
)


class CameraNode(Node):
    """ROS2 node for acquiring camera images and publishing transformed board view."""

    def __init__(self):
        super().__init__("camera_node")

        # Declare ROS parameters with default values
        # A detailed description of each parameter is provided in the config YAML file.

        # Camera settings
        self.declare_parameter("camera_fps", 120)
        self.declare_parameter("camera_width", 1280)
        self.declare_parameter("camera_height", 720)

        # Topic + calibration settings
        self.declare_parameter("board_image_topic", "board_image/compressed")
        self.declare_parameter("goal_positions_topic", "goal_positions")
        self.declare_parameter(
            "calibration_file", "klask_imaging_pkg/data/calibration_data.npz"
        )

        # Detection thresholds
        self.declare_parameter("flood_threshold", 120)
        self.declare_parameter("goal_flood_threshold", 20)

        # Border segment settings
        self.declare_parameter("boarder_seg_inside_offset", 30)
        self.declare_parameter("boarder_seg_outside_offset", 20)
        self.declare_parameter("boarder_seg_corner_distance", 100)

        # Goal detection settings
        self.declare_parameter(
            "goal_h_factor", 0.085
        )  # Horizontal factor for goal seed positioning
        self.declare_parameter(
            "goal_o_factor", 0.01
        )  # Offset factor for goal seed positioning
        self.declare_parameter(
            "goal_e_factor", 1.2
        )  # Ellipse scaling factor for goal size

        # Image processing settings
        self.declare_parameter(
            "use_smoothing", True
        )  # Apply smoothing to improve border detection stability
        self.declare_parameter(
            "smoothing_kernel", 5
        )  # Kernel size for Gaussian blur (3, 5, 7, etc.)

        # Line validation settings
        self.declare_parameter("line_angle_threshold", 10.0)  # degrees
        self.declare_parameter("line_position_threshold", 50.0)  # pixels
        self.declare_parameter("min_edge_points", 10)  # minimum points for line fitting

        # Debug/Display settings
        self.declare_parameter(
            "debug_view", False
        )  # Enable extensive debug views of initial board detection
        self.declare_parameter("show_image", False)  # Display final output image
        self.declare_parameter("show_image_fps", 10)  # Display update frequency in Hz

        # Profiling settings
        self.declare_parameter("enable_profiling", False)
        self.declare_parameter(
            "profiling_duration", 100.0
        )  # Run profiler for N seconds
        self.declare_parameter(
            "profiling_top_functions", 30
        )  # Show top N functions in stats

        # Get parameter values
        self.camera_fps = self.get_parameter("camera_fps").value
        self.camera_width = self.get_parameter("camera_width").value
        self.camera_height = self.get_parameter("camera_height").value
        self.board_image_topic = self.get_parameter("board_image_topic").value
        self.goal_positions_topic = self.get_parameter("goal_positions_topic").value
        self.calibration_file = self.get_parameter("calibration_file").value
        self.flood_threshold = self.get_parameter("flood_threshold").value
        self.goal_flood_threshold = self.get_parameter("goal_flood_threshold").value
        self.boarder_seg_inside_offset = self.get_parameter(
            "boarder_seg_inside_offset"
        ).value
        self.boarder_seg_outside_offset = self.get_parameter(
            "boarder_seg_outside_offset"
        ).value
        self.boarder_seg_corner_distance = self.get_parameter(
            "boarder_seg_corner_distance"
        ).value
        self.goal_h_factor = self.get_parameter("goal_h_factor").value
        self.goal_o_factor = self.get_parameter("goal_o_factor").value
        self.goal_e_factor = self.get_parameter("goal_e_factor").value
        self.use_smoothing = self.get_parameter("use_smoothing").value
        self.smoothing_kernel = self.get_parameter("smoothing_kernel").value
        self.line_angle_threshold = self.get_parameter("line_angle_threshold").value
        self.line_position_threshold = self.get_parameter(
            "line_position_threshold"
        ).value
        self.min_edge_points = self.get_parameter("min_edge_points").value
        self.debug_view = self.get_parameter("debug_view").value
        self.show_image = self.get_parameter("show_image").value
        self.show_image_fps = self.get_parameter("show_image_fps").value
        self.enable_profiling = self.get_parameter("enable_profiling").value
        self.profiling_duration = self.get_parameter("profiling_duration").value
        self.profiling_top_functions = self.get_parameter(
            "profiling_top_functions"
        ).value

        # Compute flood seed points based on camera dimensions
        self.flood_seed = [
            (int(self.camera_width // 2) + 50, int(self.camera_height // 2) + 50),
            (int(self.camera_width // 2) + 50, int(self.camera_height // 2) - 50),
            (int(self.camera_width // 2) - 50, int(self.camera_height // 2) + 50),
            (int(self.camera_width // 2) - 50, int(self.camera_height // 2) - 50),
        ]

        # Publishers
        self.image_publisher = self.create_publisher(
            CompressedImage, self.board_image_topic, 10
        )
        self.goal_publisher = self.create_publisher(
            StampedPolygon, self.goal_positions_topic, 10
        )

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Timer for image acquisition (this is faster than the camera FPS to avoid frame drops)
        self.timer = self.create_timer(
            1.0 / (4.0 * self.camera_fps), self._timer_callback
        )

        # Timer for goal publishing
        self.goal_timer = self.create_timer(1.0, self._publish_goal_positions)

        # Camera setup
        self._setup_camera()

        # Load camera calibration
        mtx, dist = load_calibration_data(self.calibration_file)
        newcameramtx, _ = cv2.getOptimalNewCameraMatrix(
            mtx,
            dist,
            (self.camera_width, self.camera_height),
            0,
            (self.camera_width, self.camera_height),
        )
        self.mapx, self.mapy = cv2.initUndistortRectifyMap(
            mtx,
            dist,
            None,
            newcameramtx,
            (self.camera_width, self.camera_height),
            5,
        )

        # Perspective transform
        self.width = 0
        self.height = 0

        # Border line tracking for outlier detection
        self.previous_boarder_lines: list[np.ndarray] | None = None

        # Goal positions
        self.left_goal: list[float] | None = None
        self.right_goal: list[float] | None = None

        # Image display tracking
        if self.show_image:
            self.last_display_time = 0.0
            self.frame_count = 0
            self.fps_display = 0.0
            self.fps_timer = self.create_timer(1.0, self._update_fps_display)

        # Profiling setup
        self.profiler = None
        if self.enable_profiling:
            self.profiler = cProfile.Profile()
            self.profiler.enable()
            self.get_logger().info(
                f"cProfile profiling enabled for {self.profiling_duration} seconds"
            )
            # Create one-shot timer to stop profiling after duration
            self.profiling_timer = self.create_timer(
                self.profiling_duration, self._stop_profiling
            )

    def _setup_camera(self) -> None:
        """Initialize and configure the camera."""
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera")
            return

        # Configure camera properties
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("U", "Y", "V", "Y"))
        self.cap.set(cv2.CAP_PROP_FPS, self.camera_fps)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, 0.01)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f"Camera configured: Target {self.camera_fps} FPS, Actual {actual_fps} FPS"
        )

    def initial_board_detection(self) -> None:
        """Perform initial board detection using flood fill to establish perspective transform."""

        # Wait for a valid frame from the camera
        ret, frame = self.cap.read()
        while not ret:
            rclpy.spin_once(self, timeout_sec=0.1)
            ret, frame = self.cap.read()

        # Skip a few frames to allow camera auto-adjustments
        for i in range(5):
            ret, frame = self.cap.read()

        # Undistort frame
        frame_rec = cv2.remap(frame, self.mapx, self.mapy, cv2.INTER_LINEAR)

        # Convert to HSV and split channels
        frame_hsv = cv2.cvtColor(frame_rec, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(frame_hsv)

        # Apply smoothing to saturation channel for more stable border detection
        if self.use_smoothing:
            s_smooth = cv2.GaussianBlur(
                s, (self.smoothing_kernel, self.smoothing_kernel), 0
            )
        else:
            s_smooth = s

        # Perform flood fill to detect board boundaries
        flood_mask, rect = self._board_flood_fill(
            s_smooth, self.flood_seed, self.flood_threshold
        )

        # Find rotated rectangle from flood fill mask
        rotated_rect = self._find_rotated_rect_from_flood_mask(flood_mask, rect)

        # Compute border segment masks and seed lines
        (
            self.boarder_segment_masks,
            seed_lines,
            self.rotation_matrix,
            self.segment_offsets,
            self.segment_lengths,
        ) = self._compute_boarder_segment_masks(rotated_rect, s.shape)

        # Compute seed line samples
        self.line_samples = self._compute_seed_line_samples(seed_lines, 4)

        # Precompute static affine transformations for border segment processing
        self._precompute_segment_transforms(
            self.rotation_matrix,
            self.segment_offsets,
            self.segment_lengths,
        )

        # Fit border segment lines and get board corners
        (
            (
                boarder_segment_flood_masks,
                boarder_segment_flood_masks_aligned,
                boarder_segment_edge_points,
                boarder_segment_edge_lines,
            ),
            board_corners,
            self.width,
            self.height,
        ) = self._fit_boarder_segment_lines(s_smooth)

        # Compute perspective transform from fitted line intersections
        M = self._compute_perspective_transform_from_corners(
            board_corners, self.width, self.height
        )

        # Apply the transformation
        warped_frame = cv2.warpPerspective(
            frame_rec,
            M,
            (self.width, self.height),
        )
        warped_v_channel = cv2.warpPerspective(
            v,
            M,
            (self.width, self.height),
        )

        self.left_goal, self.right_goal = self._find_goal_positions(
            warped_v_channel,
            self.goal_h_factor,
            self.goal_o_factor,
            self.goal_e_factor,
        )

        # Display debug views if enabled
        if self.debug_view:
            print_segment_debug_view(
                frame_rec,
                s_smooth,
                self.boarder_segment_masks,
                seed_lines,
                self.line_samples,
                boarder_segment_flood_masks,
                boarder_segment_flood_masks_aligned,
                boarder_segment_edge_points,
                boarder_segment_edge_lines,
                warped_frame,
            )
            print_initial_debug_view(
                frame_rec,
                h,
                s_smooth,
                v,
                self.flood_seed,
                flood_mask,
                rotated_rect,
                warped_v_channel,
                (self.left_goal, self.right_goal),
            )
            cv2.waitKey(0)

    def _find_goal_positions(
        self,
        warped_v_channel: np.ndarray,
        h_factor: float,
        o_factor: float,
        e_factor: float,
    ) -> tuple[cv2.RotatedRect, cv2.RotatedRect]:
        """Find goal positions in the warped V channel image.

        Args:
            warped_v_channel: The perspective-corrected V channel image.
            h_factor: Horizontal factor for goal seed positioning.
            o_factor: Offset factor for goal seed positioning.
            e_factor: Ellipse scaling factor for goal size.
        """

        offset_combinations = list(
            itertools.product([-o_factor, o_factor], [-o_factor, o_factor])
        )
        goal_seed_centers = (
            (int(self.width * h_factor), int(self.height * 0.5)),
            (int(self.width * (1 - h_factor)), int(self.height * 0.5)),
        )

        goals = []

        for goal_seed_center in goal_seed_centers:
            goal_seed = [
                (
                    int(goal_seed_center[0] + self.width * offset[0]),
                    int(goal_seed_center[1] + self.height * offset[1]),
                )
                for offset in offset_combinations
            ]

            goal_mask, rect = self._board_flood_fill(
                warped_v_channel, goal_seed, self.goal_flood_threshold
            )

            # Get all points where flood_mask is non-zero
            points = cv2.findNonZero(goal_mask)
            goal_ellipse = cv2.fitEllipse(points)

            # Scale ellipse size by e_factor
            center, axes, angle = goal_ellipse
            goal_ellipse = (center, (axes[0] * e_factor, axes[1] * e_factor), angle)

            goals.append(goal_ellipse)

            if self.debug_view:
                cv2.ellipse(warped_v_channel, goal_ellipse, (0, 255, 0), 2)
                for x, y in goal_seed:
                    cv2.circle(
                        warped_v_channel,
                        (x, y),
                        3,
                        (0, 255, 0),
                        -1,
                    )

        return tuple(goals)

    def _precompute_segment_transforms(
        self,
        rotation_matrix: np.ndarray,
        segment_offsets: list[np.ndarray],
        segment_lengths: list[int],
    ) -> None:
        """Precompute static affine transformations for border segment processing.

        These transformations remain constant across all frames and only depend on
        the initial board detection results.
        """
        # Define rotation matrices for each segment (constant)
        self.diff_rotations = (
            -np.eye(2),
            np.array([[0, -1], [1, 0]]),
            np.eye(2),
            np.array([[0, 1], [-1, 0]]),
        )

        # Aliasing crop size
        self.aliasing_crop_size = 2

        # Precompute affine matrices and inverse affine matrices for each segment
        self.affine_matrices = []
        self.inverse_affine_matrices = []
        self.warp_sizes = []

        for segment_offset, length, diff_rotation in zip(
            segment_offsets, segment_lengths, self.diff_rotations
        ):
            # Compute offset for affine transform
            offset = -diff_rotation @ rotation_matrix.T @ segment_offset.reshape(
                (2, 1)
            ) + np.array([[int(length // 2)], [self.boarder_seg_inside_offset]])

            # Compute affine matrix
            affine_matrix = np.hstack([diff_rotation @ rotation_matrix.T, offset])
            self.affine_matrices.append(affine_matrix)

            # Compute inverse affine matrix for transforming edge points back
            rotation_part = diff_rotation @ rotation_matrix.T
            offset_part = offset + np.array([[1], [1]])
            inverse_rotation = rotation_part.T
            inverse_offset = -inverse_rotation @ offset_part
            inverse_affine = np.hstack([inverse_rotation, inverse_offset])
            self.inverse_affine_matrices.append(inverse_affine)

            # Store warp size
            warp_size = (
                length,
                self.boarder_seg_inside_offset + self.boarder_seg_outside_offset,
            )
            self.warp_sizes.append(warp_size)

    def _fit_boarder_segment_lines(
        self,
        channel: np.ndarray,
    ) -> np.ndarray:
        """Fit lines to border segments using flood fill and edge detection.

        Args:
            channel: Single channel image (typically V channel from HSV).

        Returns:
            4x4 array of corner points in (x, y) format for perspective transform.
        """

        # Lists to store results
        boarder_segment_flood_masks = []
        boarder_segment_flood_masks_aligned = []
        boarder_segment_edge_points = []
        boarder_segment_edge_lines = []

        # Process each border segment using precomputed transformations
        for (
            boarder_segment_mask,
            line_sample_points,
            affine_matrix,
            inverse_affine,
            warp_size,
        ) in zip(
            self.boarder_segment_masks,
            self.line_samples,
            self.affine_matrices,
            self.inverse_affine_matrices,
            self.warp_sizes,
        ):
            # Mask the channel to the current border segment
            masked_image = cv2.bitwise_and(channel, channel, mask=boarder_segment_mask)

            # Perform flood fill on the masked image with the seed line sample points
            boarder_segment_flood_mask, _ = self._board_flood_fill(
                masked_image, line_sample_points, self.flood_threshold
            )
            boarder_segment_flood_masks.append(boarder_segment_flood_mask)

            # Align the flood mask using precomputed affine transform
            aligned_mask = cv2.warpAffine(
                boarder_segment_flood_mask,
                affine_matrix,
                warp_size,
            )
            # Because the affine transform can introduce edge artifacts, we crop a few pixels
            aligned_mask = aligned_mask[
                self.aliasing_crop_size : -self.aliasing_crop_size,
                self.aliasing_crop_size : -self.aliasing_crop_size,
            ]
            boarder_segment_flood_masks_aligned.append(aligned_mask)

            # Detect falling edges (255 -> 0) in y direction
            diff_y = np.diff(aligned_mask.astype(np.int16), axis=0)
            falling_edges = diff_y < -100

            # Get the first falling edge row index for each column
            first_edge_rows = np.argmax(falling_edges, axis=0)

            # Filter out columns with no edges
            has_edge = falling_edges[first_edge_rows, np.arange(falling_edges.shape[1])]

            # Create edge points array
            edge_points = np.column_stack(
                [
                    np.arange(falling_edges.shape[1]),
                    first_edge_rows,
                ]
            )
            edge_points = edge_points[has_edge] + np.array(
                [self.aliasing_crop_size, self.aliasing_crop_size]
            )

            # Transform edge points back to original frame using precomputed inverse affine
            edge_points_homogeneous = np.hstack(
                [edge_points, np.ones((len(edge_points), 1))]
            )
            edge_points_original = edge_points_homogeneous @ inverse_affine.T
            boarder_segment_edge_points.append(edge_points_original)

            # Fit line to edge points with validation
            segment_idx = len(boarder_segment_edge_lines)
            fitted_line = self._fit_and_validate_line(edge_points_original, segment_idx)
            boarder_segment_edge_lines.append(fitted_line)

        # Compute intersection points of fitted lines to get board corners
        # Vectorize the intersection computation for all 4 corners
        lines = np.roll(
            np.array(boarder_segment_edge_lines), 1, axis=0
        )  # Shape: (4, 4) - [vx, vy, x0, y0] for each line

        # Get current and next lines (with wrapping)
        vx1 = lines[:, 0].flatten()
        vy1 = lines[:, 1].flatten()
        x01 = lines[:, 2].flatten()
        y01 = lines[:, 3].flatten()

        # Shift by one to get next lines (wrapping around)
        vx2 = np.roll(vx1, -1)
        vy2 = np.roll(vy1, -1)
        x02 = np.roll(x01, -1)
        y02 = np.roll(y01, -1)

        # Compute intersection using vectorized analytical formula
        # t1 = (dx*vy2 - dy*vx2) / (vx1*vy2 - vy1*vx2)
        dx = x02 - x01
        dy = y02 - y01
        denominator = vx1 * vy2 - vy1 * vx2
        t1 = (dx * vy2 - dy * vx2) / denominator

        # Compute intersection points
        board_corners = np.column_stack([x01 + t1 * vx1, y01 + t1 * vy1]).astype(
            np.float32
        )

        # Compute board width and height from corner points
        edge_lengths = np.linalg.norm(
            board_corners - np.roll(board_corners, 1, axis=0), axis=1
        )
        width = int((edge_lengths[1] + edge_lengths[3]) / 2)
        height = int((edge_lengths[0] + edge_lengths[2]) / 2)

        # Store validated lines for next iteration
        self.previous_boarder_lines = boarder_segment_edge_lines.copy()

        return (
            (
                boarder_segment_flood_masks,
                boarder_segment_flood_masks_aligned,
                boarder_segment_edge_points,
                boarder_segment_edge_lines,
            ),
            board_corners,
            width,
            height,
        )

    def _fit_and_validate_line(
        self, edge_points: np.ndarray, segment_idx: int
    ) -> list[float]:
        """Fit a line to edge points with outlier detection and validation.

        Args:
            edge_points: Array of edge points (Nx2)
            segment_idx: Index of the border segment (0-3)

        Returns:
            Line parameters [vx, vy, x0, y0]
        """
        # Check if we have enough points to fit a line
        if len(edge_points) < self.min_edge_points:
            if self.previous_boarder_lines is not None:
                # Not enough points, use previous line
                return self.previous_boarder_lines[segment_idx]
            else:
                # No previous line available, return a default horizontal line
                return [
                    np.array([1.0]),
                    np.array([0.0]),
                    np.array([0.0]),
                    np.array([0.0]),
                ]

        # Fit line to edge points
        [vx, vy, x0, y0] = cv2.fitLine(edge_points, cv2.DIST_HUBER, 0, 0.01, 0.01)
        current_line = [vx, vy, x0, y0]

        # If no previous line, accept current fit
        if self.previous_boarder_lines is None:
            return current_line

        # Validate against previous line
        previous_line = self.previous_boarder_lines[segment_idx]

        # Extract direction vectors and positions
        prev_vx, prev_vy, prev_x0, prev_y0 = previous_line

        # Compute angle difference between lines
        # Normalize direction vectors
        prev_dir = np.array([prev_vx, prev_vy]).flatten()
        curr_dir = np.array([vx, vy]).flatten()
        prev_dir = prev_dir / np.linalg.norm(prev_dir)
        curr_dir = curr_dir / np.linalg.norm(curr_dir)

        # Compute angle using dot product (handle both parallel and anti-parallel)
        dot_product = np.abs(np.dot(prev_dir, curr_dir))
        dot_product = np.clip(dot_product, -1.0, 1.0)
        angle_diff = np.rad2deg(np.arccos(dot_product))

        # Compute position difference (perpendicular distance between lines)
        # Distance from point (prev_x0, prev_y0) to current line
        point_to_curr = np.array([prev_x0 - x0, prev_y0 - y0]).flatten()
        # Project onto perpendicular direction (rotate direction by 90 degrees)
        perp_dir = np.array([-curr_dir[1], curr_dir[0]])
        position_diff = np.abs(np.dot(point_to_curr, perp_dir))

        # Check if differences exceed thresholds
        if (
            angle_diff > self.line_angle_threshold
            or position_diff > self.line_position_threshold
        ):
            # Outlier detected, use previous line
            return previous_line

        # Valid fit, return current line
        return current_line

    def _compute_seed_line_samples(
        self, seed_lines: list[np.ndarray], line_sample_count: int
    ) -> list[list[np.ndarray]]:
        """Compute sample points along each seed line for flood fill seeding.

        Args:
            seed_lines: List of seed line endpoints for each segment.
            line_sample_count: Number of sample points per line.

        Returns:
            List of sample point lists, one per segment.
        """

        line_samples = []
        for seed_line in seed_lines:
            # Move the seed sample line slightly inward to avoid edge artifacts
            line_vec = seed_line[:, 1] - seed_line[:, 0]
            line_length = np.linalg.norm(line_vec)
            dir = line_vec / line_length
            # Sample points along the line
            interval = line_length / (line_sample_count + 1)
            sample_points = [
                np.intp(seed_line[:, 0] + dir * (interval * (j + 1)))
                for j in range(line_sample_count)
            ]
            line_samples.append(sample_points)
        return line_samples

    def _compute_boarder_segment_masks(
        self, rotated_rect, shape: tuple[int, int]
    ) -> tuple[
        list[np.ndarray],
        list[np.ndarray],
        np.ndarray,
        list[np.ndarray],
        tuple[int, int, int, int],
    ]:
        """Compute masks for each border segment of the game board.

        Args:
            rotated_rect: Detected rotated rectangle representing the board.
            shape: Image shape (height, width).

        Returns:
            Tuple containing:
                - List of border segment masks
                - List of seed lines for each segment
                - Rotation matrix
                - List of segment offsets
                - Segment lengths tuple (top, right, bottom, left)
        """
        # Create boarder masks
        corner_pts, rect_width, rect_height, rotation_matrix = (
            self._corners_from_rotated_rect(rotated_rect)
        )
        boarder_segment_masks = [np.zeros(shape, dtype=np.uint8) for _ in range(4)]

        # Compute segment dimensions
        long_side_length = rect_width - 2 * self.boarder_seg_corner_distance
        short_side_length = rect_height - 2 * self.boarder_seg_corner_distance
        center = np.reshape(rotated_rect[0], (2, 1))

        # Helper to create rectangle points from offsets
        def rect_from_offset(
            top_offset, bottom_offset, left_offset, right_offset
        ) -> np.ndarray:
            return np.array(
                [
                    [-left_offset, top_offset],
                    [right_offset, top_offset],
                    [right_offset, -bottom_offset],
                    [-left_offset, -bottom_offset],
                ]
            ).T

        # Define parameter combinations for each border segment
        parameter_combination = [
            (
                self.boarder_seg_inside_offset,
                self.boarder_seg_outside_offset,
                int(long_side_length // 2),
                int(long_side_length // 2),
            ),
            (
                int(short_side_length // 2),
                int(short_side_length // 2),
                self.boarder_seg_inside_offset,
                self.boarder_seg_outside_offset,
            ),
            (
                self.boarder_seg_outside_offset,
                self.boarder_seg_inside_offset,
                int(long_side_length // 2),
                int(long_side_length // 2),
            ),
            (
                int(short_side_length // 2),
                int(short_side_length // 2),
                self.boarder_seg_outside_offset,
                self.boarder_seg_inside_offset,
            ),
        ]
        # Lengths of each border segment rectangle
        rects_length = (
            int(long_side_length),
            int(short_side_length),
            int(long_side_length),
            int(short_side_length),
        )

        # Compute rectangles
        rects = [rect_from_offset(*params) for params in parameter_combination]
        rects_transformed = [rotation_matrix @ rect + center for rect in rects]
        seed_lines = []
        segment_offsets = []

        # Fill in masks and compute seed lines
        for i, boarder_segment_mask in enumerate(boarder_segment_masks):
            # Get corner points for this segment
            pt1 = np.reshape(corner_pts[i], (2, 1))
            pt2 = np.reshape(corner_pts[(i + 1) % 4], (2, 1))
            # Compute the center point of the line segment
            line_center = (pt2 - pt1) / 2 + pt1
            segment_offsets.append(line_center)
            # Compute the rectangle points
            parallel_dir = line_center - center
            rect_pts = np.intp(rects_transformed[i] + parallel_dir)
            # Compute seed line points slightly offset from the rectangle edge
            seed_line_pts = np.hstack(
                (
                    rect_pts[:, (1 - i) % 4].reshape(2, 1),
                    rect_pts[:, (4 - i) % 4].reshape(2, 1),
                )
            )
            seed_offset = np.intp(parallel_dir / np.linalg.norm(parallel_dir) * 10)
            seed_line_pts += seed_offset
            seed_lines.append(np.intp(seed_line_pts).astype(np.int32))

            # Transpose from 2xN to Nx2 and convert to int32 for cv2
            rect_pts = rect_pts.T

            # Fill the polygon mask
            cv2.fillConvexPoly(boarder_segment_mask, rect_pts, 255)

        return (
            boarder_segment_masks,
            seed_lines,
            rotation_matrix,
            segment_offsets,
            rects_length,
        )

    def _board_flood_fill(
        self,
        channel: np.ndarray,
        seed: tuple[int, int] | list[tuple[int, int]],
        threshold: int,
    ) -> tuple[np.ndarray, cv2.typing.Rect]:
        """
        Perform flood fill on the given channel starting from the seed point(s).

        Args:
            channel (np.ndarray): The image channel to perform flood fill on.
            seed (tuple[int, int] | list[tuple[int, int]]): The seed point(s) for flood fill.
            threshold (int): Threshold relative to seed pixel value (loDiff=upDiff=threshold).
        Returns:
            flood_mask (np.ndarray): The resulting flood fill mask.
            rect (cv2.typing.Rect): The bounding rectangle of the flooded area.
        """
        # flags: connectivity (4) + fill mask only + fixed range (relative to seed)
        # FLOODFILL_FIXED_RANGE makes loDiff/upDiff relative to seed pixel, not neighbors
        flags = 4 | (255 << 8) | cv2.FLOODFILL_MASK_ONLY | cv2.FLOODFILL_FIXED_RANGE

        if isinstance(seed, list) is False:
            seed = [seed]

        flood_mask = None
        for single_seed in seed:
            sane_seed = (max(0, single_seed[0]), max(0, single_seed[1]))

            _, _, flood_mask, rect = cv2.floodFill(
                channel,
                flood_mask,
                tuple(sane_seed),
                255,
                loDiff=threshold,
                upDiff=threshold,
                flags=flags,
            )

        # Get all points where flood_mask is non-zero
        flood_mask = flood_mask[1:-1, 1:-1]

        return flood_mask, rect

    def _find_rotated_rect_from_flood_mask(
        self, flood_mask: np.ndarray, rect: cv2.typing.Rect
    ) -> cv2.RotatedRect:
        """Find rotated rectangle from flood fill mask.

        Args:
            flood_mask: Binary mask from flood fill operation.
            rect: Bounding rectangle from flood fill.

        Returns:
            Minimum area rotated rectangle fitting the mask.
        """

        # Get all points where flood_mask is non-zero
        points = cv2.findNonZero(flood_mask)

        if points is not None and len(points) > 0:
            # Get minimum area rectangle (rotated rectangle) directly from points
            rotated_rect = cv2.minAreaRect(points)
        else:
            # Fallback to axis-aligned rectangle if no points found
            x, y, w, h = rect
            rotated_rect = cv2.RotatedRect((x + w / 2, y + h / 2), (w, h), 0.0)

        return rotated_rect

    def _corners_from_rotated_rect(
        self, rotated_rect: cv2.RotatedRect
    ) -> tuple[np.ndarray, float, float, np.ndarray]:
        """Extract corner points and parameters from rotated rectangle.

        Args:
            rotated_rect: OpenCV rotated rectangle object.

        Returns:
            Tuple containing:
                - Corner points as 2x4 array (x, y rows)
                - Rectangle width
                - Rectangle height
                - 2x2 rotation matrix
        """
        # rotated_rect format: ((center_x, center_y), (width, height), angle)
        center, (rect_width, rect_height), angle = rotated_rect

        if rect_height > rect_width:
            rect_width, rect_height = rect_height, rect_width
            angle -= 90.0

        # Create corner points of axis-aligned rectangle centered at origin
        # Order: [top-left, top-right, bottom-right, bottom-left]
        half_w = rect_width / 2.0
        half_h = rect_height / 2.0
        corners = np.array(
            [
                [-half_w, -half_h],  # top-left
                [half_w, -half_h],  # top-right
                [half_w, half_h],  # bottom-right
                [-half_w, half_h],  # bottom-left
            ],
            dtype=np.float32,
        )

        # Create rotation matrix
        angle_rad = np.deg2rad(angle)
        cos_a = np.cos(angle_rad)
        sin_a = np.sin(angle_rad)
        rotation_matrix = np.array([[cos_a, -sin_a], [sin_a, cos_a]], dtype=np.float32)

        # Rotate corners
        rotated_corners = corners @ rotation_matrix.T
        rotated_corners += np.array(center, dtype=np.float32)

        # Translate to actual center position
        return (rotated_corners, rect_width, rect_height, rotation_matrix)

    def _compute_perspective_transform_from_corners(
        self, corners: np.ndarray, width: int, height: int
    ) -> None:
        """Compute perspective transformation matrix from board corners."""

        # Destination points (perfect rectangle)
        dst_pts = np.array(
            [
                [0, 0],
                [width, 0],
                [width, height],
                [0, height],
            ],
            dtype="float32",
        )

        # Compute transformation matrix
        return cv2.getPerspectiveTransform(corners, dst_pts)

    def _update_fps_display(self) -> None:
        """Update FPS display value every second."""
        self.fps_display = self.frame_count
        self.frame_count = 0

    def _stop_profiling(self) -> None:
        """Stop profiling and print stats (called once by timer)."""
        if self.profiler is not None:
            self.profiler.disable()
            self.get_logger().info("Profiling complete. Generating stats...")
            print_profiling_stats(self.profiler, self.profiling_top_functions)
            self.profiler = None
            # Cancel the timer so it doesn't fire again
            self.profiling_timer.cancel()
            self.profiling_timer = None

    def _timer_callback(self) -> None:
        """Main timer callback for processing camera frames."""
        # Capture and validate frame
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to grab frame")
            return

        # Undistort frame using calibration data
        rectified_frame = cv2.remap(frame, self.mapx, self.mapy, cv2.INTER_LINEAR)

        # Process frame
        self._process_frame(rectified_frame)

    def _process_frame(self, undistorted_frame: np.ndarray) -> None:
        """Process frame to create perspective-corrected view and publish."""

        # Convert to HSV and split channels
        frame_hsv = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(frame_hsv)

        # Apply smoothing to saturation channel for more stable border detection
        if self.use_smoothing:
            s_smooth = cv2.GaussianBlur(
                s, (self.smoothing_kernel, self.smoothing_kernel), 0
            )
        else:
            s_smooth = s

        (
            (
                _,
                _,
                boarder_segment_edge_points,
                boarder_segment_edge_lines,
            ),
            board_corners,
            _,
            _,
        ) = self._fit_boarder_segment_lines(s_smooth)

        # Compute perspective transform from fitted line intersections
        M = self._compute_perspective_transform_from_corners(
            board_corners, self.width, self.height
        )

        # Apply perspective warp to get top-down view
        warped = cv2.warpPerspective(
            undistorted_frame,
            M,
            (self.width, self.height),
        )

        # Display image if enabled and enough time has elapsed
        if self.show_image:

            # Increment frame counter
            self.frame_count += 1

            current_time = time.time()
            if current_time - self.last_display_time >= 1.0 / self.show_image_fps:
                self.last_display_time = current_time

                show_final_output(
                    warped,
                    self.fps_display,
                    "Board View",
                    goal_ellipses=(self.left_goal, self.right_goal),
                )
                show_online_boarders(
                    undistorted_frame,
                    boarder_segment_edge_points,
                    boarder_segment_edge_lines,
                    self.fps_display,
                    "Edge Points and Fitted Lines",
                )

        # Publish the transformed image
        self._publish_board_image(warped)

    def _publish_board_image(self, image: np.ndarray) -> None:
        """Publish the transformed and cropped board image as a compressed ROS2 message."""
        try:
            # Convert OpenCV image to ROS CompressedImage message
            msg = self.bridge.cv2_to_compressed_imgmsg(image, dst_format="jpg")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"
            self.image_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish board image: {e}")

    def _publish_goal_positions(self) -> None:
        """Publish goal positions."""
        polygon = Polygon()

        if self.left_goal is not None:
            point = Point32()
            point.x = float(self.left_goal[0][0])
            point.y = float(self.left_goal[0][1])
            point.z = 0.0
            polygon.points.append(point)

        if self.right_goal is not None:
            point = Point32()
            point.x = float(self.right_goal[0][0])
            point.y = float(self.right_goal[0][1])
            point.z = 0.0
            polygon.points.append(point)

        if len(polygon.points) == 2:
            stamped_polygon = StampedPolygon()
            stamped_polygon.polygon = polygon
            stamped_polygon.header.stamp = self.get_clock().now().to_msg()
            self.goal_publisher.publish(stamped_polygon)


def main(args=None):
    rclpy.init(args=args)

    camera_node = CameraNode()
    camera_node.initial_board_detection()

    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
