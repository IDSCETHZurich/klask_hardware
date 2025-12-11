"""ROS2 node for camera image acquisition, AprilTag detection, and perspective transformation."""

import cv2
import time
import rclpy
import numpy as np
import cProfile
import os
from pathlib import Path
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
)


class CameraNode(Node):
    """ROS2 node for acquiring camera images and publishing transformed board view."""

    # Constants
    CAMERA_FPS = 120
    CAMERA_WIDTH = 1280
    CAMERA_HEIGHT = 720
    FLOOD_SEED = [
        (int(CAMERA_WIDTH // 2) + 50, int(CAMERA_HEIGHT // 2) + 50),
        (int(CAMERA_WIDTH // 2) + 50, int(CAMERA_HEIGHT // 2) - 50),
        (int(CAMERA_WIDTH // 2) - 50, int(CAMERA_HEIGHT // 2) + 50),
        (int(CAMERA_WIDTH // 2) - 50, int(CAMERA_HEIGHT // 2) - 50),
    ]
    FLOOD_THRESHOLD = 7
    BOARDER_SEG_INSIDE_OFFSET = 50
    BOARDER_SEG_OUTSIDE_OFFSET = 20
    BOARDER_SEG_CORNER_DISTANCE = 100

    # Initial board detection settings
    USE_STORED_IMAGE = False  # Set to True to load image from data/ folder
    STORED_IMAGE_FILENAME = "debug_board_tilted_1.jpg"  # Filename in data/ folder

    # Debug/Display settings
    DEBUG_VIEW = False  # Set to True to show debug views during processing
    SHOW_IMAGE = True  # Set to True to display final output image
    SHOW_IMAGE_FPS = 10  # Display update frequency in Hz

    # Profiling settings
    ENABLE_PROFILING = False
    PROFILING_DURATION = 100.0  # Run profiler for N seconds
    PROFILING_TOP_FUNCTIONS = 30  # Show top N functions in stats

    def __init__(self):
        super().__init__("camera_node")

        # Publishers
        self.image_publisher = self.create_publisher(
            CompressedImage, "board_image/compressed", 10
        )
        self.goal_publisher = self.create_publisher(
            StampedPolygon, "goal_positions", 10
        )

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Timer for image acquisition
        self.timer = self.create_timer(1.0 / 480.0, self.timer_callback)

        # Camera setup
        self._setup_camera()

        # Load camera calibration
        mtx, dist = load_calibration_data("calibration_data.npz")
        newcameramtx, _ = cv2.getOptimalNewCameraMatrix(
            mtx,
            dist,
            (self.CAMERA_WIDTH, self.CAMERA_HEIGHT),
            0,
            (self.CAMERA_WIDTH, self.CAMERA_HEIGHT),
        )
        self.mapx, self.mapy = cv2.initUndistortRectifyMap(
            mtx,
            dist,
            None,
            newcameramtx,
            (self.CAMERA_WIDTH, self.CAMERA_HEIGHT),
            5,
        )

        # Perspective transform
        self.M: np.ndarray | None = None
        self.width = 0
        self.height = 0

        # Goal positions
        self.left_goal: list[float] | None = None
        self.right_goal: list[float] | None = None

        # Image display tracking
        if self.SHOW_IMAGE:
            self.last_display_time = 0.0
            self.frame_count = 0
            self.fps_display = 0.0
            self.fps_timer = self.create_timer(1.0, self.update_fps_display)

        # Profiling setup
        self.profiler = None
        self.profiling_start_time = None
        if self.ENABLE_PROFILING:
            self.profiler = cProfile.Profile()
            self.profiler.enable()
            self.profiling_start_time = time.time()
            self.get_logger().info(
                f"cProfile profiling enabled for {self.PROFILING_DURATION} seconds"
            )

    def _setup_camera(self) -> None:
        """Initialize and configure the camera."""
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera")
            return

        # Configure camera properties
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("U", "Y", "V", "Y"))
        self.cap.set(cv2.CAP_PROP_FPS, self.CAMERA_FPS)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, 0.01)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.CAMERA_HEIGHT)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f"Camera configured: Target {self.CAMERA_FPS} FPS, Actual {actual_fps} FPS"
        )

    def initial_board_detection(self) -> None:
        """Perform initial board detection using flood fill to establish perspective transform."""

        # TODO: Remove this after the debuging is done
        if self.USE_STORED_IMAGE:
            # Load stored image
            package_dir = Path(__file__).resolve().parent
            image_path = os.path.join(package_dir, "data", self.STORED_IMAGE_FILENAME)

            self.get_logger().info(f"Loading stored image from: {image_path}")
            frame = cv2.imread(image_path)

            if frame is None:
                self.get_logger().error(
                    f"Failed to load stored image from {image_path}, falling back to camera"
                )
            else:
                self.get_logger().info(
                    f"Successfully loaded stored image: {frame.shape}"
                )
        else:
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

        # Perform flood fill to detect board boundaries
        flood_mask, rect = self._board_flood_fill(
            s, self.FLOOD_SEED, self.FLOOD_THRESHOLD
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
        ) = self._fit_boarder_segment_lines(s)

        # Compute perspective transform from fitted line intersections
        self._compute_perspective_transform_from_corners(board_corners)

        # Apply the transformation and display the result
        warped_from_corners = cv2.warpPerspective(
            frame_rec,
            self.M,
            (self.width, self.height),
        )

        # Display debug views if enabled
        if self.DEBUG_VIEW:
            print_segment_debug_view(
                frame_rec,
                s,
                self.boarder_segment_masks,
                seed_lines,
                self.line_samples,
                boarder_segment_flood_masks,
                boarder_segment_flood_masks_aligned,
                boarder_segment_edge_points,
                boarder_segment_edge_lines,
                warped_from_corners,
            )
            print_initial_debug_view(
                frame_rec,
                h,
                s,
                v,
                self.FLOOD_SEED,
                flood_mask,
                rotated_rect,
            )
            cv2.waitKey(0)

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

        # Aliasing crop size (constant)
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
            ) + np.array([[int(length // 2)], [self.BOARDER_SEG_INSIDE_OFFSET]])

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
                self.BOARDER_SEG_INSIDE_OFFSET + self.BOARDER_SEG_OUTSIDE_OFFSET,
            )
            self.warp_sizes.append(warp_size)

    def _fit_boarder_segment_lines(
        self,
        channel: np.ndarray,
    ) -> np.ndarray:

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
                masked_image, line_sample_points, self.FLOOD_THRESHOLD
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

            # Fit line to edge points
            [vx, vy, x0, y0] = cv2.fitLine(
                edge_points_original, cv2.DIST_HUBER, 0, 0.01, 0.01
            )

            # If too slow try this instead:
            # [vx, vy, x0, y0] = cv2.fitLine(edge_points_original, cv2.DIST_L2, 0, 0.01, 0.01)

            boarder_segment_edge_lines.append([vx, vy, x0, y0])

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

        # TODO: Return only board_corners and remove other return values
        return (
            (
                boarder_segment_flood_masks,
                boarder_segment_flood_masks_aligned,
                boarder_segment_edge_points,
                boarder_segment_edge_lines,
            ),
            board_corners,
        )

    def _compute_seed_line_samples(
        self, seed_lines: list[np.ndarray], line_sample_count: int
    ) -> list[list[np.ndarray]]:
        """Compute sample points along each seed line for flood fill seeding."""

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
        # Create boarder masks
        corner_pts, rect_width, rect_height, rotation_matrix = (
            self._corners_from_rotated_rect(rotated_rect)
        )
        boarder_segment_masks = [np.zeros(shape, dtype=np.uint8) for _ in range(4)]

        # Compute segment dimensions
        long_side_length = rect_width - 2 * self.BOARDER_SEG_CORNER_DISTANCE
        short_side_length = rect_height - 2 * self.BOARDER_SEG_CORNER_DISTANCE
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
                self.BOARDER_SEG_INSIDE_OFFSET,
                self.BOARDER_SEG_OUTSIDE_OFFSET,
                int(long_side_length // 2),
                int(long_side_length // 2),
            ),
            (
                int(short_side_length // 2),
                int(short_side_length // 2),
                self.BOARDER_SEG_INSIDE_OFFSET,
                self.BOARDER_SEG_OUTSIDE_OFFSET,
            ),
            (
                self.BOARDER_SEG_OUTSIDE_OFFSET,
                self.BOARDER_SEG_INSIDE_OFFSET,
                int(long_side_length // 2),
                int(long_side_length // 2),
            ),
            (
                int(short_side_length // 2),
                int(short_side_length // 2),
                self.BOARDER_SEG_OUTSIDE_OFFSET,
                self.BOARDER_SEG_INSIDE_OFFSET,
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
            threshold (int): The threshold for flood fill.
        Returns:
            flood_mask (np.ndarray): The resulting flood fill mask.
            rect (cv2.typing.Rect): The bounding rectangle of the flooded area.
        """
        # Perform flood fill
        # flags: connectivity (4 or 8) + fill mask only option
        flags = 4 | (255 << 8) | cv2.FLOODFILL_MASK_ONLY

        if isinstance(seed, list) is False:
            seed = [seed]

        flood_mask = None
        for single_seed in seed:
            _, _, flood_mask, rect = cv2.floodFill(
                channel,
                flood_mask,
                tuple(single_seed),
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
        """Find rotated rectangle from flood fill mask."""

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
        """Get corner points from rotated rectangle."""
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

    def _compute_perspective_transform_from_corners(self, corners: np.ndarray) -> None:
        """Compute perspective transformation matrix from board corners."""

        # corners should be in order: [top-left, top-right, bottom-right, bottom-left]
        # Calculate board dimensions from corners
        # TODO: Should we use a fixed size instead?
        top_width = np.linalg.norm(corners[1] - corners[0])
        bottom_width = np.linalg.norm(corners[2] - corners[3])
        left_height = np.linalg.norm(corners[3] - corners[0])
        right_height = np.linalg.norm(corners[2] - corners[1])

        # Use average dimensions
        self.width = int((top_width + bottom_width) / 2)
        self.height = int((left_height + right_height) / 2)

        # Destination points (perfect rectangle)
        dst_pts = np.array(
            [
                [0, 0],
                [self.width, 0],
                [self.width, self.height],
                [0, self.height],
            ],
            dtype="float32",
        )

        # Compute transformation matrix
        self.M = cv2.getPerspectiveTransform(corners, dst_pts)

    def update_fps_display(self) -> None:
        """Update FPS display value every second."""
        self.fps_display = self.frame_count
        self.frame_count = 0

    def timer_callback(self) -> None:
        """Main timer callback for processing camera frames."""
        # Capture and validate frame
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to grab frame")
            return

        # Undistort frame using calibration data
        rectified_frame = cv2.remap(frame, self.mapx, self.mapy, cv2.INTER_LINEAR)

        # Process frame
        self.process_frame(rectified_frame)

    def process_frame(self, undistorted_frame: np.ndarray) -> None:
        """Process frame to create perspective-corrected view and publish."""

        # Convert to HSV and split channels
        frame_hsv = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(frame_hsv)

        (_, board_corners) = self._fit_boarder_segment_lines(s)

        # Compute perspective transform from fitted line intersections
        self._compute_perspective_transform_from_corners(board_corners)

        # Apply perspective warp to get top-down view
        warped = cv2.warpPerspective(
            undistorted_frame,
            self.M,
            (self.width, self.height),
        )

        # Display image if enabled and enough time has elapsed
        if self.SHOW_IMAGE:

            # Increment frame counter
            self.frame_count += 1

            current_time = time.time()
            if current_time - self.last_display_time >= 1.0 / self.SHOW_IMAGE_FPS:
                self.last_display_time = current_time

                show_final_output(warped, self.fps_display)

        # Publish the transformed image
        self.publish_board_image(warped)

    def publish_board_image(self, image: np.ndarray) -> None:
        """Publish the transformed and cropped board image as a compressed ROS2 message."""
        try:
            # Convert OpenCV image to ROS CompressedImage message
            msg = self.bridge.cv2_to_compressed_imgmsg(image, dst_format="jpg")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"
            self.image_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish board image: {e}")

    def publish_goal_positions(self) -> None:
        """Publish goal positions."""
        polygon = Polygon()

        if self.left_goal is not None:
            point = Point32()
            point.x = float(self.left_goal[0])
            point.y = float(self.left_goal[1])
            point.z = 0.0
            polygon.points.append(point)

        if self.right_goal is not None:
            point = Point32()
            point.x = float(self.right_goal[0])
            point.y = float(self.right_goal[1])
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
