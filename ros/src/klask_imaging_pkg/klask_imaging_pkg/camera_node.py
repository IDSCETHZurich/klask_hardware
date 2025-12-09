"""ROS2 node for camera image acquisition, AprilTag detection, and perspective transformation."""

import cv2
import time
import rclpy
import numpy as np
import apriltag
import cProfile
import pstats
import io
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Polygon, Point32
from klask_interfaces.msg import StampedPolygon
from cv_bridge import CvBridge

from .utils import load_calibration_data, apply_ema_filter


class CameraNode(Node):
    """ROS2 node for acquiring camera images and publishing transformed board view."""

    # Constants
    CAMERA_FPS = 120
    CAMERA_WIDTH = 1280
    CAMERA_HEIGHT = 720
    APRILTAG_FAMILY = "tag36h11"
    FLOOD_SEED = (CAMERA_WIDTH // 2, CAMERA_HEIGHT // 2)
    FLOOD_THRESHOLD = 5

    # Debug/Display settings
    DEBUG_VIEW = True

    # Profiling settings
    ENABLE_PROFILING = False
    PROFILING_DURATION = 100.0  # Run profiler for N seconds
    PROFILING_TOP_FUNCTIONS = 30  # Show top N functions in stats

    # AprilTag IDs mapping
    TAG_NAMES = {
        0: "center right",
        1: "center left",
        6: "xy gantry",
        7: "x gantry",
    }

    # Goal detection constants
    GOAL_TAG_IDS = range(2)  # Tags 0-1 are goal tags

    # EMA filter alpha values
    EMA_ALPHA_APRILTAG = 0.1
    EMA_ALPHA_GANTRY = 1.0  # No smoothing for gantry tags

    # AprilTag detection rate
    APRILTAG_DETECTION_FPS = 5.0  # Detect AprilTags at this frequency (Hz)

    def __init__(self):
        super().__init__("camera_node")

        # Publishers
        self.image_publisher = self.create_publisher(
            CompressedImage, "board_image/compressed", 10
        )
        self.goal_publisher = self.create_publisher(
            StampedPolygon, "goal_positions", 10
        )
        self.gantry_publisher = self.create_publisher(
            StampedPolygon, "gantry_positions", 10
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

        # AprilTag detector
        options = apriltag.DetectorOptions(families=self.APRILTAG_FAMILY)
        self.detector = apriltag.Detector(options)
        self.previous_tag_corners: dict[int, np.ndarray | None] = {
            i: None for i in range(8)
        }

        # AprilTag detection timing
        self.last_apriltag_detection_time: float = 0.0
        self.apriltag_detection_interval: float = 1.0 / self.APRILTAG_DETECTION_FPS

        # Perspective transform
        self.M: np.ndarray | None = None
        self.width = 0
        self.height = 0

        # Goal positions
        self.left_goal: list[float] | None = None
        self.right_goal: list[float] | None = None

        # Gantry/magnet tracking
        self.xy_gantry: list[float] | None = None
        self.x_gantry: list[float] | None = None

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

        # Comprehensive initial board detection
        self._initial_board_detection()

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

    def _initial_board_detection(self) -> None:
        """Perform initial board detection using flood fill to establish perspective transform."""

        # Wait for a valid frame from the camera
        ret, frame = self.cap.read()
        while not ret:
            rclpy.spin_once(self, timeout_sec=0.1)
            ret, frame = self.cap.read()

        # Skip a few frames to allow camera auto-adjustments
        for i in range(5):
            ret, frame = self.cap.read()

        ret, frame = self.cap.read()
        frame_rec = cv2.remap(frame, self.mapx, self.mapy, cv2.INTER_LINEAR)

        frame_hsv = cv2.cvtColor(frame_rec, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(frame_hsv)

        # Perform flood fill to detect board boundaries
        flood_mask, rect = self._board_flood_fill(
            s, None, self.FLOOD_SEED, self.FLOOD_THRESHOLD
        )

        # Find rotated rectangle from flood fill mask
        rotated_rect = self._find_rotated_rect_from_flood_mask(flood_mask, rect)

        # Compute border segment masks
        boarder_segment_masks, seed_lines = self._compute_boarder_segment_masks(
            rotated_rect, s.shape
        )

        line_samples = self._compute_seed_line_samples(seed_lines)

        boarder_segment_flood_masks = []

        for i, (boarder_segment_mask, line_sample_points) in enumerate(
            zip(boarder_segment_masks, line_samples)
        ):
            boarder_segment_flood_mask = None
            for sample_point in line_sample_points:
                masked_image = cv2.bitwise_and(s, s, mask=boarder_segment_mask)
                boarder_segment_flood_mask, _ = self._board_flood_fill(
                    masked_image,
                    boarder_segment_flood_mask,
                    tuple(sample_point),
                    self.FLOOD_THRESHOLD,
                    crop_mask=False,
                )
            boarder_segment_flood_mask = boarder_segment_flood_mask[1:-1, 1:-1]
            boarder_segment_flood_masks.append(boarder_segment_flood_mask)

        # Compute perspective transform
        self.compute_perspective_transform_from_rotated_rect(rotated_rect)

        self.get_logger().info(f"Board detected: {self.width}x{self.height} pixels")

        if self.DEBUG_VIEW:
            self._print_debug_view(
                frame_rec,
                h,
                s,
                v,
                flood_mask,
                boarder_segment_masks,
                rotated_rect,
                seed_lines,
                line_samples,
                boarder_segment_flood_masks,
            )

    def _compute_seed_line_samples(self, seed_lines: list[np.ndarray]):

        line_sample_count = 4
        line_samples = []
        for seed_line in seed_lines:
            line_vec = seed_line[:, 1] - seed_line[:, 0]
            line_length = np.linalg.norm(line_vec)
            dir = line_vec / line_length
            interval = line_length / (line_sample_count + 1)
            sample_points = [
                np.intp(seed_line[:, 0] + dir * (interval * (j + 1)))
                for j in range(line_sample_count)
            ]
            line_samples.append(sample_points)
        return line_samples

    def _print_debug_view(
        self,
        frame_rec: np.ndarray,
        h: np.ndarray,
        s: np.ndarray,
        v: np.ndarray,
        flood_mask: np.ndarray,
        boarder_segment_masks: list[np.ndarray],
        rotated_rect: cv2.RotatedRect,
        seed_lines: list[tuple[np.ndarray, np.ndarray]],
        seed_line_samples: list[list[np.ndarray]],
        boarder_segment_flood_masks: list[np.ndarray],
    ) -> None:
        cv2.imshow("Initial Board Analysis - Rect", frame_rec)
        self._plot_single_channel(h, "Initial Board Analysis - H Channel")
        self._plot_single_channel(s, "Initial Board Analysis - S Channel")
        self._plot_single_channel(v, "Initial Board Analysis - V Channel")
        cv2.imshow("Initial Board Analysis - Flood Fill", flood_mask)

        # Draw all border segment polygons on the original frame
        frame_with_polygons = frame_rec.copy()
        merged_mask = np.zeros_like(s, dtype=np.uint8)
        merged_flood_mask = np.zeros_like(s, dtype=np.uint8)
        for i, (mask, seed_line, seed_line_sample, boarder_segment_flood_mask) in enumerate(
            zip(boarder_segment_masks, seed_lines, seed_line_samples, boarder_segment_flood_masks)
        ):
            # Find contours from the mask
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
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
            merged_flood_mask = cv2.bitwise_or(merged_flood_mask, boarder_segment_flood_mask)

        # Display border segment
        masked_image = cv2.bitwise_and(s, s, mask=merged_mask)
        self._plot_single_channel(masked_image, "Initial Board Analysis - Border Segments")

        cv2.imshow(
                f"Initial Board Analysis - Border Segment Flood Mask",
                merged_flood_mask,
            )

        cv2.imshow(
            "Initial Board Analysis - Border Segments Overlay", frame_with_polygons
        )

        # Draw the rotated rectangle on a copy of the original image
        frame_with_rect = frame_rec.copy()
        box = cv2.boxPoints(rotated_rect)
        box = np.intp(box)
        cv2.drawContours(frame_with_rect, [box], 0, (0, 255, 0), 2)

        cv2.imshow("Initial Board Analysis - Flood Fill Rectangle", frame_with_rect)
        cv2.waitKey(0)

    def _compute_boarder_segment_masks(
        self, rotated_rect, shape: tuple[int, int]
    ) -> list[np.ndarray]:
        # Create boarder masks
        corner_pts, rect_width, rect_height, rotation_matrix = (
            self._corners_from_rotated_rect(rotated_rect)
        )
        boarder_segment_masks = [np.zeros(shape, dtype=np.uint8) for _ in range(4)]
        outside_offset = 20
        inside_offset = 50
        corner_distance = 100
        long_side_length = rect_width - 2 * corner_distance
        short_side_length = rect_height - 2 * corner_distance
        center = np.reshape(rotated_rect[0], (2, 1))

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

        parameter_combination = [
            (inside_offset, outside_offset, long_side_length / 2, long_side_length / 2),
            (
                short_side_length / 2,
                short_side_length / 2,
                inside_offset,
                outside_offset,
            ),
            (outside_offset, inside_offset, long_side_length / 2, long_side_length / 2),
            (
                short_side_length / 2,
                short_side_length / 2,
                outside_offset,
                inside_offset,
            ),
        ]

        rects = [rect_from_offset(*params) for params in parameter_combination]
        rects_transformed = [rotation_matrix @ rect + center for rect in rects]
        seed_lines = []

        for i, boarder_segment_mask in enumerate(boarder_segment_masks):
            pt1 = np.reshape(corner_pts[i], (2, 1))
            pt2 = np.reshape(corner_pts[(i + 1) % 4], (2, 1))
            line_center = (pt2 - pt1) / 2 + pt1
            parallel_dir = line_center - center
            rect_pts = np.intp(rects_transformed[i] + parallel_dir)
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

        return boarder_segment_masks, seed_lines

    def _board_flood_fill(
        self,
        channel: np.ndarray,
        mask: np.ndarray | None,
        seed: tuple[int, int],
        threshold: int,
        crop_mask: bool = True,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, cv2.typing.Rect]:

        # Perform flood fill
        # flags: connectivity (4 or 8) + fill mask only option
        flags = 4 | (255 << 8) | cv2.FLOODFILL_MASK_ONLY
        retval, image, flood_mask, rect = cv2.floodFill(
            channel, mask, seed, 255, loDiff=threshold, upDiff=threshold, flags=flags
        )

        # Get all points where flood_mask is non-zero
        if crop_mask:
            flood_mask = flood_mask[1:-1, 1:-1]

        return flood_mask, rect

    def _find_rotated_rect_from_flood_mask(
        self, flood_mask: np.ndarray, rect: cv2.typing.Rect
    ) -> cv2.RotatedRect:
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

    def compute_perspective_transform_from_rotated_rect(
        self, rotated_rect: tuple[tuple[float, float], tuple[float, float], float]
    ) -> None:
        """Compute perspective transformation matrix from rotated rectangle."""

        src_pts, rect_width, rect_height, _ = self._corners_from_rotated_rect(
            rotated_rect
        )

        # Set dimensions
        self.width = int(rect_width)
        self.height = int(rect_height)

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
        self.M = cv2.getPerspectiveTransform(src_pts, dst_pts)

    def _plot_single_channel(self, channel: np.ndarray, title: str) -> None:
        """Plot a single channel with color scale."""

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

    def _print_profiling_stats(self) -> None:
        """Print cProfile statistics."""
        if self.profiler is None:
            return

        self.profiler.disable()

        # Create a string buffer to capture the stats output
        s = io.StringIO()

        # Sort by total time (tottime) and print top functions
        ps = pstats.Stats(self.profiler, stream=s).sort_stats("tottime")

        self.get_logger().info("\n" + "=" * 80)
        self.get_logger().info(
            f"cProfile Statistics (profiled for {self.PROFILING_DURATION} seconds)"
        )
        self.get_logger().info("=" * 80)

        # Print stats to string buffer
        ps.print_stats(self.PROFILING_TOP_FUNCTIONS)

        # Log the output
        self.get_logger().info("\n" + s.getvalue())

        # Also print callers for more detailed analysis
        s = io.StringIO()
        ps = pstats.Stats(self.profiler, stream=s).sort_stats("tottime")
        ps.print_callers(20)

        self.get_logger().info("\nTop Callers:")
        self.get_logger().info(s.getvalue())
        self.get_logger().info("=" * 80 + "\n")

    def timer_callback(self) -> None:
        """Main timer callback for processing camera frames."""
        # Capture and validate frame
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to grab frame")
            return

        # Undistort frame using calibration data
        rectified_frame = cv2.remap(frame, self.mapx, self.mapy, cv2.INTER_LINEAR)

        # Detect AprilTags at specified frequency
        current_time = time.time()
        if (
            current_time - self.last_apriltag_detection_time
            >= self.apriltag_detection_interval
        ):
            self.detect_apriltags(rectified_frame)
            self.last_apriltag_detection_time = current_time

        # Process frame
        self.process_frame(rectified_frame)

    def detect_apriltags(self, undistorted_frame: np.ndarray) -> None:
        """Detect and process AprilTags in the frame."""
        gray = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2GRAY)
        results = self.detector.detect(gray)

        tag_corners: dict[int, np.ndarray | None] = {i: None for i in range(8)}

        # Process detected tags with EMA smoothing
        for r in results:
            if r.tag_id not in self.previous_tag_corners:
                continue

            # Determine alpha based on tag type (gantry tags use no smoothing)
            alpha = self.EMA_ALPHA_GANTRY if r.tag_id >= 6 else self.EMA_ALPHA_APRILTAG

            smoothed_corners = []
            for i, corner in enumerate(r.corners):
                if self.previous_tag_corners[r.tag_id] is None:
                    smoothed_corner = corner
                else:
                    smoothed_corner = apply_ema_filter(
                        corner,
                        self.previous_tag_corners[r.tag_id][i],
                        alpha=alpha,
                    )
                smoothed_corners.append(smoothed_corner)

            # Store smoothed corners
            tag_corners[r.tag_id] = np.array(smoothed_corners, dtype="float32")
            self.previous_tag_corners[r.tag_id] = np.array(
                smoothed_corners, dtype="float32"
            )

        # Update goal coordinates and publish
        self.update_goal_coordinates()

        # Reset gantry tag corners if not detected
        for gantry_tag_id in [6, 7]:
            if tag_corners[gantry_tag_id] is None:
                self.previous_tag_corners[gantry_tag_id] = None

        # Update gantry and magnet positions and publish
        self.update_gantry_coordinates()

    def process_frame(self, undistorted_frame: np.ndarray) -> None:
        """Process frame to create perspective-corrected view and publish."""
        # Perform flood fill to detect board boundaries every frame
        h, s, v, flood_mask, rect = self._board_flood_fill(
            undistorted_frame, self.FLOOD_SEED, self.FLOOD_THRESHOLD
        )
        rotated_rect = self._find_rotated_rect_from_flood_mask(flood_mask, rect)

        # Update perspective transform from flood fill rotated rectangle
        self.compute_perspective_transform_from_rotated_rect(rotated_rect)

        # Apply perspective warp to get top-down view
        warped = cv2.warpPerspective(
            undistorted_frame,
            self.M,
            (self.width, self.height),
        )

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

    def update_goal_coordinates(self) -> None:
        """Update goal positions from AprilTag detections and publish."""
        if self.M is None:
            return

        goals_updated = False

        # Update right goal (tag 0)
        if self.previous_tag_corners[0] is not None:
            right_goal_centroid = np.mean(self.previous_tag_corners[0], axis=0)
            right_goal_warped = cv2.perspectiveTransform(
                np.array([[right_goal_centroid]], dtype="float32"), self.M
            )[0][0]
            self.right_goal = right_goal_warped.tolist()
            goals_updated = True

        # Update left goal (tag 1)
        if self.previous_tag_corners[1] is not None:
            left_goal_centroid = np.mean(self.previous_tag_corners[1], axis=0)
            left_goal_warped = cv2.perspectiveTransform(
                np.array([[left_goal_centroid]], dtype="float32"), self.M
            )[0][0]
            self.left_goal = left_goal_warped.tolist()
            goals_updated = True

        # Publish goal positions if both are available
        if goals_updated and self.left_goal is not None and self.right_goal is not None:
            self.publish_goal_positions()

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

    def update_gantry_coordinates(self) -> None:
        """Update gantry positions from AprilTag detections and publish."""
        if self.M is None:
            return

        # Update xy gantry position (tag 6)
        if self.previous_tag_corners[6] is not None:
            xy_gantry_centroid = np.mean(self.previous_tag_corners[6], axis=0)
            xy_gantry_warped = cv2.perspectiveTransform(
                np.array([[xy_gantry_centroid]], dtype="float32"), self.M
            )[0][0]
            self.xy_gantry = xy_gantry_warped.tolist()
        else:
            self.xy_gantry = None

        # Update x gantry position (tag 7)
        if self.previous_tag_corners[7] is not None:
            x_gantry_centroid = np.mean(self.previous_tag_corners[7], axis=0)
            x_gantry_warped = cv2.perspectiveTransform(
                np.array([[x_gantry_centroid]], dtype="float32"), self.M
            )[0][0]
            self.x_gantry = x_gantry_warped.tolist()
        else:
            self.x_gantry = None

        # Publish gantry positions
        self.publish_gantry_positions()

    def publish_gantry_positions(self) -> None:
        """Publish gantry and magnet positions."""
        XY_GANTRY_X_OFFSET = 112
        X_GANTRY_X_OFFSET = 77

        polygon = Polygon()

        # Calculate magnet position
        if self.xy_gantry is not None:
            magnet_x = float(self.xy_gantry[0] - XY_GANTRY_X_OFFSET)
            magnet_y = float(self.xy_gantry[1])

            point = Point32()
            point.x = magnet_x
            point.y = magnet_y
            point.z = 0.0
            polygon.points.append(point)
        elif self.x_gantry is not None:
            magnet_x = float(self.x_gantry[0] - X_GANTRY_X_OFFSET)

            point = Point32()
            point.x = magnet_x
            point.y = 0.0  # Y position unknown
            point.z = 1.0  # Use z=1 to indicate y is unknown
            polygon.points.append(point)

        if len(polygon.points) > 0:
            stamped_polygon = StampedPolygon()
            stamped_polygon.polygon = polygon
            stamped_polygon.header.stamp = self.get_clock().now().to_msg()
            self.gantry_publisher.publish(stamped_polygon)


def main(args=None):
    rclpy.init(args=args)

    camera_node = CameraNode()

    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
