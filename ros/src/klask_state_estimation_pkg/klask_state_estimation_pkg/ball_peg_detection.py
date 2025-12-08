"""ROS2 node for detecting and tracking ball and peg positions using AprilTags and computer vision."""

import cv2
import time
import rclpy
import numpy as np
import apriltag
import cProfile
import pstats
import io
from rclpy.node import Node
from geometry_msgs.msg import Polygon, Point32
from klask_interfaces.msg import StampedPolygon, StampedInt32
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

from .kalman_filter import KalmanFilter
from .utils import load_calibration_data, apply_ema_filter, resize_with_aspect_ratio


class BallPegDetection(Node):
    """ROS2 node for detecting and tracking ball and peg positions using AprilTags and computer vision."""

    # Constants
    CAMERA_FPS = 120
    CAMERA_WIDTH = 1280
    CAMERA_HEIGHT = 720
    TAG_SIZE_MM = 20
    APRILTAG_FAMILY = "tag36h11"
    FLOOD_SEED = (CAMERA_WIDTH // 2, CAMERA_HEIGHT // 2)
    FLOOD_THRESHOLD = 5

    # Debug/Display settings
    DEBUG_VIEW = False
    SHOW_IMAGE = False
    SHOW_FPS = False
    PRINT_OUTCOME = False

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

    # Goal detection constants
    GOAL_RADIUS = 22
    GOAL_PERSPECTIVE_SHIFT = 4
    MAX_GOAL_COUNTER = 30  # Frames required before confirming goal

    # Collision detection constants
    COLLISION_DISTANCE = 35.0  # Distance threshold for collision detection

    # EMA filter alpha values
    EMA_ALPHA_APRILTAG = 0.1
    EMA_ALPHA_GANTRY = 1.0  # No smoothing for gantry tags

    # AprilTag detection rate
    APRILTAG_DETECTION_FPS = 5.0  # Detect AprilTags at this frequency (Hz)

    # HSV color range constants for object detection
    BALL_HSV_LOWER = (0, 150, 150)  # Orange ball lower bound
    BALL_HSV_UPPER = (45, 255, 255)  # Orange ball upper bound
    PEG_HSV_LOWER = (0, 0, 0)  # Black peg lower bound
    PEG_HSV_UPPER = (255, 255, 45)  # Black peg upper bound

    # Visualization canvas size
    CANVAS_WIDTH = 1280
    CANVAS_HEIGHT = 720

    # Display update rate
    DISPLAY_UPDATE_INTERVAL = 0.1  # 10Hz display update (100ms between frames)

    def __init__(self):
        super().__init__("detect_ball_peg")

        # Publishers
        self.state_publisher = self.create_publisher(
            StampedPolygon, "ball_peg_states", 10
        )
        self.outcome_publisher = self.create_publisher(StampedInt32, "outcome", 10)
        self.image_publisher = self.create_publisher(
            CompressedImage, "board_image/compressed", 10
        )

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Timers
        self.timer = self.create_timer(1.0 / 480.0, self.timer_callback)

        # Kalman Filters
        self.ball_kf = KalmanFilter(
            process_noise_position=2.0,
            process_noise_velocity=30.0,
            measurement_noise_position=1.0,
        )
        self.left_peg_kf = KalmanFilter(
            process_noise_position=2.0,
            process_noise_velocity=800.0,
            measurement_noise_position=12.0,
            stop_threshold=0.3,
        )
        self.right_peg_kf = KalmanFilter(
            process_noise_position=2.0,
            process_noise_velocity=800.0,
            measurement_noise_position=12.0,
            stop_threshold=0.3,
        )

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

        # Timing
        self.previous_time: float | None = None
        self.dt: float = 0.01

        # AprilTag detection timing
        self.last_apriltag_detection_time: float = 0.0
        self.apriltag_detection_interval: float = 1.0 / self.APRILTAG_DETECTION_FPS

        # FPS tracking
        self.fps_update_interval = 1.0  # Update FPS display every second
        self.fps_last_update_time = time.time()
        self.fps_frame_count = 0
        self.current_fps = 0.0

        # Display throttling
        self.last_display_time = 0.0

        # Detection state flags
        self.goals_detected_once_printed = False
        self.goals_detected_this_frame = False

        # Perspective transform
        self.M: np.ndarray | None = None
        self.width = 0
        self.height = 0
        self.inset_pixels = 0
        self.board_rect: tuple[int, int, int, int] | None = (
            None  # Store flood fill rect
        )

        # Object positions
        self.left_peg_position: tuple[float, float] | None = None
        self.right_peg_position: tuple[float, float] | None = None
        self.ball_position: tuple[float, float] | None = None

        # Goal positions (initial estimates, updated from AprilTags)
        self.left_goal = [44.44, 188.20]
        self.right_goal = [498.0, 188.50]

        # Playing field boundaries [x_min, x_max, y_min, y_max]
        self.edge = np.array([0.0, 530.0, 0.0, 370.0])

        # Gantry/magnet tracking
        self.xy_gantry: list[float] | None = None
        self.x_gantry: list[float] | None = None
        self.magnet: list[float | None] | None = None

        # Goal detection counters
        self.ball_in_left_goal_counter = 0
        self.ball_in_right_goal_counter = 0
        self.peg_in_left_goal_counter = 0
        self.peg_in_right_goal_counter = 0
        self.outcome_published = False

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

        # Comprehensive initial board image analysis
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

        frame_rec = cv2.remap(frame, self.mapx, self.mapy, cv2.INTER_LINEAR)
        if self.DEBUG_VIEW:
            cv2.imshow("Initial Board Analysis - Rect", frame_rec)

        h, s, v, flood_mask, rect = self._board_flood_fill(
            frame_rec, self.FLOOD_SEED, self.FLOOD_THRESHOLD
        )

        # Store the board rectangle and compute perspective transform
        self.board_rect = rect
        self.compute_perspective_transform_from_rect(rect, frame_rec.shape)
        self.apriltags_detected_once = True  # Board is detected via flood fill
        self.corner_points_detected_once_printed = True
        self.get_logger().info(f"Board detected via flood fill: rect={rect}")

        if self.DEBUG_VIEW:
            cv2.imshow("Initial Board Analysis - H Channel", h)
            cv2.imshow("Initial Board Analysis - S Channel", s)
            cv2.imshow("Initial Board Analysis - V Channel", v)
            self._plot_single_channel(
                s, "Initial Board Analysis - S Channel with Color Scale"
            )

            cv2.imshow("Initial Board Analysis - Flood Fill", flood_mask)

            # Draw the bounding rectangle on a copy of the original image
            frame_with_rect = frame_rec.copy()
            x, y, w, h = rect
            cv2.rectangle(frame_with_rect, (x, y), (x + w, y + h), (0, 255, 0), 2)

            cv2.imshow("Initial Board Analysis - Flood Fill Rectangle", frame_with_rect)

            cv2.waitKey(0)
            cv2.destroyAllWindows()

    def _board_flood_fill(
        self, frame_rec: np.ndarray, seed: tuple[int, int], threshold: int
    ) -> np.ndarray:
        frame_hsv = cv2.cvtColor(frame_rec, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(frame_hsv)
        # Perform flood fill
        # flags: connectivity (4 or 8) + fill mask only option
        flags = 4 | (255 << 8) | cv2.FLOODFILL_MASK_ONLY
        retval, image, flood_mask, rect = cv2.floodFill(
            s, None, seed, 255, loDiff=threshold, upDiff=threshold, flags=flags
        )
        return h, s, v, flood_mask, rect

    def _plot_single_channel(self, channel: np.ndarray, title: str) -> None:
        """Plot a single channel with color scale."""

        # Apply colormap to S channel for better visualization
        s_colored = cv2.applyColorMap(channel, cv2.COLORMAP_JET)

        # Create a color scale bar (0-255 range)
        scale_height = s_colored.shape[0]
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

        # Concatenate the colored S channel with the scale bar
        s_with_scale = np.hstack([s_colored, scale_bar_colored])

        cv2.imshow(title, s_with_scale)

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
        # Check if profiling duration has elapsed
        if self.ENABLE_PROFILING and self.profiler is not None:
            if time.time() - self.profiling_start_time >= self.PROFILING_DURATION:
                self._print_profiling_stats()
                self.profiler = None  # Disable further profiling

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

        # Process frame for ball/peg detection
        self.process_frame(rectified_frame)

        # Update FPS counter
        self._update_fps()

    def _update_fps(self) -> None:
        """Update FPS calculation."""
        self.fps_frame_count += 1
        current_time = time.time()
        elapsed = current_time - self.fps_last_update_time

        if elapsed >= self.fps_update_interval:
            self.current_fps = self.fps_frame_count / elapsed
            self.fps_frame_count = 0
            self.fps_last_update_time = current_time

            # Log FPS when image display is disabled
            if not self.SHOW_IMAGE:
                self.get_logger().info(f"FPS: {self.current_fps:.1f}")

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

        # Check goal tags (0-1)
        detected_goal_tags = sum(
            1 for tag_id in self.GOAL_TAG_IDS if tag_corners[tag_id] is not None
        )
        self.goals_detected_this_frame = detected_goal_tags == 2
        self.update_goal_coordinates()

        # Reset gantry tag corners if not detected
        for gantry_tag_id in [6, 7]:
            if tag_corners[gantry_tag_id] is None:
                self.previous_tag_corners[gantry_tag_id] = None

        # Update gantry and magnet positions
        self.update_gantry_coordinates()
        self.update_magnet_coordinates()

    def process_frame(self, undistorted_frame: np.ndarray) -> None:
        """Process frame to detect ball and pegs, then publish results."""
        # Update delta time for Kalman filters
        current_time = time.time()
        self.dt = (
            current_time - self.previous_time
            if self.previous_time is not None
            else 0.01
        )
        self.previous_time = current_time

        # Perform flood fill to detect board boundaries every frame
        h, s, v, flood_mask, rect = self._board_flood_fill(
            undistorted_frame, self.FLOOD_SEED, self.FLOOD_THRESHOLD
        )

        # Update perspective transform from flood fill rectangle
        self.board_rect = rect
        self.compute_perspective_transform_from_rect(rect, undistorted_frame.shape)

        # Apply perspective warp to get top-down view
        warped = cv2.warpPerspective(
            undistorted_frame,
            self.M,
            (self.width, self.height),
        )

        # No cropping needed since flood fill rectangle defines the exact board area
        cropped = warped

        # Publish the transformed and cropped image
        self.publish_board_image(cropped)

        # Detect ball and pegs
        canvas, self.left_peg_position, self.right_peg_position, self.ball_position = (
            self.detect_ball_peg(
                cropped,
                left_goal=self.left_goal,
                right_goal=self.right_goal,
            )
        )

        # Display image at 10Hz to improve performance
        if self.SHOW_IMAGE:
            current_time = time.time()
            if current_time - self.last_display_time >= self.DISPLAY_UPDATE_INTERVAL:
                cv2.imshow("Canvas", canvas)
                cv2.waitKey(1)
                self.last_display_time = current_time

        # Publish states
        self.publish_estimated_positions_and_velocities()

        # Check for goals if all objects are detected
        if all(
            [
                self.ball_position,
                self.left_peg_position,
                self.right_peg_position,
                self.left_goal,
                self.right_goal,
            ]
        ):
            self.check_goal()

    def compute_perspective_transform_from_rect(
        self, rect: tuple[int, int, int, int], frame_shape: tuple[int, int, int]
    ) -> None:
        """Compute perspective transformation matrix from flood fill rectangle."""
        x, y, w, h = rect

        # Source points from rectangle corners (clockwise from top-left)
        src_pts = np.array(
            [
                [x, y],  # Top-left
                [x + w, y],  # Top-right
                [x + w, y + h],  # Bottom-right
                [x, y + h],  # Bottom-left
            ],
            dtype="float32",
        )

        # Set dimensions (no inset needed for flood fill rectangle)
        self.inset_pixels = 0
        self.width = w
        self.height = h

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

    def detect_ball_peg(
        self,
        cropped: np.ndarray,
        left_goal: list[float] | None = None,
        right_goal: list[float] | None = None,
    ) -> tuple[
        np.ndarray,
        tuple[float, float] | None,
        tuple[float, float] | None,
        tuple[float, float] | None,
    ]:
        """
        Detect ball and pegs using HSV color filtering and Kalman filtering.

        Returns:
            Tuple of (canvas, left_peg_position, right_peg_position, ball_position)
        """
        # Convert to HSV and apply blur for noise reduction
        frame_hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
        frame_hsv_blur = cv2.GaussianBlur(frame_hsv, (7, 7), 0)

        # Create masks for ball (orange) and pegs (black)
        masked_ball = cv2.inRange(
            frame_hsv_blur, self.BALL_HSV_LOWER, self.BALL_HSV_UPPER
        )
        masked_peg = cv2.inRange(frame_hsv_blur, self.PEG_HSV_LOWER, self.PEG_HSV_UPPER)

        # Create visualization overlay
        overlaid_frame = self._create_overlay(cropped, masked_peg, masked_ball)

        # Detect pegs in left and right halves
        height, width = masked_peg.shape
        left_half = masked_peg[:, : width // 2]
        right_half = masked_peg[:, width // 2 :]

        left_peg_position = self._detect_peg(
            left_half, self.left_peg_kf, 0, overlaid_frame, (255, 0, 0), (248, 193, 110)
        )
        right_peg_position = self._detect_peg(
            right_half,
            self.right_peg_kf,
            width // 2,
            overlaid_frame,
            (0, 100, 0),
            (144, 238, 144),
        )

        # Detect ball with collision detection
        ball_position = self._detect_ball(
            masked_ball, left_peg_position, right_peg_position, overlaid_frame
        )

        # Draw goals and magnet visualization
        if self.SHOW_IMAGE:
            self._draw_goals(overlaid_frame, left_goal, right_goal)

        # Resize and center on canvas
        canvas = self._create_canvas(overlaid_frame)

        return canvas, left_peg_position, right_peg_position, ball_position

    def _create_overlay(
        self, frame: np.ndarray, masked_peg: np.ndarray, masked_ball: np.ndarray
    ) -> np.ndarray:
        """Create visualization overlay with masks."""
        mask_3_channel_peg = cv2.cvtColor(masked_peg, cv2.COLOR_GRAY2BGR)
        mask_3_channel_ball = cv2.cvtColor(masked_ball, cv2.COLOR_GRAY2BGR)

        overlaid = cv2.addWeighted(frame, 1.0, mask_3_channel_peg, 0.3, 0)
        overlaid = cv2.addWeighted(overlaid, 1.0, mask_3_channel_ball, 0.3, 0)
        return overlaid

    def _detect_peg(
        self,
        half_mask: np.ndarray,
        kf: KalmanFilter,
        x_offset: int,
        overlaid_frame: np.ndarray,
        color: tuple[int, int, int],
        velocity_color: tuple[int, int, int],
    ) -> tuple[float, float] | None:
        """Detect peg in half of the frame."""
        contours, _ = cv2.findContours(
            half_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return None

        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M["m00"] == 0:
            return None

        # Calculate centroid
        cX = int(M["m10"] / M["m00"]) + x_offset
        cY = int(M["m01"] / M["m00"])

        # Apply Kalman filter
        kf.predict(self.dt)
        kf.update([cX, cY])

        position = kf.get_position()

        if self.SHOW_IMAGE:
            velocity = kf.get_velocity()
            self._draw_object_with_velocity(
                overlaid_frame, position, velocity, color, velocity_color, 0.5
            )

        return (float(position[0]), float(position[1]))

    def _detect_ball(
        self,
        masked_ball: np.ndarray,
        left_peg_position: tuple[float, float] | None,
        right_peg_position: tuple[float, float] | None,
        overlaid_frame: np.ndarray,
    ) -> tuple[float, float] | None:
        """Detect ball with collision-aware Kalman filtering."""
        contours, _ = cv2.findContours(
            masked_ball, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return None

        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)

        if M["m00"] == 0:
            return None

        # Calculate centroid
        cX_ball = int(M["m10"] / M["m00"])
        cY_ball = int(M["m01"] / M["m00"])

        # Detect potential collisions
        x_collision, y_collision = self._detect_collisions(
            cX_ball, cY_ball, left_peg_position, right_peg_position
        )

        # Apply Kalman filter with collision awareness
        self.ball_kf.predict(self.dt, x_collision=x_collision, y_collision=y_collision)
        self.ball_kf.update([cX_ball, cY_ball])

        position = self.ball_kf.get_position()

        if self.SHOW_IMAGE:
            velocity = self.ball_kf.get_velocity()
            self._draw_object_with_velocity(
                overlaid_frame, position, velocity, (0, 0, 255), (0, 165, 255), 0.2
            )

        return (float(position[0]), float(position[1]))

    def _detect_collisions(
        self,
        ball_x: float,
        ball_y: float,
        left_peg_position: tuple[float, float] | None,
        right_peg_position: tuple[float, float] | None,
    ) -> tuple[bool, bool]:
        """Detect if ball is near edges or pegs (potential collision)."""
        at_x_edge = (
            ball_x < self.edge[0] + self.COLLISION_DISTANCE
            or ball_x + self.COLLISION_DISTANCE > self.edge[1]
        )
        at_y_edge = (
            ball_y < self.edge[2] + self.COLLISION_DISTANCE
            or ball_y + self.COLLISION_DISTANCE > self.edge[3]
        )

        close_to_peg = False
        if left_peg_position and right_peg_position:
            dist_left = np.linalg.norm(
                np.array([ball_x, ball_y]) - np.array(left_peg_position)
            )
            dist_right = np.linalg.norm(
                np.array([ball_x, ball_y]) - np.array(right_peg_position)
            )
            close_to_peg = (
                dist_left < self.COLLISION_DISTANCE
                or dist_right < self.COLLISION_DISTANCE
            )

        # Determine collision type
        if close_to_peg or (at_x_edge and at_y_edge):
            return True, True
        elif at_y_edge:
            return False, True
        elif at_x_edge:
            return True, True  # Note: Original code had y_collision=True for x_edge
        else:
            return False, False

    def _draw_object_with_velocity(
        self,
        frame: np.ndarray,
        position: np.ndarray,
        velocity: list[float],
        dot_color: tuple[int, int, int],
        arrow_color: tuple[int, int, int],
        velocity_scale: float,
    ) -> None:
        """Draw object position and velocity vector."""
        pos_int = (int(position[0]), int(position[1]))

        # Draw position
        cv2.circle(frame, pos_int, 5, dot_color, -1)

        # Draw velocity arrow
        end_point = (
            int(position[0] + velocity[0] * velocity_scale),
            int(position[1] + velocity[1] * velocity_scale),
        )
        cv2.arrowedLine(frame, pos_int, end_point, arrow_color, 2, tipLength=0.3)

    def _draw_goals(
        self,
        frame: np.ndarray,
        left_goal: list[float] | None,
        right_goal: list[float] | None,
    ) -> None:
        """Draw goal circles on the frame."""
        goal_color = (140, 255, 0)

        if left_goal is not None:
            center = (int(left_goal[0]), int(left_goal[1]))
            cv2.circle(frame, center, self.GOAL_RADIUS, goal_color, 2)
            cv2.circle(frame, center, 5, goal_color, -1)

        if right_goal is not None:
            center = (int(right_goal[0]), int(right_goal[1]))
            cv2.circle(frame, center, self.GOAL_RADIUS, goal_color, 2)
            cv2.circle(frame, center, 5, goal_color, -1)

    def _create_canvas(self, overlaid_frame: np.ndarray) -> np.ndarray:
        """Resize frame and center it on a canvas."""
        resized_image, new_w, new_h = resize_with_aspect_ratio(
            overlaid_frame, self.CANVAS_WIDTH, self.CANVAS_HEIGHT
        )

        # Ensure dimensions are within bounds
        if new_w > self.CANVAS_WIDTH or new_h > self.CANVAS_HEIGHT:
            self.get_logger().warn(
                f"Resized image ({new_w}x{new_h}) exceeds canvas ({self.CANVAS_WIDTH}x{self.CANVAS_HEIGHT})"
            )
            new_w = min(new_w, self.CANVAS_WIDTH)
            new_h = min(new_h, self.CANVAS_HEIGHT)
            resized_image = cv2.resize(
                resized_image, (new_w, new_h), interpolation=cv2.INTER_AREA
            )

        # Center on black canvas
        canvas = np.zeros((self.CANVAS_HEIGHT, self.CANVAS_WIDTH, 3), dtype=np.uint8)
        x_offset = (self.CANVAS_WIDTH - new_w) // 2
        y_offset = (self.CANVAS_HEIGHT - new_h) // 2
        canvas[y_offset : y_offset + new_h, x_offset : x_offset + new_w] = resized_image

        # Draw FPS counter on canvas
        if self.SHOW_FPS and self.current_fps > 0:
            fps_text = f"FPS: {self.current_fps:.1f}"
            cv2.putText(
                canvas,
                fps_text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        return canvas

    def publish_estimated_positions_and_velocities(self) -> None:
        """Publish ball and peg positions/velocities, goals, and magnet position."""
        polygon = Polygon()

        # Add ball and peg data (position and velocity for each)
        vectors = [
            self.ball_position,
            self.ball_kf.get_velocity(),
            self.left_peg_kf.get_position(),
            self.left_peg_kf.get_velocity(),
            self.right_peg_kf.get_position(),
            self.right_peg_kf.get_velocity(),
        ]

        for vector in vectors:
            if vector is not None:
                polygon.points.append(self._create_point32(vector[0], vector[1]))

        # Add goal coordinates
        if self.left_goal is not None and self.right_goal is not None:
            polygon.points.append(
                self._create_point32(self.left_goal[0], self.left_goal[1])
            )
            polygon.points.append(
                self._create_point32(self.right_goal[0], self.right_goal[1])
            )

        # Add magnet position
        if self.magnet is not None:
            magnet_y = self.magnet[1] if self.magnet[1] is not None else 0.0
            polygon.points.append(self._create_point32(self.magnet[0], magnet_y))

        # Publish only if we have all 9 points
        if len(polygon.points) == 9:
            stamped_polygon = StampedPolygon()
            stamped_polygon.polygon = polygon
            stamped_polygon.header.stamp = self.get_clock().now().to_msg()
            self.state_publisher.publish(stamped_polygon)

    def _create_point32(self, x: float, y: float, z: float = 0.0) -> Point32:
        """Create a Point32 message from coordinates."""
        point = Point32()
        point.x = float(x)
        point.y = float(y)
        point.z = z
        return point

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
        """Update goal positions from AprilTag detections."""
        if self.M is None or not self.goals_detected_this_frame:
            return

        # Update right goal (tag 0)
        if self.previous_tag_corners[0] is not None:
            right_goal_centroid = np.mean(self.previous_tag_corners[0], axis=0)
            right_goal_warped = cv2.perspectiveTransform(
                np.array([[right_goal_centroid]], dtype="float32"), self.M
            )[0][0]
            # No offset needed since flood fill rect defines exact board boundaries
            self.right_goal = right_goal_warped.tolist()

        # Update left goal (tag 1)
        if self.previous_tag_corners[1] is not None:
            left_goal_centroid = np.mean(self.previous_tag_corners[1], axis=0)
            left_goal_warped = cv2.perspectiveTransform(
                np.array([[left_goal_centroid]], dtype="float32"), self.M
            )[0][0]
            # No offset needed since flood fill rect defines exact board boundaries
            self.left_goal = left_goal_warped.tolist()

    def update_gantry_coordinates(self) -> None:
        """Update gantry positions from AprilTag detections."""
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

    def update_magnet_coordinates(self) -> None:
        """Update magnet position based on gantry positions."""
        XY_GANTRY_X_OFFSET = 112
        X_GANTRY_X_OFFSET = 77

        if self.xy_gantry is not None:
            self.magnet = [
                float(self.xy_gantry[0] - XY_GANTRY_X_OFFSET),
                float(self.xy_gantry[1]),
            ]
        elif self.x_gantry is not None:
            self.magnet = [float(self.x_gantry[0] - X_GANTRY_X_OFFSET), None]

    def check_goal(self) -> None:
        """Check if ball or peg has scored and publish outcome."""
        if self.left_goal is None or self.right_goal is None:
            return

        if not self.goals_detected_once_printed:
            self.get_logger().info("Both goals detected: Ready to play!")
            self.goals_detected_once_printed = True

        # Calculate distances
        distances = {
            "ball_left": np.linalg.norm(np.array(self.ball_position) - self.left_goal),
            "ball_right": np.linalg.norm(
                np.array(self.ball_position) - self.right_goal
            ),
            "peg_left": np.linalg.norm(
                np.array(self.left_peg_position) - self.left_goal
            ),
            "peg_right": np.linalg.norm(
                np.array(self.right_peg_position) - self.right_goal
            ),
        }

        # Update counters
        self.ball_in_left_goal_counter = self._update_goal_counter(
            distances["ball_left"], self.ball_in_left_goal_counter
        )
        self.ball_in_right_goal_counter = self._update_goal_counter(
            distances["ball_right"], self.ball_in_right_goal_counter
        )
        self.peg_in_left_goal_counter = self._update_goal_counter(
            distances["peg_left"], self.peg_in_left_goal_counter
        )
        self.peg_in_right_goal_counter = self._update_goal_counter(
            distances["peg_right"], self.peg_in_right_goal_counter
        )

        # Check and publish outcomes
        self.ball_in_left_goal_counter = self._check_and_publish_goal_outcome(
            self.ball_in_left_goal_counter, "Ball in left goal!", 0
        )
        self.ball_in_right_goal_counter = self._check_and_publish_goal_outcome(
            self.ball_in_right_goal_counter, "Ball in right goal!", 1
        )
        self.peg_in_left_goal_counter = self._check_and_publish_goal_outcome(
            self.peg_in_left_goal_counter, "Peg in left goal!", 2
        )
        self.peg_in_right_goal_counter = self._check_and_publish_goal_outcome(
            self.peg_in_right_goal_counter, "Peg in right goal!", 3
        )

        # Reset outcome flag when all clear
        if all(
            counter == 0
            for counter in [
                self.ball_in_left_goal_counter,
                self.ball_in_right_goal_counter,
                self.peg_in_left_goal_counter,
                self.peg_in_right_goal_counter,
            ]
        ):
            if self.outcome_published and self.goals_detected_this_frame:
                self.get_logger().info("Both goals detected: Ready to play again!")
                self.outcome_published = False

    def _update_goal_counter(self, distance: float, counter: int) -> int:
        """Update goal counter based on distance."""
        if distance < self.GOAL_RADIUS:
            return min(counter + 1, self.MAX_GOAL_COUNTER)
        elif counter > 0:
            return counter - 1
        return counter

    def _check_and_publish_goal_outcome(
        self, counter: int, message: str, outcome_number: int
    ) -> int:
        """Check if goal threshold reached and publish outcome."""
        if counter == self.MAX_GOAL_COUNTER:
            if self.PRINT_OUTCOME:
                self.get_logger().info(message)

            # Reset counter to avoid repeated triggers
            counter = 5

            if not self.outcome_published:
                self._publish_outcome(outcome_number)

        return counter

    def _publish_outcome(self, outcome_number: int) -> None:
        """Publish goal outcome message."""
        msg = StampedInt32()
        msg.data.data = outcome_number
        msg.header.stamp = self.get_clock().now().to_msg()
        self.outcome_publisher.publish(msg)
        self.outcome_published = True
