"""ROS2 node for ball and peg state estimation from processed camera images."""

import cv2
import time
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import UInt8
from klask_interfaces.msg import StampedPolygon, StampedInt32, State
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

from .kalman_filter import KalmanFilter
from .utils import create_point_from_list
from .debug import draw_object_with_velocity, plot_image


class StateEstimatorNode(Node):
    """ROS2 node for estimating ball and peg positions from camera images."""

    # Debug/Display settings
    SHOW_IMAGE = True
    PRINT_OUTCOME = False

    # Goal detection constants
    GOAL_RADIUS = 22
    MAX_GOAL_COUNTER = 30  # Frames required before confirming goal

    # Collision detection constants
    COLLISION_DISTANCE = 35.0  # Distance threshold for collision detection

    # HSV color range constants for object detection
    BALL_HSV_LOWER = (10, 60, 200)  # Orange ball lower bound
    BALL_HSV_UPPER = (45, 160, 255)  # Orange ball upper bound
    PEG_HSV_LOWER = (90, 150, 0)  # Black peg lower bound
    PEG_HSV_UPPER = (130, 255, 45)  # Black peg upper bound

    # Visualization canvas size
    CANVAS_WIDTH = 1280
    CANVAS_HEIGHT = 720

    # State publishing frequency [Hz]
    PUBLISH_FREQUENCY = 80.0

    # Display update rate
    DISPLAY_UPDATE_INTERVAL = 0.1  # 10Hz display update (100ms between frames)

    def __init__(self):
        super().__init__("state_estimator")

        # Publishers
        self.state_publisher = self.create_publisher(State, "board_state", 10)

        # Subscribers
        self.image_subscription = self.create_subscription(
            CompressedImage, "board_image/compressed", self._image_callback, 10
        )
        self.goal_subscription = self.create_subscription(
            StampedPolygon, "goal_positions", self._goal_callback, 10
        )

        # Timer for state publishing
        self.state_timer = self.create_timer(
            1.0 / self.PUBLISH_FREQUENCY, self._publish_timer_callback
        )

        # CV Bridge for image conversion
        self.bridge = CvBridge()

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

        # Timing
        self.previous_time: float | None = None
        self.dt: float = 0.01

        # FPS tracking
        self.frame_count = 0
        self.current_fps = 0.0
        self.fps_timer = self.create_timer(1.0, self._update_fps)

        # Display throttling
        self.last_display_time = 0.0

        # Object positions
        self.left_peg_position: tuple[float, float] | None = None
        self.right_peg_position: tuple[float, float] | None = None
        self.ball_position: tuple[float, float] | None = None

        # Goal positions (from camera node)
        self.left_goal: list[float] | None = None
        self.right_goal: list[float] | None = None

        # Playing field boundaries [x_min, x_max, y_min, y_max]
        self.edge = np.array([0.0, 530.0, 0.0, 370.0])

        # Goal detection counters
        self.ball_in_left_goal_counter = 0
        self.ball_in_right_goal_counter = 0
        self.peg_in_left_goal_counter = 0
        self.peg_in_right_goal_counter = 0
        self.outcome_published = False

        # Board Status
        self.board_status: int | None = None

        self.get_logger().info("State estimator node started")

    def _goal_callback(self, msg: StampedPolygon) -> None:
        """Callback for receiving goal positions."""
        if len(msg.polygon.points) >= 2:
            self.left_goal = [msg.polygon.points[0].x, msg.polygon.points[0].y]
            self.right_goal = [msg.polygon.points[1].x, msg.polygon.points[1].y]

    def _image_callback(self, msg: CompressedImage) -> None:
        """Callback for receiving compressed board images."""
        try:
            # Convert ROS CompressedImage message to OpenCV image
            cv_image = self.bridge.compressed_imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )

            # Update delta time for Kalman filters
            current_time = time.time()
            self.dt = (
                current_time - self.previous_time
                if self.previous_time is not None
                else 0.01
            )
            self.previous_time = current_time

            # Detect ball and pegs
            (
                self.left_peg_position,
                self.right_peg_position,
                self.ball_position,
            ) = self._detect_ball_peg(
                cv_image,
                left_goal=self.left_goal,
                right_goal=self.right_goal,
            )

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
                self._check_goal()

            # Increment frame counter
            self.frame_count += 1

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def _detect_ball_peg(
        self,
        frame: np.ndarray,
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
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        frame_hsv_blur = cv2.GaussianBlur(frame_hsv, (7, 7), 0)

        # Create masks for ball (orange) and pegs (black)
        masked_ball = cv2.inRange(
            frame_hsv_blur, self.BALL_HSV_LOWER, self.BALL_HSV_UPPER
        )
        masked_peg = cv2.inRange(frame_hsv_blur, self.PEG_HSV_LOWER, self.PEG_HSV_UPPER)

        # Create visualization overlay
        overlaid_frame = self._create_overlay(frame, masked_peg, masked_ball)

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

        # Display image at set rate
        if self.SHOW_IMAGE:
            current_time = time.time()
            if current_time - self.last_display_time >= self.DISPLAY_UPDATE_INTERVAL:
                plot_image(
                    overlaid_frame,
                    left_goal,
                    right_goal,
                    self.GOAL_RADIUS,
                    self.CANVAS_WIDTH,
                    self.CANVAS_HEIGHT,
                    self.current_fps,
                )
                self.last_display_time = current_time

        return left_peg_position, right_peg_position, ball_position

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
            draw_object_with_velocity(
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
            draw_object_with_velocity(
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
            return True, True
        else:
            return False, False

    def _publish_timer_callback(self) -> None:
        """Publish estimated positions/velocities and check for goals."""
        # Publish states
        self._publish_estimated_positions_and_velocities()

    def _publish_estimated_positions_and_velocities(self) -> None:

        if (
            self.ball_kf is None
            or self.left_peg_kf is None
            or self.right_peg_kf is None
            or self.left_goal is None
            or self.right_goal is None
            or self.board_status is None
        ):
            return

        msg = State()
        msg.ball.position = create_point_from_list(self.ball_kf.get_position())
        msg.ball.velocity = create_point_from_list(self.ball_kf.get_velocity())
        msg.left_peg.position = create_point_from_list(self.left_peg_kf.get_position())
        msg.left_peg.velocity = create_point_from_list(self.left_peg_kf.get_velocity())
        msg.right_peg.position = create_point_from_list(
            self.right_peg_kf.get_position()
        )
        msg.right_peg.velocity = create_point_from_list(
            self.right_peg_kf.get_velocity()
        )
        msg.left_goal_pos = create_point_from_list(self.left_goal)
        msg.right_goal_pos = create_point_from_list(self.right_goal)

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = UInt8(data=self.board_status)

        self.state_publisher.publish(msg)

    def _check_goal(self) -> None:
        """Check if ball or peg has scored and publish outcome."""
        if self.left_goal is None or self.right_goal is None:
            return

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
            if self.outcome_published:
                self.get_logger().info("Goals cleared: Ready to play again!")
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

    def _update_fps(self) -> None:
        """Update FPS calculation (called every second by timer)."""
        self.current_fps = float(self.frame_count)
        self.frame_count = 0


def main(args=None):
    rclpy.init(args=args)

    state_estimator = StateEstimatorNode()

    try:
        rclpy.spin(state_estimator)
    except KeyboardInterrupt:
        pass
    finally:
        state_estimator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
