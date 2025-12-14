"""ROS2 node for ball and peg state estimation from processed camera images."""

import cv2
import time
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import UInt64
from klask_interfaces.msg import StampedPolygon, StampedInt32, State
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

from .kalman_filter import KalmanFilter
from .utils import create_point_from_list
from .debug import draw_object_with_velocity, plot_image
from .board_state import BoardState


class StateEstimatorNode(Node):
    """ROS2 node for estimating ball and peg positions from camera images."""

    # Debug/Display settings
    SHOW_IMAGE = True
    PRINT_OUTCOME = False

    # Goal detection constants
    GOAL_RADIUS = 22
    GOAL_HYST_COUNTER = 30  # Frames required before confirming goal

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

        # Board Status
        self.board_state: BoardState = BoardState.UNKNOWN

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
                self.board_state |= BoardState.READY
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

    def _check_goal(self) -> None:
        """Check if ball or peg has scored and publish outcome."""

        data = [
            (self.ball_position, self.left_goal, self.ball_in_left_goal_counter),
            (self.ball_position, self.right_goal, self.ball_in_right_goal_counter),
            (self.left_peg_position, self.left_goal, self.peg_in_left_goal_counter),
            (self.right_peg_position, self.right_goal, self.peg_in_right_goal_counter),
        ]
        (
            self.ball_in_left_goal_counter,
            self.ball_in_right_goal_counter,
            self.peg_in_left_goal_counter,
            self.peg_in_right_goal_counter,
        ) = [
            self._update_goal_counter(
                np.array(obj_pos),
                np.array(goal_pos),
                counter,
            )
            for obj_pos, goal_pos, counter in data
        ]

        for counter, flag in [
            (self.ball_in_left_goal_counter, BoardState.BALL_IN_LEFT_GOAL),
            (self.ball_in_right_goal_counter, BoardState.BALL_IN_RIGHT_GOAL),
            (self.peg_in_left_goal_counter, BoardState.PEG_IN_LEFT_GOAL),
            (self.peg_in_right_goal_counter, BoardState.PEG_IN_RIGHT_GOAL),
        ]:
            if counter == self.GOAL_HYST_COUNTER:
                self.board_state |= flag
            elif counter == 0:
                self.board_state &= ~flag

    def _update_goal_counter(
        self, object_pos: np.ndarray, goal_pos: np.ndarray, counter: int
    ) -> int:
        """Update goal counter based on distance."""

        distance = np.linalg.norm(object_pos - goal_pos)
        if distance < self.GOAL_RADIUS:
            return min(counter + 1, self.GOAL_HYST_COUNTER)
        elif counter > 0:
            return counter - 1
        return counter

    def _update_fps(self) -> None:
        """Update FPS calculation (called every second by timer)."""
        self.current_fps = float(self.frame_count)
        self.frame_count = 0

    def _publish_timer_callback(self) -> None:
        """Publish estimated positions/velocities and check for goals."""

        if not (self.board_state & BoardState.READY):
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
        msg.status = UInt64(data=int(self.board_state))

        self.state_publisher.publish(msg)


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
