import cv2
import rclpy
import numpy as np
import time
import os
import apriltag

from geometry_msgs.msg import Polygon, Point32
from klask_interfaces.msg import StampedPolygon, StampedInt32
from rclpy.node import Node


class KalmanFilter:
    """Kalman Filter for tracking position and velocity in 2D space."""
    
    # Collision noise constants
    COLLISION_POSITION_NOISE = 4.0
    COLLISION_VELOCITY_NOISE = 10000.0
    DEFAULT_VELOCITY_THRESHOLD = 1.0
    
    def __init__(
        self,
        process_noise_position: float = 1.0,
        process_noise_velocity: float = 1.0,
        measurement_noise_position: float = 1.0,
        stop_threshold: float = -1.0,
        velocity_threshold: float = DEFAULT_VELOCITY_THRESHOLD,
    ):
        """
        Initialize Kalman Filter.
        
        Args:
            process_noise_position: Process noise for position
            process_noise_velocity: Process noise for velocity
            measurement_noise_position: Measurement noise for position
            stop_threshold: Threshold for detecting stopped motion (-1 to disable)
            velocity_threshold: Minimum velocity magnitude to consider non-zero
        """
        # State vector: [pos_x, pos_y, vel_x, vel_y]
        self.state = np.zeros(4)
        self.stop_threshold = stop_threshold
        self.velocity_threshold = velocity_threshold
        self.previous_position = None

        # Process noise covariance matrix
        self.Q = np.diag([
            process_noise_position,
            process_noise_position,
            process_noise_velocity,
            process_noise_velocity,
        ])
        
        # Measurement noise covariance matrix
        self.R = measurement_noise_position * np.eye(2)

        # Measurement matrix (observe positions only)
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])

        # Initial estimation error covariance
        self.P = np.eye(4)
        
        # State transition matrix (will be updated with dt)
        self.F = np.eye(4)

    def predict(self, dt: float, x_collision: bool = False, y_collision: bool = False) -> None:
        """
        Predict the next state based on the motion model.
        
        Args:
            dt: Time step since last prediction
            x_collision: Whether collision occurred in x-direction
            y_collision: Whether collision occurred in y-direction
        """
        # Update state transition matrix with time step
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # Adjust process noise for collisions
        Q = self.Q.copy()
        if x_collision:
            Q += np.diag([self.COLLISION_POSITION_NOISE, 0.0, self.COLLISION_VELOCITY_NOISE, 0.0])
        if y_collision:
            Q += np.diag([0.0, self.COLLISION_POSITION_NOISE, 0.0, self.COLLISION_VELOCITY_NOISE])

        # Prediction step
        self.state = self.F @ self.state
        self.P = self.F @ self.P @ self.F.T + Q

    def update(self, measurement: tuple[float, float]) -> None:
        """
        Update state with new measurement.
        
        Args:
            measurement: Measured position (x, y)
        """
        z = np.array(measurement)
        y = z - (self.H @ self.state)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        self.state = self.state + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P

        if self.stop_threshold > 0:
            self._check_stop_threshold()

    def get_position(self) -> np.ndarray:
        """Get current estimated position."""
        return self.state[0:2]

    def get_velocity(self) -> list[float]:
        """Get current estimated velocity with threshold filtering."""
        v_x = self.state[2] if abs(self.state[2]) >= self.velocity_threshold else 0.0
        v_y = self.state[3] if abs(self.state[3]) >= self.velocity_threshold else 0.0
        return [v_x, v_y]

    def _check_stop_threshold(self) -> None:
        """Check if object has stopped moving based on position change."""
        current_position = self.get_position()

        if self.previous_position is None:
            self.previous_position = current_position
            return

        displacement = np.linalg.norm(current_position - self.previous_position)

        if displacement < self.stop_threshold:
            self.state[2:4] = 0.0  # Set velocities to zero

        self.previous_position = current_position


def load_calibration_data(filename: str = "calibration_data.npz") -> tuple[np.ndarray, np.ndarray]:
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

    ros2_ws_path = current_dir[:ros2_ws_index + len("ros2_ws")]
    filepath = os.path.join(
        ros2_ws_path, 
        "src/klask_state_estimation_pkg/klask_state_estimation_pkg", 
        filename
    )

    if not os.path.exists(filepath):
        raise FileNotFoundError(f"Calibration file not found at: {filepath}")

    with np.load(filepath) as data:
        return data["mtx"], data["dist"]


def apply_ema_filter(
    current_value: np.ndarray, 
    previous_value: np.ndarray | None, 
    alpha: float = 0.1
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
    image: np.ndarray, 
    target_width: int, 
    target_height: int
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


class BallPegDetection(Node):
    def __init__(self):
        super().__init__("detect_ball_peg")
        # publish the detected ball and peg coordinates
        self.state_publisher = self.create_publisher(
            StampedPolygon, "ball_peg_states", 10
        )
        # publish when ball or peg is in goal
        self.outcome_publisher = self.create_publisher(StampedInt32, "outcome", 10)

        self.timer = self.create_timer(
            1.0 / 1200.0, self.timer_callback
        )  # Timer set for 120 fps
        self.delay_checker_timer = self.create_timer(
            1.0, self.delay_checker_callback
        )  # Timer set for 1 Hz

        self.show_image = True

        self.check_delay = True
        self.check_delay_at = 0
        self.timer_time = time.time()
        self.timer_counter = 0
        self.publisher_counter = 0
        self.delay_values = []
        self.log_count = 0

        # Initialize the KFs for the ball and pegs
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

        # Open the default camera (use the correct index if you have multiple cameras)
        self.cap = cv2.VideoCapture(0)

        # Check if the camera opened successfully
        if not self.cap.isOpened():
            print("Error: Could not open the camera.", flush=True)
            return

        # Set the desired frame rate
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("U", "Y", "V", "Y"))
        self.cap.set(cv2.CAP_PROP_FPS, 120)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, 0.01)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)  # 1280x720
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Display actual FPS
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        print(f"Attempting to capture at 120 FPS. Actual FPS: {actual_fps}", flush=True)

        # Load the camera calibration data
        self.mtx, self.dist = load_calibration_data("calibration_data.npz")

        self.count = 0

        self.options = apriltag.DetectorOptions(families="tag36h11")
        self.detector = apriltag.Detector(self.options)
        self.print_apriltags_not_found = False
        self.print_outcome = False

        self.TAG_SIZE_MM = 20

        # Initialize previous filtered corners for each tag (corners of the detected tag) for EMA
        self.previous_tag_corners = {i: None for i in range(8)}

        self.previous_time = None
        self.dt = None

        self.apriltags_detected_once = False
        self.corner_points_detected_once_printed = False
        self.goals_detected_once_printed = False
        self.goals_detected_this_frame = False

        self.M: np.ndarray = None
        self.width: int = 0
        self.height: int = 0
        self.inset_pixels: int = 0

        self.left_peg_position = None
        self.right_peg_position = None
        self.ball_position = None

        self.left_peg_velocity = None
        self.right_peg_velocity = None
        self.ball_velocity = None

        self.previous_tag_corners_goals = {i: None for i in range(2)}

        self.options = apriltag.DetectorOptions(families="tag36h11")
        self.detector = apriltag.Detector(self.options)

        self.left_goal = [44.44, 188.20]
        self.right_goal = [498.0, 188.50]

        self.edge = np.array([0.0, 530.0, 0.0, 370.0])
        self.distance = 35.0  # if ball is in distance to wall or peg a collision could appear and therefore the process noise should increase

        self.xy_gantry = None
        self.x_gantry = None

        self.magnet = None

        self.goal_radius = 22
        self.goal_perspective_shift = 4

        self.ball_in_left_goal_counter = 0
        self.ball_in_right_goal_counter = 0
        self.peg_in_left_goal_counter = 0
        self.peg_in_right_goal_counter = 0
        self.max_goal_counter = (
            30  # outcome will print after ball/peg has been in goal for XX frames
        )

        self.outcome_published = False  # make sure each goal is only printed once

    def timer_callback(self):
        if self.check_delay:
            self.timer_time = time.time()
            self.check_delay = False
            self.check_delay_at = self.timer_counter

        # Capture one frame
        ret, frame = self.cap.read()

        # If frame is read correctly, ret is True
        if not ret:
            print("Failed to grab frame", flush=True)
            return

        # Rectify frame
        rectified_frame = cv2.undistort(frame, self.mtx, self.dist, None, self.mtx)

        # cv2.imshow("Raw", rectified_frame)
        # cv2.waitKey(1)

        # Call detect_apriltags
        if self.count % 1 == 0:  # adjust modulo if needed
            self.detect_apriltags(rectified_frame)

        self.count += 1

        # Call the process_frame function
        self.process_frame(rectified_frame)

        self.timer_counter += 1

    def detect_apriltags(self, undistorted_frame):
        gray = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2GRAY)
        results = self.detector.detect(gray)

        tag_corners = {i: None for i in range(8)}

        for r in results:
            if r.tag_id in self.previous_tag_corners:
                # Smooth each corner of the tag using EMA
                smoothed_corners = []
                for i, corner in enumerate(r.corners):
                    if self.previous_tag_corners[r.tag_id] is None:
                        smoothed_corner = corner
                    else:
                        if r.tag_id in range(6):
                            smoothed_corner = apply_ema_filter(
                                corner,
                                self.previous_tag_corners[r.tag_id][i],
                                alpha=0.1,
                            )
                        else:  # alpha = 1.0 for gantry tags
                            smoothed_corner = apply_ema_filter(
                                corner,
                                self.previous_tag_corners[r.tag_id][i],
                                alpha=1.0,
                            )
                    smoothed_corners.append(smoothed_corner)

                # Store the smoothed corners
                tag_corners[r.tag_id] = np.array(smoothed_corners, dtype="float32")
                self.previous_tag_corners[r.tag_id] = np.array(
                    smoothed_corners, dtype="float32"
                )

        # Check if all corner tags are detected
        detected_tags = 0

        tag_id_to_name = {
            0: "center right",
            1: "center left",
            2: "top left",
            3: "bottom left",
            4: "top right",
            5: "bottom right",
            6: "xy gantry",
            7: "x gantry",
        }

        # Count how many corner apriltags are detected
        for tag_id in range(2, 6):
            if tag_corners[tag_id] is not None:
                detected_tags += 1
            elif self.print_apriltags_not_found:
                tag_name = tag_id_to_name.get(tag_id, f"unknown tag {tag_id}")
                print(f"Warning: {tag_name} April tag not detected", flush=True)

        if detected_tags == 4:
            # all corner apriltags detected
            self.apriltags_detected_once = True

            if not self.corner_points_detected_once_printed:
                print("All corners detected once", flush=True)
                self.corner_points_detected_once_printed = True

            self.compute_perspective_transform(tag_corners)

        # Count how many goal apriltags are detected
        detected_goals = 0

        for tag_id in range(2):  # Goal AprilTags are 0 and 1
            if tag_corners[tag_id] is not None:
                detected_goals += 1

        self.update_goal_coordinates()
        if detected_goals == 2:
            # Both goal apriltags detected
            self.goals_detected_this_frame = True
        else:
            self.goals_detected_this_frame = False

        if tag_corners[6] is None:
            self.previous_tag_corners[6] = None

        if tag_corners[7] is None:
            self.previous_tag_corners[7] = None

        self.update_gantry_coordinates()
        self.update_magnet_coordinates()

    def process_frame(self, undistorted_frame):
        current_time = time.time()
        # Update time for KF
        if self.previous_time is not None:
            self.dt = current_time - self.previous_time
        else:
            self.dt = 0.01  # Initial guess

        self.previous_time = current_time

        if self.apriltags_detected_once:

            warped = cv2.warpPerspective(
                undistorted_frame,
                self.M,
                (
                    self.width + 2 * self.inset_pixels,
                    self.height + 2 * self.inset_pixels,
                ),
            )

            # Change numbers if apriltags are moved on the phyiscal board
            cropped = warped[
                self.inset_pixels + 16 : -self.inset_pixels - 14,
                self.inset_pixels + 18 : -self.inset_pixels - 18,
            ]  # top, bottom, left, right

            (
                canvas,
                self.left_peg_position,
                self.right_peg_position,
                self.ball_position,
            ) = self.detect_ball_peg(
                cropped,
                target_width=1280,  # 426x240
                target_height=720,
                left_peg_position=self.left_peg_position,
                right_peg_position=self.right_peg_position,
                ball_position=self.ball_position,
                alpha=0.7,  # You can adjust the alpha to control the smoothing
                left_goal=self.left_goal,
                right_goal=self.right_goal,
                goal_radius=self.goal_radius,
            )

            if self.show_image:
                cv2.imshow("Canvas", canvas)
                cv2.waitKey(1)

            # Publish the detected ball and peg coordinates
            self.publish_estimated_positions_and_velocities()

            # Check if a goal is scored
            if (
                self.ball_position is not None
                and self.left_peg_position is not None
                and self.right_peg_position is not None
                and self.left_goal is not None
                and self.right_goal is not None
            ):

                self.check_goal()

    def compute_perspective_transform(self, tag_corners):
        TAG_SIZE_MM = 20

        # Perspective transformation and ball detection
        tag_pixel_size = np.mean(
            [
                np.linalg.norm(tag_corners[2][0] - tag_corners[2][1]),
                np.linalg.norm(tag_corners[3][0] - tag_corners[3][1]),
                np.linalg.norm(tag_corners[4][0] - tag_corners[4][1]),
                np.linalg.norm(tag_corners[5][0] - tag_corners[5][1]),
            ]
        )

        pixels_per_mm = tag_pixel_size / TAG_SIZE_MM

        src_pts = np.array(
            [
                tag_corners[2][3],
                tag_corners[4][2],
                tag_corners[5][2],
                tag_corners[3][0],
            ],
            dtype="float32",
        )

        self.inset_pixels = int(50 * pixels_per_mm)
        self.width = (
            int(
                max(
                    np.linalg.norm(tag_corners[2][3] - tag_corners[4][2]),
                    np.linalg.norm(tag_corners[3][0] - tag_corners[5][2]),
                )
            )
            - 2 * self.inset_pixels
        )

        self.height = (
            int(
                max(
                    np.linalg.norm(tag_corners[2][3] - tag_corners[3][0]),
                    np.linalg.norm(tag_corners[4][2] - tag_corners[5][2]),
                )
            )
            - 2 * self.inset_pixels
        )

        dst_pts = np.array(
            [
                [self.inset_pixels, self.inset_pixels],
                [self.width + self.inset_pixels, self.inset_pixels],
                [self.width + self.inset_pixels, self.height + self.inset_pixels],
                [self.inset_pixels, self.height + self.inset_pixels],
            ],
            dtype="float32",
        )

        self.M = cv2.getPerspectiveTransform(src_pts, dst_pts)

    def detect_ball_peg(
        self,
        cropped,
        target_width=1280,
        target_height=720,
        left_peg_position=None,
        right_peg_position=None,
        ball_position=None,
        alpha=0.1,
        left_goal=None,
        right_goal=None,
        goal_radius=22,
    ):
        # Convert the cropped frame to HSV and apply Gaussian blur
        frame_hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
        frame_hsv_blur = cv2.GaussianBlur(frame_hsv, (7, 7), 0)

        # Mask for the ball (using HSV range)
        low_hsv_ball = (0, 150, 150)
        high_hsv_ball = (45, 255, 255)
        masked_ball = cv2.inRange(frame_hsv_blur, low_hsv_ball, high_hsv_ball)

        # Mask for the peg (using HSV range)
        low_hsv_peg = (0, 0, 0)
        high_hsv_peg = (255, 255, 45)
        masked_peg = cv2.inRange(frame_hsv_blur, low_hsv_peg, high_hsv_peg)

        # Convert masks to 3 channels for visualization
        mask_3_channel_peg = cv2.cvtColor(masked_peg, cv2.COLOR_GRAY2BGR)
        mask_3_channel_ball = cv2.cvtColor(masked_ball, cv2.COLOR_GRAY2BGR)

        # Overlay the peg and ball masks onto the cropped frame
        overlaid_frame = cv2.addWeighted(cropped, 1.0, mask_3_channel_peg, 0.3, 0)
        overlaid_frame = cv2.addWeighted(
            overlaid_frame, 1.0, mask_3_channel_ball, 0.3, 0
        )

        # Split the frame into left and right halves
        height, width = masked_peg.shape
        left_half = masked_peg[:, : width // 2]
        right_half = masked_peg[:, width // 2 :]

        # Find the contours and centroids for the left half (peg)
        left_contours, _ = cv2.findContours(
            left_half, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if left_contours:
            largest_left_contour = max(left_contours, key=cv2.contourArea)
            M_left = cv2.moments(largest_left_contour)
            if M_left["m00"] != 0:
                cX_left = int(M_left["m10"] / M_left["m00"])
                cY_left = int(M_left["m01"] / M_left["m00"])

                # Apply Kalman filter to smooth the centroid position for the left peg
                # Predict and update left peg KF with new position measurement
                self.left_peg_kf.predict(self.dt)
                self.left_peg_kf.update([cX_left, cY_left])

                estimated_left_peg_position = self.left_peg_kf.get_position()
                if self.show_image:
                    estimated_left_peg_velocity = self.left_peg_kf.get_velocity()

                    # Draw the centroid of the left peg as a blue dot
                    cv2.circle(
                        overlaid_frame,
                        (
                            int(estimated_left_peg_position[0]),
                            int(estimated_left_peg_position[1]),
                        ),
                        5,
                        (255, 0, 0),
                        -1,
                    )

                    # Draw the velocity vector of the left peg for debugging
                    # Define the starting point as the estimated position
                    start_point = (
                        int(estimated_left_peg_position[0]),
                        int(estimated_left_peg_position[1]),
                    )

                    factor = 0.5
                    end_point = (
                        int(
                            estimated_left_peg_position[0]
                            + estimated_left_peg_velocity[0] * factor
                        ),
                        int(
                            estimated_left_peg_position[1]
                            + estimated_left_peg_velocity[1] * factor
                        ),
                    )

                    # Draw the velocity vector of the left peg as an arrow
                    cv2.arrowedLine(
                        overlaid_frame,
                        start_point,
                        end_point,
                        (248, 193, 110),
                        2,
                        tipLength=0.3,
                    )

                # Update the previous centroid for the left peg
                left_peg_position = (
                    estimated_left_peg_position[0],
                    estimated_left_peg_position[1],
                )

        # Find the contours and centroids for the right half (peg)
        right_contours, _ = cv2.findContours(
            right_half, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if right_contours:
            largest_right_contour = max(right_contours, key=cv2.contourArea)
            M_right = cv2.moments(largest_right_contour)
            if M_right["m00"] != 0:
                cX_right = (
                    int(M_right["m10"] / M_right["m00"]) + width // 2
                )  # Offset the x-coordinate by half the width
                cY_right = int(M_right["m01"] / M_right["m00"])

                # Apply Kalman filter to smooth the centroid position for the left peg
                # Predict and update left peg KF with new position measurement
                self.right_peg_kf.predict(self.dt)
                self.right_peg_kf.update([cX_right, cY_right])

                estimated_right_peg_position = self.right_peg_kf.get_position()
                if self.show_image:
                    estimated_right_peg_velocity = self.right_peg_kf.get_velocity()

                    # Draw the centroid of the right peg as a green dot
                    cv2.circle(
                        overlaid_frame,
                        (
                            int(estimated_right_peg_position[0]),
                            int(estimated_right_peg_position[1]),
                        ),
                        5,
                        (0, 100, 0),
                        -1,
                    )

                    # Draw the velocity vector of the right peg for debugging
                    # Define the starting point as the estimated position
                    start_point = (
                        int(estimated_right_peg_position[0]),
                        int(estimated_right_peg_position[1]),
                    )

                    factor = 0.5
                    end_point = (
                        int(
                            estimated_right_peg_position[0]
                            + estimated_right_peg_velocity[0] * factor
                        ),
                        int(
                            estimated_right_peg_position[1]
                            + estimated_right_peg_velocity[1] * factor
                        ),
                    )

                    # Draw the velocity vector of the right peg as an arrow
                    cv2.arrowedLine(
                        overlaid_frame,
                        start_point,
                        end_point,
                        (144, 238, 144),
                        2,
                        tipLength=0.3,
                    )

                # Update the previous centroid for the right peg
                right_peg_position = (
                    estimated_right_peg_position[0],
                    estimated_right_peg_position[1],
                )

        # Find contours in the ball mask
        contours, _ = cv2.findContours(
            masked_ball, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        # Calculate the centroid of the largest contour (assuming it's the ball)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cX_ball = int(M["m10"] / M["m00"])
                cY_ball = int(M["m01"] / M["m00"])

                # Apply Kalman filter to smooth the centroid position for the ball
                # Predict and update ball KF with new position measurement

                at_x_edge = (
                    cX_ball < self.edge[0] + self.distance
                    or cX_ball + self.distance > self.edge[1]
                )
                at_y_edge = (
                    cY_ball < self.edge[2] + self.distance
                    or cY_ball + self.distance > self.edge[3]
                )
                close_to_peg = (
                    np.linalg.norm(np.array([cX_ball, cY_ball]) - right_peg_position)
                    < self.distance
                    or np.linalg.norm(np.array([cX_ball, cY_ball]) - left_peg_position)
                    < self.distance
                )

                if close_to_peg or (at_x_edge and at_y_edge):
                    self.ball_kf.predict(self.dt, x_collision=True, y_collision=True)

                elif at_x_edge:
                    self.ball_kf.predict(self.dt, y_collision=True)

                elif at_y_edge:
                    self.ball_kf.predict(self.dt, x_collision=True, y_collision=True)

                else:
                    self.ball_kf.predict(self.dt)

                self.ball_kf.update([cX_ball, cY_ball])

                estimated_ball_position = self.ball_kf.get_position()
                if self.show_image:
                    estimated_ball_velocity = self.ball_kf.get_velocity()

                    # Draw the centroid of the ball as a red dot
                    cv2.circle(
                        overlaid_frame,
                        (
                            int(estimated_ball_position[0]),
                            int(estimated_ball_position[1]),
                        ),
                        5,
                        (0, 0, 255),
                        -1,
                    )

                    # Draw the velocity vector of the ball for debugging
                    # Define the starting point as the estimated position
                    start_point = (
                        int(estimated_ball_position[0]),
                        int(estimated_ball_position[1]),
                    )

                    factor = 0.2
                    end_point = (
                        int(
                            estimated_ball_position[0]
                            + estimated_ball_velocity[0] * factor
                        ),
                        int(
                            estimated_ball_position[1]
                            + estimated_ball_velocity[1] * factor
                        ),
                    )

                    # Draw the velocity vector of the ball as an arrow
                    cv2.arrowedLine(
                        overlaid_frame,
                        start_point,
                        end_point,
                        (0, 165, 255),
                        2,
                        tipLength=0.3,
                    )

                # Update the previous centroid for the ball
                ball_position = (estimated_ball_position[0], estimated_ball_position[1])

            else:
                ball_position = (0, 0)

        else:
            ball_position = (0, 0)

        if self.show_image:
            # Draw the goal coordinates on the image
            if left_goal is not None:
                cv2.circle(
                    overlaid_frame,
                    (int(left_goal[0]), int(left_goal[1])),
                    goal_radius,
                    (140, 255, 0),
                    2,
                )
            if right_goal is not None:
                cv2.circle(
                    overlaid_frame,
                    (int(right_goal[0]), int(right_goal[1])),
                    goal_radius,
                    (140, 255, 0),
                    2,
                )
            # Draw a Point in the middle of each goal
            cv2.circle(
                overlaid_frame,
                (int(left_goal[0]), int(left_goal[1])),
                5,
                (140, 255, 0),
                -1,
            )
            cv2.circle(
                overlaid_frame,
                (int(right_goal[0]), int(right_goal[1])),
                5,
                (140, 255, 0),
                -1,
            )

            # Draw vertical line at x-coordinate of magnet
            decouple_radius = 40
            # if self.magnet is not None:
            #     if self.magnet[1] is None:
            #         cv2.line(overlaid_frame, (int(self.magnet[0]), 0), (int(self.magnet[0]), 400), (0, 0, 255), 2)
            #         cv2.line(overlaid_frame, (int(self.magnet[0]) + decouple_radius, 0), (int(self.magnet[0]) + decouple_radius, 400), (0,165,255), 2)
            #         cv2.line(overlaid_frame, (int(self.magnet[0]) - decouple_radius, 0), (int(self.magnet[0]) - decouple_radius, 400), (0,165,255), 2)
            #     else:
            #         cv2.circle(overlaid_frame, (int(self.magnet[0]), int(self.magnet[1])), 2, (0,0,255), -1)
            #         cv2.circle(overlaid_frame, (int(self.magnet[0]), int(self.magnet[1])), decouple_radius, (0,165,255), 2)

        # Resize the final image to fit within the target size while maintaining aspect ratio
        resized_image, new_w, new_h = resize_with_aspect_ratio(
            overlaid_frame, target_width, target_height
        )

        # Ensure the resized image fits within the target canvas
        if new_w > target_width or new_h > target_height:
            print(
                f"Warning: Resized image ({new_w}x{new_h}) exceeds target canvas size ({target_width}x{target_height})."
            )
            new_w, new_h = min(new_w, target_width), min(new_h, target_height)
            resized_image = cv2.resize(
                resized_image, (new_w, new_h), interpolation=cv2.INTER_AREA
            )

        # Create a black canvas and place the resized image on it
        canvas = np.zeros((target_height, target_width, 3), dtype=np.uint8)
        x_offset = (target_width - new_w) // 2
        y_offset = (target_height - new_h) // 2
        canvas[y_offset : y_offset + new_h, x_offset : x_offset + new_w] = resized_image

        # Return canvas and updated centroids for left peg, right peg, and ball
        return canvas, left_peg_position, right_peg_position, ball_position

    def publish_estimated_positions_and_velocities(self):
        # Prepare and publish ball, peg positions and velocities
        polygon = Polygon()
        vectors = [
            # self.ball_kf.get_position(),
            self.ball_position,
            self.ball_kf.get_velocity(),
            self.left_peg_kf.get_position(),
            self.left_peg_kf.get_velocity(),
            self.right_peg_kf.get_position(),
            self.right_peg_kf.get_velocity(),
            # tuple((4 / 5000) * v for v in self.right_peg_kf.get_velocity())  # Scale each component of the velocity
        ]

        for vector in vectors:
            if vector is not None:
                point = Point32()
                point.x = float(vector[0])
                point.y = float(vector[1])
                point.z = 0.0
                polygon.points.append(point)

        # Append the goal coordinates
        if self.left_goal is not None and self.right_goal is not None:
            left_goal = Point32()
            left_goal.x = self.left_goal[0]
            left_goal.y = self.left_goal[1]
            left_goal.z = 0.0
            polygon.points.append(left_goal)

            right_goal = Point32()
            right_goal.x = self.right_goal[0]
            right_goal.y = self.right_goal[1]
            right_goal.z = 0.0
            polygon.points.append(right_goal)

        if self.magnet is not None:
            magnet = Point32()
            magnet.x = self.magnet[0]
            if self.magnet[1] is not None:
                magnet.y = self.magnet[1]
            else:
                magnet.y = 0.0
            magnet.z = 0.0
            polygon.points.append(magnet)

        if len(polygon.points) == 9:
            stamped_polygon = StampedPolygon()
            stamped_polygon.polygon = polygon
            stamped_polygon.header.stamp = self.get_clock().now().to_msg()
            self.state_publisher.publish(stamped_polygon)

        #        if(self.publisher_counter == self.check_delay_at):
        #            delta_time_ms = (time.time() - self.timer_time) * 1000
        #            print(f"Delta_t ball_peg_detection Node in ms: {delta_time_ms}", flush=True)
        #            if self.publisher_counter > 10:
        #                self.delay_values.append(delta_time_ms)  # Store the delay
        #
        #            self.log_count += 1
        #
        #            if self.log_count == 100:
        #                self.log_delay_statistics()

        self.publisher_counter += 1

    def update_goal_coordinates(self):
        if self.M is None:
            return

        if not self.goals_detected_this_frame:
            return

        if self.previous_tag_corners[0] is not None:
            right_goal_corners = self.previous_tag_corners[0]
            right_goal_centroid = np.mean(right_goal_corners, axis=0)
            right_goal_warped = cv2.perspectiveTransform(
                np.array([[right_goal_centroid]], dtype="float32"), self.M
            )[0][0]
            self.right_goal = right_goal_warped + np.array(
                [
                    self.goal_perspective_shift - 18 - self.inset_pixels,
                    -16 - self.inset_pixels,
                ]
            )

        if self.previous_tag_corners[1] is not None:
            left_goal_corners = self.previous_tag_corners[1]
            left_goal_centroid = np.mean(left_goal_corners, axis=0)
            left_goal_warped = cv2.perspectiveTransform(
                np.array([[left_goal_centroid]], dtype="float32"), self.M
            )[0][0]
            self.left_goal = left_goal_warped - np.array(
                [
                    self.goal_perspective_shift + 18 + self.inset_pixels,
                    16 + self.inset_pixels,
                ]
            )

    def update_gantry_coordinates(self):
        if self.M is None:
            return

        if self.previous_tag_corners[6] is not None:
            xy_gantry_corners = self.previous_tag_corners[6]
            xy_gantry_centroid = np.mean(xy_gantry_corners, axis=0)
            xy_gantry_warped = cv2.perspectiveTransform(
                np.array([[xy_gantry_centroid]], dtype="float32"), self.M
            )[0][0]
            self.xy_gantry = xy_gantry_warped - np.array(
                [18 + self.inset_pixels, 16 + self.inset_pixels]
            )
        else:
            self.xy_gantry = None

        if self.previous_tag_corners[7] is not None:
            x_gantry_corners = self.previous_tag_corners[7]
            x_gantry_centroid = np.mean(x_gantry_corners, axis=0)
            x_gantry_warped = cv2.perspectiveTransform(
                np.array([[x_gantry_centroid]], dtype="float32"), self.M
            )[0][0]
            self.x_gantry = x_gantry_warped - np.array(
                [18 + self.inset_pixels, 16 + self.inset_pixels]
            )
        else:
            self.x_gantry = None

    def update_magnet_coordinates(self):
        xy_shift = 112
        x_shift = 77
        if self.xy_gantry is not None:
            self.magnet = [
                float(self.xy_gantry[0] - xy_shift),
                float(self.xy_gantry[1]),
            ]
        elif self.x_gantry is not None:
            self.magnet = [float(self.x_gantry[0] - x_shift), None]

    def check_goal(self):
        # Check if goals have been detected once
        if self.left_goal is None or self.right_goal is None:
            return

        if not self.goals_detected_once_printed:
            print("Both goals detected once: Ready to play!", flush=True)
            self.goals_detected_once_printed = True

        # Calculate the distances between the ball and the goals
        distance_left_goal_ball = np.linalg.norm(
            np.array(self.ball_position) - self.left_goal
        )
        distance_right_goal_ball = np.linalg.norm(
            np.array(self.ball_position) - self.right_goal
        )

        # Calculate the distances between the pegs and their goals
        distance_left_goal_peg = np.linalg.norm(
            np.array(self.left_peg_position) - self.left_goal
        )
        distance_right_goal_peg = np.linalg.norm(
            np.array(self.right_peg_position) - self.right_goal
        )

        # Check if ball or peg is in a goal for more than xx frames, update the counters
        self.ball_in_left_goal_counter = self.update_goal_counter(
            distance_left_goal_ball, self.ball_in_left_goal_counter
        )
        self.ball_in_right_goal_counter = self.update_goal_counter(
            distance_right_goal_ball, self.ball_in_right_goal_counter
        )

        self.peg_in_left_goal_counter = self.update_goal_counter(
            distance_left_goal_peg, self.peg_in_left_goal_counter
        )
        self.peg_in_right_goal_counter = self.update_goal_counter(
            distance_right_goal_peg, self.peg_in_right_goal_counter
        )

        # Print and publish when goal has been scored
        # Ball in left goal
        if self.ball_in_left_goal_counter == self.max_goal_counter:
            if self.print_outcome:
                print("Ball in left goal!", flush=True)
            self.ball_in_left_goal_counter = 5  # make sure it doesn't print every frame

            if not self.outcome_published:
                self.publish_outcome(0)

        # Ball in right goal
        if self.ball_in_right_goal_counter == self.max_goal_counter:
            if self.print_outcome:
                print("Ball in right goal!", flush=True)
            self.ball_in_right_goal_counter = 5

            if not self.outcome_published:
                self.publish_outcome(1)

        # Peg in left goal
        if self.peg_in_left_goal_counter == self.max_goal_counter:
            if self.print_outcome:
                print("Peg in left goal!", flush=True)
            self.peg_in_left_goal_counter = 5

            if not self.outcome_published:
                self.publish_outcome(2)

        # Peg in right goal
        if self.peg_in_right_goal_counter == self.max_goal_counter:
            if self.print_outcome:
                print("Peg in right goal!", flush=True)
            self.peg_in_right_goal_counter = 5

            if not self.outcome_published:
                self.publish_outcome(3)

        # Reset the outcome printed flag if nothing is in goal for a while
        if (
            self.ball_in_left_goal_counter == 0
            and self.ball_in_right_goal_counter == 0
            and self.peg_in_left_goal_counter == 0
            and self.peg_in_right_goal_counter == 0
        ):

            # Make sure that both goals have been detected once before resetting the outcome
            if self.outcome_published and self.goals_detected_this_frame:
                print("Both goals detected once: Ready to play again!", flush=True)
                self.outcome_published = False

    def update_goal_counter(self, distance, counter):
        if distance < self.goal_radius:
            if counter < self.max_goal_counter:
                counter += 1
        elif counter > 0:
            counter -= 1
        return counter

    def publish_outcome(self, outcome_number):
        msg = StampedInt32()
        msg.data.data = outcome_number
        msg.header.stamp = self.get_clock().now().to_msg()
        self.outcome_publisher.publish(msg)
        self.outcome_published = True

    def delay_checker_callback(self):
        self.check_delay = True

    def log_delay_statistics(self):
        if not self.delay_values:
            print("No delay values to evaluate.", flush=True)
            return

        mean_delay = np.mean(self.delay_values)
        std_delay = np.std(self.delay_values)
        min_delay = np.min(self.delay_values)
        max_delay = np.max(self.delay_values)

        print(f"Delay Statistics:", flush=True)
        print(f"Mean: {mean_delay:.2f} ms", flush=True)
        print(f"Standard Deviation: {std_delay:.2f} ms", flush=True)
        print(f"Min: {min_delay:.2f} ms", flush=True)
        print(f"Max: {max_delay:.2f} ms", flush=True)


def main(args=None):
    rclpy.init(args=args)
    ball_peg_detection = BallPegDetection()
    rclpy.spin(ball_peg_detection)
    ball_peg_detection.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
