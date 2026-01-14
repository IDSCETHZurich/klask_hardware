"""ROS2 node for viewing compressed image stream from the board camera.

Optionally overlays the latest estimated state (ball/pegs/goals) received from
the `board_state` topic. State is received in engineering units (meters, m/s)
and converted back to pixel coordinates for overlay visualization.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from klask_interfaces.msg import State
from klask_interfaces_py import BoardState
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import time
import numpy as np


class ImageViewer(Node):
    """ROS2 node that subscribes to compressed images and displays them."""

    def __init__(self):
        """Initialize the ImageViewer node."""
        super().__init__("image_viewer")

        # Declare and get parameters
        self.declare_parameter("board_image_topic", "board_image/compressed")
        self.board_image_topic = str(self.get_parameter("board_image_topic").value)

        self.declare_parameter("board_state_topic", "board_state")
        self.board_state_topic = str(self.get_parameter("board_state_topic").value)

        # Enable/disable state overlay (default: true)
        self.declare_parameter("show_state_overlay", True)
        self.show_state_overlay = bool(self.get_parameter("show_state_overlay").value)

        # Enable/disable cmd_vel overlay (default: false)
        self.declare_parameter("show_cmd_vel_overlay", False)
        self.show_cmd_vel_overlay = bool(self.get_parameter("show_cmd_vel_overlay").value)

        # Enable/disable status overlay (default: true)
        self.declare_parameter("show_status_overlay", True)
        self.show_status_overlay = bool(self.get_parameter("show_status_overlay").value)

        self.declare_parameter("cmd_vel_left_player_topic", "cmd_vel/left_player")
        self.cmd_vel_left_player_topic = str(self.get_parameter("cmd_vel_left_player_topic").value)

        self.declare_parameter("cmd_vel_right_player_topic", "cmd_vel/right_player")
        self.cmd_vel_right_player_topic = str(self.get_parameter("cmd_vel_right_player_topic").value)

        # =============================
        # EU to Pixel Conversion Parameters
        # =============================
        # Board dimensions in meters (must match state_estimator)
        self.declare_parameter("board_width_meters", 0.42)
        self.board_width_meters = float(self.get_parameter("board_width_meters").value)

        self.declare_parameter("board_height_meters", 0.32)
        self.board_height_meters = float(self.get_parameter("board_height_meters").value)

        # Current image dimensions (updated each frame)
        self.current_image_width: float = 0.0
        self.current_image_height: float = 0.0

        # Goal visualization radius (pixels)
        self.declare_parameter("goal_radius", 22)
        self.goal_radius = int(self.get_parameter("goal_radius").value)

        # Overlay appearance
        self.declare_parameter("overlay_goal_color", [140, 255, 0])
        self.overlay_goal_color = self._color_param("overlay_goal_color", (140, 255, 0))

        self.declare_parameter("overlay_goal_center_radius", 5)
        self.overlay_goal_center_radius = int(self.get_parameter("overlay_goal_center_radius").value)

        self.declare_parameter("overlay_goal_circle_thickness", 2)
        self.overlay_goal_circle_thickness = int(self.get_parameter("overlay_goal_circle_thickness").value)

        self.declare_parameter("overlay_dot_radius", 5)
        self.overlay_dot_radius = int(self.get_parameter("overlay_dot_radius").value)

        self.declare_parameter("overlay_arrow_thickness", 2)
        self.overlay_arrow_thickness = int(self.get_parameter("overlay_arrow_thickness").value)

        self.declare_parameter("overlay_arrow_tip_length", 0.3)
        self.overlay_arrow_tip_length = float(self.get_parameter("overlay_arrow_tip_length").value)

        # Object colors/scales (BGR)
        self.declare_parameter("overlay_left_peg_dot_color", [255, 0, 0])
        self.overlay_left_peg_dot_color = self._color_param("overlay_left_peg_dot_color", (255, 0, 0))
        self.declare_parameter("overlay_left_peg_arrow_color", [248, 193, 110])
        self.overlay_left_peg_arrow_color = self._color_param("overlay_left_peg_arrow_color", (248, 193, 110))
        self.declare_parameter("overlay_left_peg_velocity_scale", 0.5)
        self.overlay_left_peg_velocity_scale = float(self.get_parameter("overlay_left_peg_velocity_scale").value)

        self.declare_parameter("overlay_right_peg_dot_color", [0, 100, 0])
        self.overlay_right_peg_dot_color = self._color_param("overlay_right_peg_dot_color", (0, 100, 0))
        self.declare_parameter("overlay_right_peg_arrow_color", [144, 238, 144])
        self.overlay_right_peg_arrow_color = self._color_param("overlay_right_peg_arrow_color", (144, 238, 144))
        self.declare_parameter("overlay_right_peg_velocity_scale", 0.5)
        self.overlay_right_peg_velocity_scale = float(self.get_parameter("overlay_right_peg_velocity_scale").value)

        self.declare_parameter("overlay_ball_dot_color", [0, 0, 255])
        self.overlay_ball_dot_color = self._color_param("overlay_ball_dot_color", (0, 0, 255))
        self.declare_parameter("overlay_ball_arrow_color", [0, 165, 255])
        self.overlay_ball_arrow_color = self._color_param("overlay_ball_arrow_color", (0, 165, 255))
        self.declare_parameter("overlay_ball_velocity_scale", 0.2)
        self.overlay_ball_velocity_scale = float(self.get_parameter("overlay_ball_velocity_scale").value)

        # Cmd_vel overlay appearance
        self.declare_parameter("overlay_cmd_vel_left_arrow_color", [255, 100, 255])
        self.overlay_cmd_vel_left_arrow_color = self._color_param("overlay_cmd_vel_left_arrow_color", (255, 100, 255))
        self.declare_parameter("overlay_cmd_vel_left_velocity_scale", 0.5)
        self.overlay_cmd_vel_left_velocity_scale = float(
            self.get_parameter("overlay_cmd_vel_left_velocity_scale").value
        )

        self.declare_parameter("overlay_cmd_vel_right_arrow_color", [255, 255, 100])
        self.overlay_cmd_vel_right_arrow_color = self._color_param("overlay_cmd_vel_right_arrow_color", (255, 255, 100))
        self.declare_parameter("overlay_cmd_vel_right_velocity_scale", 0.5)
        self.overlay_cmd_vel_right_velocity_scale = float(
            self.get_parameter("overlay_cmd_vel_right_velocity_scale").value
        )

        self.declare_parameter("overlay_cmd_vel_arrow_thickness", 3)
        self.overlay_cmd_vel_arrow_thickness = int(self.get_parameter("overlay_cmd_vel_arrow_thickness").value)

        self.declare_parameter("overlay_cmd_vel_arrow_tip_length", 0.3)
        self.overlay_cmd_vel_arrow_tip_length = float(self.get_parameter("overlay_cmd_vel_arrow_tip_length").value)

        # Status overlay appearance
        self.declare_parameter("overlay_status_font_scale", 0.7)
        self.overlay_status_font_scale = float(self.get_parameter("overlay_status_font_scale").value)

        self.declare_parameter("overlay_status_font_thickness", 2)
        self.overlay_status_font_thickness = int(self.get_parameter("overlay_status_font_thickness").value)

        self.declare_parameter("overlay_status_text_color", [0, 255, 255])
        self.overlay_status_text_color = self._color_param("overlay_status_text_color", (0, 255, 255))

        self.declare_parameter("overlay_status_margin", 10)
        self.overlay_status_margin = int(self.get_parameter("overlay_status_margin").value)

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Subscribe to compressed image topic
        self.image_subscription = self.create_subscription(
            CompressedImage, self.board_image_topic, self.image_callback, 10
        )

        # Subscribe to estimated state topic
        self.state_subscription = self.create_subscription(State, self.board_state_topic, self.state_callback, 10)

        self.latest_state: State | None = None

        # Subscribe to cmd_vel topics
        self.cmd_vel_left_subscription = self.create_subscription(
            Twist, self.cmd_vel_left_player_topic, self.cmd_vel_left_callback, 10
        )
        self.cmd_vel_right_subscription = self.create_subscription(
            Twist, self.cmd_vel_right_player_topic, self.cmd_vel_right_callback, 10
        )

        self.latest_cmd_vel_left: Twist | None = None
        self.latest_cmd_vel_right: Twist | None = None

        info_msg = (
            f"Image viewer node started. " f"Subscribing to '{self.board_image_topic}' and '{self.board_state_topic}'"
        )
        if self.show_cmd_vel_overlay:
            info_msg += f", '{self.cmd_vel_left_player_topic}', '{self.cmd_vel_right_player_topic}'"
        self.get_logger().info(info_msg)

        # Window name
        self.window_name = "Board Image Stream"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        # FPS tracking
        self.fps_update_interval = 1.0  # Update FPS display every second
        self.fps_last_update_time = time.time()
        self.fps_frame_count = 0
        self.current_fps = 0.0

    def image_callback(self, msg: CompressedImage) -> None:
        """Handle incoming compressed images."""
        try:
            # Convert ROS CompressedImage message to OpenCV image
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Store current image dimensions for EU-to-pixel conversion
            # (image size can vary each frame due to rotation/cropping)
            height, width = cv_image.shape[:2]
            self.current_image_width = float(width)
            self.current_image_height = float(height)

            # Overlay latest estimated state (if enabled + available)
            if self.show_state_overlay and self.latest_state is not None:
                self._overlay_state(cv_image, self.latest_state)

            # Overlay cmd_vel vectors (if enabled + available)
            if self.show_cmd_vel_overlay and self.latest_state is not None:
                self._overlay_cmd_vel(cv_image, self.latest_state)

            # Overlay status flags (if enabled + available)
            if self.show_status_overlay and self.latest_state is not None:
                self._overlay_status(cv_image, self.latest_state)

            # Update FPS counter
            self._update_fps()

            # Draw FPS on image
            if self.current_fps > 0:
                fps_text = f"FPS: {self.current_fps:.1f}"
                cv2.putText(
                    cv_image,
                    fps_text,
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

            # Display the image
            cv2.imshow(self.window_name, cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def _color_param(self, name: str, default_value: tuple[int, int, int]) -> tuple[int, int, int]:
        """Parse color parameter from ROS parameter.

        Args:
            name: Parameter name.
            default_value: Default RGB color tuple.

        Returns:
            RGB color tuple (B, G, R) for OpenCV.
        """
        value = self.get_parameter(name).value
        try:
            values = [int(v) for v in value]
        except Exception:
            self.get_logger().warn(f"Parameter '{name}' must be a list of 3 ints; using default {list(default_value)}")
            return default_value

        if len(values) != 3:
            self.get_logger().warn(
                f"Parameter '{name}' must have length 3; got {len(values)}. Using default {list(default_value)}"
            )
            return default_value

        return (values[0], values[1], values[2])

    def state_callback(self, msg: State) -> None:
        """Store latest state for overlay."""
        self.latest_state = msg

    def cmd_vel_left_callback(self, msg: Twist) -> None:
        """Store latest left player cmd_vel for overlay."""
        self.latest_cmd_vel_left = msg

    def cmd_vel_right_callback(self, msg: Twist) -> None:
        """Store latest right player cmd_vel for overlay."""
        self.latest_cmd_vel_right = msg

    def _get_meter_to_pixel_x(self) -> float:
        """Get current meter-to-pixel conversion factor for X axis."""
        if self.current_image_width > 0:
            return self.current_image_width / self.board_width_meters
        return 0.0

    def _get_meter_to_pixel_y(self) -> float:
        """Get current meter-to-pixel conversion factor for Y axis."""
        if self.current_image_height > 0:
            return self.current_image_height / self.board_height_meters
        return 0.0

    def _convert_position_to_pixels(self, x_meters: float, y_meters: float) -> tuple[float, float]:
        """Convert position from EU (meters) to pixels."""
        return (
            x_meters * self._get_meter_to_pixel_x(),
            y_meters * self._get_meter_to_pixel_y(),
        )

    def _convert_velocity_to_pixels(self, vx_meters: float, vy_meters: float) -> tuple[float, float]:
        """Convert velocity from EU (m/s) to pixels/s."""
        return (
            vx_meters * self._get_meter_to_pixel_x(),
            vy_meters * self._get_meter_to_pixel_y(),
        )

    def _overlay_state(self, frame: np.ndarray, state: State) -> None:
        """Draw ball/pegs velocity arrows and goals on the frame.

        State is received in engineering units (meters, m/s) and converted
        back to pixel coordinates for visualization overlay.
        """
        # Convert goal positions from EU to pixels
        left_goal_px = self._convert_position_to_pixels(state.left_goal_pos.x, state.left_goal_pos.y)
        right_goal_px = self._convert_position_to_pixels(state.right_goal_pos.x, state.right_goal_pos.y)

        left_goal = (int(left_goal_px[0]), int(left_goal_px[1]))
        right_goal = (int(right_goal_px[0]), int(right_goal_px[1]))

        cv2.circle(
            frame,
            left_goal,
            self.goal_radius,
            self.overlay_goal_color,
            self.overlay_goal_circle_thickness,
        )
        cv2.circle(
            frame,
            left_goal,
            self.overlay_goal_center_radius,
            self.overlay_goal_color,
            -1,
        )
        cv2.circle(
            frame,
            right_goal,
            self.goal_radius,
            self.overlay_goal_color,
            self.overlay_goal_circle_thickness,
        )
        cv2.circle(
            frame,
            right_goal,
            self.overlay_goal_center_radius,
            self.overlay_goal_color,
            -1,
        )

        # Convert object positions and velocities from EU to pixels
        left_peg_pos_px = self._convert_position_to_pixels(state.left_peg.position.x, state.left_peg.position.y)
        left_peg_vel_px = self._convert_velocity_to_pixels(state.left_peg.velocity.x, state.left_peg.velocity.y)
        right_peg_pos_px = self._convert_position_to_pixels(state.right_peg.position.x, state.right_peg.position.y)
        right_peg_vel_px = self._convert_velocity_to_pixels(state.right_peg.velocity.x, state.right_peg.velocity.y)
        ball_pos_px = self._convert_position_to_pixels(state.ball.position.x, state.ball.position.y)
        ball_vel_px = self._convert_velocity_to_pixels(state.ball.velocity.x, state.ball.velocity.y)

        # Draw objects with velocity arrows (now in pixel coordinates)
        self._draw_object_with_velocity(
            frame,
            position=left_peg_pos_px,
            velocity=left_peg_vel_px,
            dot_color=self.overlay_left_peg_dot_color,
            arrow_color=self.overlay_left_peg_arrow_color,
            velocity_scale=self.overlay_left_peg_velocity_scale,
        )
        self._draw_object_with_velocity(
            frame,
            position=right_peg_pos_px,
            velocity=right_peg_vel_px,
            dot_color=self.overlay_right_peg_dot_color,
            arrow_color=self.overlay_right_peg_arrow_color,
            velocity_scale=self.overlay_right_peg_velocity_scale,
        )
        self._draw_object_with_velocity(
            frame,
            position=ball_pos_px,
            velocity=ball_vel_px,
            dot_color=self.overlay_ball_dot_color,
            arrow_color=self.overlay_ball_arrow_color,
            velocity_scale=self.overlay_ball_velocity_scale,
        )

    def _draw_object_with_velocity(
        self,
        frame: np.ndarray,
        position: tuple[float, float],
        velocity: tuple[float, float],
        dot_color: tuple[int, int, int],
        arrow_color: tuple[int, int, int],
        velocity_scale: float,
    ) -> None:
        """Draw object as a dot with velocity arrow overlay.

        Args:
            frame: Image frame to draw on.
            position: Object position in pixels (x, y).
            velocity: Object velocity in pixels/second (vx, vy).
            dot_color: BGR color for object dot.
            arrow_color: BGR color for velocity arrow.
            velocity_scale: Scale factor for velocity arrow length.
        """
        pos_int = (int(position[0]), int(position[1]))
        cv2.circle(frame, pos_int, self.overlay_dot_radius, dot_color, -1)

        end_point = (
            int(position[0] + velocity[0] * velocity_scale),
            int(position[1] + velocity[1] * velocity_scale),
        )
        cv2.arrowedLine(
            frame,
            pos_int,
            end_point,
            arrow_color,
            self.overlay_arrow_thickness,
            tipLength=self.overlay_arrow_tip_length,
        )

    def _overlay_cmd_vel(self, frame: np.ndarray, state: State) -> None:
        """Draw cmd_vel command vectors for left and right players.

        Command velocities are in m/s and are converted to pixel coordinates
        for visualization. Arrows start from the respective peg positions.
        """
        # Draw left player cmd_vel
        if self.latest_cmd_vel_left is not None:
            left_peg_pos_px = self._convert_position_to_pixels(state.left_peg.position.x, state.left_peg.position.y)
            # cmd_vel.linear.x and .y are in m/s
            left_cmd_vel_px = self._convert_velocity_to_pixels(
                self.latest_cmd_vel_left.linear.x,
                self.latest_cmd_vel_left.linear.y,
            )

            pos_int = (int(left_peg_pos_px[0]), int(left_peg_pos_px[1]))
            end_point = (
                int(left_peg_pos_px[0] + left_cmd_vel_px[0] * self.overlay_cmd_vel_left_velocity_scale),
                int(left_peg_pos_px[1] + left_cmd_vel_px[1] * self.overlay_cmd_vel_left_velocity_scale),
            )
            cv2.arrowedLine(
                frame,
                pos_int,
                end_point,
                self.overlay_cmd_vel_left_arrow_color,
                self.overlay_cmd_vel_arrow_thickness,
                tipLength=self.overlay_cmd_vel_arrow_tip_length,
            )

        # Draw right player cmd_vel
        if self.latest_cmd_vel_right is not None:
            right_peg_pos_px = self._convert_position_to_pixels(state.right_peg.position.x, state.right_peg.position.y)
            # cmd_vel.linear.x and .y are in m/s
            right_cmd_vel_px = self._convert_velocity_to_pixels(
                self.latest_cmd_vel_right.linear.x,
                self.latest_cmd_vel_right.linear.y,
            )

            pos_int = (int(right_peg_pos_px[0]), int(right_peg_pos_px[1]))
            end_point = (
                int(right_peg_pos_px[0] + right_cmd_vel_px[0] * self.overlay_cmd_vel_right_velocity_scale),
                int(right_peg_pos_px[1] + right_cmd_vel_px[1] * self.overlay_cmd_vel_right_velocity_scale),
            )
            cv2.arrowedLine(
                frame,
                pos_int,
                end_point,
                self.overlay_cmd_vel_right_arrow_color,
                self.overlay_cmd_vel_arrow_thickness,
                tipLength=self.overlay_cmd_vel_arrow_tip_length,
            )

    def _overlay_status(self, frame: np.ndarray, state: State) -> None:
        """Draw status flags on the frame.

        Status flags are displayed at the bottom of the frame:
        - READY/UNKNOWN: centered at bottom
        - Left player flags (ball/peg in left goal): bottom left corner
        - Right player flags (ball/peg in right goal): bottom right corner

        Only active flags are displayed.
        """
        status_value = state.status.data
        frame_height, frame_width = frame.shape[:2]
        margin = self.overlay_status_margin
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = self.overlay_status_font_scale
        thickness = self.overlay_status_font_thickness
        color = self.overlay_status_text_color

        # Calculate vertical position (bottom of frame, accounting for margin)
        y_pos = frame_height - margin

        # Center status (READY or UNKNOWN)
        if status_value == BoardState.UNKNOWN:
            center_text = "UNKNOWN"
        elif status_value & BoardState.READY:
            center_text = "READY"
        else:
            center_text = None

        if center_text:
            text_size = cv2.getTextSize(center_text, font, font_scale, thickness)[0]
            x_pos = (frame_width - text_size[0]) // 2
            cv2.putText(
                frame,
                center_text,
                (x_pos, y_pos),
                font,
                font_scale,
                color,
                thickness,
                cv2.LINE_AA,
            )

        # Left player status (bottom left corner)
        left_texts = []
        if status_value & BoardState.BALL_IN_LEFT_GOAL:
            left_texts.append("Ball in left goal")
        if status_value & BoardState.PEG_IN_LEFT_GOAL:
            left_texts.append("Peg in left goal")

        for i, text in enumerate(left_texts):
            cv2.putText(
                frame,
                text,
                (margin, y_pos - i * 30),
                font,
                font_scale,
                color,
                thickness,
                cv2.LINE_AA,
            )

        # Right player status (bottom right corner)
        right_texts = []
        if status_value & BoardState.BALL_IN_RIGHT_GOAL:
            right_texts.append("Ball in right goal")
        if status_value & BoardState.PEG_IN_RIGHT_GOAL:
            right_texts.append("Peg in right goal")

        for i, text in enumerate(right_texts):
            text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
            x_pos = frame_width - text_size[0] - margin
            cv2.putText(
                frame,
                text,
                (x_pos, y_pos - i * 30),
                font,
                font_scale,
                color,
                thickness,
                cv2.LINE_AA,
            )

    def _update_fps(self) -> None:
        """Update FPS calculation."""
        self.fps_frame_count += 1
        current_time = time.time()
        elapsed = current_time - self.fps_last_update_time

        if elapsed >= self.fps_update_interval:
            self.current_fps = self.fps_frame_count / elapsed
            self.fps_frame_count = 0
            self.fps_last_update_time = current_time

    def destroy_node(self):
        """Clean up when node is destroyed."""
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    """Run the image viewer node.

    Args:
        args: ROS arguments.
    """
    rclpy.init(args=args)

    image_viewer = ImageViewer()

    try:
        rclpy.spin(image_viewer)
    except KeyboardInterrupt:
        pass
    finally:
        image_viewer.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
