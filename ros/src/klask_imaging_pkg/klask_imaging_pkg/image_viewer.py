"""ROS2 node for viewing compressed image stream from the board camera.

Optionally overlays the latest estimated state (ball/pegs/goals) received from
the `board_state` topic.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from klask_interfaces.msg import State
from cv_bridge import CvBridge
import cv2
import time
import numpy as np


class ImageViewer(Node):
    """ROS2 node that subscribes to compressed images and displays them."""

    def __init__(self):
        super().__init__("image_viewer")

        # Declare and get parameters
        self.declare_parameter("board_image_topic", "board_image/compressed")
        self.board_image_topic = str(self.get_parameter("board_image_topic").value)

        self.declare_parameter("board_state_topic", "board_state")
        self.board_state_topic = str(self.get_parameter("board_state_topic").value)

        # Enable/disable state overlay (default: true)
        self.declare_parameter("show_state_overlay", True)
        self.show_state_overlay = bool(self.get_parameter("show_state_overlay").value)

        # Goal visualization radius (pixels)
        self.declare_parameter("goal_radius", 22)
        self.goal_radius = int(self.get_parameter("goal_radius").value)

        # Overlay appearance
        self.declare_parameter("overlay_goal_color", [140, 255, 0])
        self.overlay_goal_color = self._color_param("overlay_goal_color", (140, 255, 0))

        self.declare_parameter("overlay_goal_center_radius", 5)
        self.overlay_goal_center_radius = int(
            self.get_parameter("overlay_goal_center_radius").value
        )

        self.declare_parameter("overlay_goal_circle_thickness", 2)
        self.overlay_goal_circle_thickness = int(
            self.get_parameter("overlay_goal_circle_thickness").value
        )

        self.declare_parameter("overlay_dot_radius", 5)
        self.overlay_dot_radius = int(self.get_parameter("overlay_dot_radius").value)

        self.declare_parameter("overlay_arrow_thickness", 2)
        self.overlay_arrow_thickness = int(
            self.get_parameter("overlay_arrow_thickness").value
        )

        self.declare_parameter("overlay_arrow_tip_length", 0.3)
        self.overlay_arrow_tip_length = float(
            self.get_parameter("overlay_arrow_tip_length").value
        )

        # Object colors/scales (BGR)
        self.declare_parameter("overlay_left_peg_dot_color", [255, 0, 0])
        self.overlay_left_peg_dot_color = self._color_param(
            "overlay_left_peg_dot_color", (255, 0, 0)
        )
        self.declare_parameter("overlay_left_peg_arrow_color", [248, 193, 110])
        self.overlay_left_peg_arrow_color = self._color_param(
            "overlay_left_peg_arrow_color", (248, 193, 110)
        )
        self.declare_parameter("overlay_left_peg_velocity_scale", 0.5)
        self.overlay_left_peg_velocity_scale = float(
            self.get_parameter("overlay_left_peg_velocity_scale").value
        )

        self.declare_parameter("overlay_right_peg_dot_color", [0, 100, 0])
        self.overlay_right_peg_dot_color = self._color_param(
            "overlay_right_peg_dot_color", (0, 100, 0)
        )
        self.declare_parameter("overlay_right_peg_arrow_color", [144, 238, 144])
        self.overlay_right_peg_arrow_color = self._color_param(
            "overlay_right_peg_arrow_color", (144, 238, 144)
        )
        self.declare_parameter("overlay_right_peg_velocity_scale", 0.5)
        self.overlay_right_peg_velocity_scale = float(
            self.get_parameter("overlay_right_peg_velocity_scale").value
        )

        self.declare_parameter("overlay_ball_dot_color", [0, 0, 255])
        self.overlay_ball_dot_color = self._color_param(
            "overlay_ball_dot_color", (0, 0, 255)
        )
        self.declare_parameter("overlay_ball_arrow_color", [0, 165, 255])
        self.overlay_ball_arrow_color = self._color_param(
            "overlay_ball_arrow_color", (0, 165, 255)
        )
        self.declare_parameter("overlay_ball_velocity_scale", 0.2)
        self.overlay_ball_velocity_scale = float(
            self.get_parameter("overlay_ball_velocity_scale").value
        )

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Subscribe to compressed image topic
        self.image_subscription = self.create_subscription(
            CompressedImage, self.board_image_topic, self.image_callback, 10
        )

        # Subscribe to estimated state topic
        self.state_subscription = self.create_subscription(
            State, self.board_state_topic, self.state_callback, 10
        )

        self.latest_state: State | None = None

        self.get_logger().info(
            "Image viewer node started. "
            f"Subscribing to '{self.board_image_topic}' and '{self.board_state_topic}'"
        )

        # Window name
        self.window_name = "Board Image Stream"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        # FPS tracking
        self.fps_update_interval = 1.0  # Update FPS display every second
        self.fps_last_update_time = time.time()
        self.fps_frame_count = 0
        self.current_fps = 0.0

    def image_callback(self, msg: CompressedImage) -> None:
        """Callback for receiving compressed images."""
        try:
            # Convert ROS CompressedImage message to OpenCV image
            cv_image = self.bridge.compressed_imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )

            # Overlay latest estimated state (if enabled + available)
            if self.show_state_overlay and self.latest_state is not None:
                self._overlay_state(cv_image, self.latest_state)

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

    def _color_param(
        self, name: str, default_value: tuple[int, int, int]
    ) -> tuple[int, int, int]:
        value = self.get_parameter(name).value
        try:
            values = [int(v) for v in value]
        except Exception:
            self.get_logger().warn(
                f"Parameter '{name}' must be a list of 3 ints; using default {list(default_value)}"
            )
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

    def _overlay_state(self, frame: np.ndarray, state: State) -> None:
        """Draw ball/pegs velocity arrows and goals on the frame."""

        # Goals
        left_goal = (int(state.left_goal_pos.x), int(state.left_goal_pos.y))
        right_goal = (int(state.right_goal_pos.x), int(state.right_goal_pos.y))

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

        # Objects
        self._draw_object_with_velocity(
            frame,
            position=(state.left_peg.position.x, state.left_peg.position.y),
            velocity=(state.left_peg.velocity.x, state.left_peg.velocity.y),
            dot_color=self.overlay_left_peg_dot_color,
            arrow_color=self.overlay_left_peg_arrow_color,
            velocity_scale=self.overlay_left_peg_velocity_scale,
        )
        self._draw_object_with_velocity(
            frame,
            position=(state.right_peg.position.x, state.right_peg.position.y),
            velocity=(state.right_peg.velocity.x, state.right_peg.velocity.y),
            dot_color=self.overlay_right_peg_dot_color,
            arrow_color=self.overlay_right_peg_arrow_color,
            velocity_scale=self.overlay_right_peg_velocity_scale,
        )
        self._draw_object_with_velocity(
            frame,
            position=(state.ball.position.x, state.ball.position.y),
            velocity=(state.ball.velocity.x, state.ball.velocity.y),
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
