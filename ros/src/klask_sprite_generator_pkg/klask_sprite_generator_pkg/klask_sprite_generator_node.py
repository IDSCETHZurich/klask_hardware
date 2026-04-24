"""Klask sprite generator node for building renderer datasets."""

import json
import math
import os
import sys
import termios
import threading
import time
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from klask_interfaces.action import HomeAndCalibrate
from klask_interfaces.msg import State
from klask_interfaces.srv import IsPlayerHomed
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


class SpriteGeneratorNode(Node):
    """Orchestrates peg motion and image capture for sprite dataset generation."""

    def __init__(self) -> None:
        """Initialize the SpriteGeneratorNode."""
        super().__init__("sprite_generator_node")

        self._declare_parameters()
        self._load_config()

        self._data_lock = threading.Lock()
        self._latest_image: Optional[np.ndarray] = None
        self._latest_image_seq = 0
        self._latest_image_time = 0.0
        self._latest_state: Optional[State] = None

        self._labels: Dict = {"samples": []}
        self._labels_path = ""
        self._output_dirs: Dict[str, str] = {}
        self._sample_id = 0
        self.bridge = CvBridge()

        self.create_subscription(CompressedImage, self.board_image_topic, self._image_callback, 10)
        self.create_subscription(State, self.board_state_topic, self._state_callback, 10)

        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.cmd_vel_left_pub = self.create_publisher(Twist, self.cmd_vel_left_topic, cmd_vel_qos)
        self.cmd_vel_right_pub = self.create_publisher(Twist, self.cmd_vel_right_topic, cmd_vel_qos)

        self.left_calibrate_client = ActionClient(self, HomeAndCalibrate, self.home_and_calibrate_left_action)
        self.right_calibrate_client = ActionClient(self, HomeAndCalibrate, self.home_and_calibrate_right_action)
        self.is_player_homed_client = self.create_client(IsPlayerHomed, self.is_player_homed_service)

        self.worker_thread = threading.Thread(target=self._run_workflow, daemon=True)
        self.worker_thread.start()

    def _declare_parameters(self) -> None:
        self.declare_parameter("board.width_m", 0.42)
        self.declare_parameter("board.height_m", 0.32)

        self.declare_parameter("grid.left_half.offset_left_m", 0.0)
        self.declare_parameter("grid.left_half.offset_right_m", 0.0)
        self.declare_parameter("grid.left_half.offset_top_m", 0.0)
        self.declare_parameter("grid.left_half.offset_bottom_m", 0.0)
        self.declare_parameter("grid.right_half.offset_left_m", 0.0)
        self.declare_parameter("grid.right_half.offset_right_m", 0.0)
        self.declare_parameter("grid.right_half.offset_top_m", 0.0)
        self.declare_parameter("grid.right_half.offset_bottom_m", 0.0)
        self.declare_parameter("grid.points_x", 2)
        self.declare_parameter("grid.points_y", 2)

        self.declare_parameter("motion.homing_velocity_mps", 0.03)
        self.declare_parameter("motion.position_tolerance_m", 0.01)
        self.declare_parameter("motion.goal_timeout_s", 20.0)
        self.declare_parameter("motion.retry_count", 1)
        self.declare_parameter("motion.position_kp", 2.0)
        self.declare_parameter("motion.control_rate_hz", 30.0)

        self.declare_parameter("timing.settle_time_s", 0.4)
        self.declare_parameter("timing.frame_freshness_timeout_s", 2.0)
        self.declare_parameter("timing.calibration_timeout_s", 60.0)
        self.declare_parameter("timing.homing_wait_timeout_s", 90.0)
        self.declare_parameter("timing.homing_poll_interval_s", 1.0)

        self.declare_parameter("startup.run_home_and_calibrate", False)

        self.declare_parameter("topics.board_image_topic", "board_image/compressed")
        self.declare_parameter("topics.board_state_topic", "/board_state")
        self.declare_parameter("topics.is_player_homed_service", "is_player_homed")
        self.declare_parameter("topics.cmd_vel_left_player", "/cmd_vel/left_player")
        self.declare_parameter("topics.cmd_vel_right_player", "/cmd_vel/right_player")
        self.declare_parameter("topics.home_and_calibrate_left_action", "home_and_calibrate_left_player")
        self.declare_parameter("topics.home_and_calibrate_right_action", "home_and_calibrate_right_player")

        self.declare_parameter("capture.background_count", 5)
        self.declare_parameter("capture.ball_count", 12)
        self.declare_parameter("capture.player", "both")
        self.declare_parameter("capture.ball_region.x_min_m", 0.0)
        self.declare_parameter("capture.ball_region.x_max_m", 0.42)
        self.declare_parameter("capture.ball_region.y_min_m", 0.0)
        self.declare_parameter("capture.ball_region.y_max_m", 0.32)
        self.declare_parameter("capture.ball_region.margin_m", 0.02)

        self.declare_parameter("output.root_dir", "sprite_dataset_runs")
        self.declare_parameter("output.run_name", "auto_timestamp")
        self.declare_parameter("output.overwrite_existing", False)

    def _load_config(self) -> None:
        self.board_width_m = float(self.get_parameter("board.width_m").value)
        self.board_height_m = float(self.get_parameter("board.height_m").value)

        self.left_offsets = {
            "left": float(self.get_parameter("grid.left_half.offset_left_m").value),
            "right": float(self.get_parameter("grid.left_half.offset_right_m").value),
            "top": float(self.get_parameter("grid.left_half.offset_top_m").value),
            "bottom": float(self.get_parameter("grid.left_half.offset_bottom_m").value),
        }
        self.right_offsets = {
            "left": float(self.get_parameter("grid.right_half.offset_left_m").value),
            "right": float(self.get_parameter("grid.right_half.offset_right_m").value),
            "top": float(self.get_parameter("grid.right_half.offset_top_m").value),
            "bottom": float(self.get_parameter("grid.right_half.offset_bottom_m").value),
        }
        self.grid_points_x = int(self.get_parameter("grid.points_x").value)
        self.grid_points_y = int(self.get_parameter("grid.points_y").value)

        self.homing_velocity_mps = float(self.get_parameter("motion.homing_velocity_mps").value)
        self.position_tolerance_m = float(self.get_parameter("motion.position_tolerance_m").value)
        self.goal_timeout_s = float(self.get_parameter("motion.goal_timeout_s").value)
        self.retry_count = int(self.get_parameter("motion.retry_count").value)
        self.position_kp = float(self.get_parameter("motion.position_kp").value)
        self.control_rate_hz = float(self.get_parameter("motion.control_rate_hz").value)

        self.settle_time_s = float(self.get_parameter("timing.settle_time_s").value)
        self.frame_freshness_timeout_s = float(self.get_parameter("timing.frame_freshness_timeout_s").value)
        self.calibration_timeout_s = float(self.get_parameter("timing.calibration_timeout_s").value)
        self.homing_wait_timeout_s = float(self.get_parameter("timing.homing_wait_timeout_s").value)
        self.homing_poll_interval_s = float(self.get_parameter("timing.homing_poll_interval_s").value)

        self.run_home_and_calibrate = bool(self.get_parameter("startup.run_home_and_calibrate").value)

        self.board_image_topic = str(self.get_parameter("topics.board_image_topic").value)
        self.board_state_topic = str(self.get_parameter("topics.board_state_topic").value)
        self.is_player_homed_service = str(self.get_parameter("topics.is_player_homed_service").value)
        self.cmd_vel_left_topic = str(self.get_parameter("topics.cmd_vel_left_player").value)
        self.cmd_vel_right_topic = str(self.get_parameter("topics.cmd_vel_right_player").value)
        self.home_and_calibrate_left_action = str(self.get_parameter("topics.home_and_calibrate_left_action").value)
        self.home_and_calibrate_right_action = str(self.get_parameter("topics.home_and_calibrate_right_action").value)

        self.background_count = int(self.get_parameter("capture.background_count").value)
        self.ball_count = int(self.get_parameter("capture.ball_count").value)
        self.capture_player = str(self.get_parameter("capture.player").value).strip().lower()
        self.ball_region = {
            "x_min": float(self.get_parameter("capture.ball_region.x_min_m").value),
            "x_max": float(self.get_parameter("capture.ball_region.x_max_m").value),
            "y_min": float(self.get_parameter("capture.ball_region.y_min_m").value),
            "y_max": float(self.get_parameter("capture.ball_region.y_max_m").value),
            "margin": float(self.get_parameter("capture.ball_region.margin_m").value),
        }

        self.output_root_dir = str(self.get_parameter("output.root_dir").value)
        self.output_run_name = str(self.get_parameter("output.run_name").value)
        self.output_overwrite_existing = bool(self.get_parameter("output.overwrite_existing").value)

        self._validate_config()

        self.config_snapshot = {
            "board": {
                "width_m": self.board_width_m,
                "height_m": self.board_height_m,
            },
            "grid": {
                "points_x": self.grid_points_x,
                "points_y": self.grid_points_y,
                "left_half": self.left_offsets,
                "right_half": self.right_offsets,
            },
            "motion": {
                "homing_velocity_mps": self.homing_velocity_mps,
                "position_tolerance_m": self.position_tolerance_m,
                "goal_timeout_s": self.goal_timeout_s,
                "retry_count": self.retry_count,
                "position_kp": self.position_kp,
                "control_rate_hz": self.control_rate_hz,
            },
            "timing": {
                "settle_time_s": self.settle_time_s,
                "frame_freshness_timeout_s": self.frame_freshness_timeout_s,
                "calibration_timeout_s": self.calibration_timeout_s,
                "homing_wait_timeout_s": self.homing_wait_timeout_s,
                "homing_poll_interval_s": self.homing_poll_interval_s,
            },
            "startup": {
                "run_home_and_calibrate": self.run_home_and_calibrate,
            },
            "capture": {
                "background_count": self.background_count,
                "ball_count": self.ball_count,
                "player": self.capture_player,
                "ball_region": self.ball_region,
            },
            "topics": {
                "board_image_topic": self.board_image_topic,
                "board_state_topic": self.board_state_topic,
                "is_player_homed_service": self.is_player_homed_service,
                "cmd_vel_left_player": self.cmd_vel_left_topic,
                "cmd_vel_right_player": self.cmd_vel_right_topic,
                "home_and_calibrate_left_action": self.home_and_calibrate_left_action,
                "home_and_calibrate_right_action": self.home_and_calibrate_right_action,
            },
            "output": {
                "root_dir": self.output_root_dir,
                "run_name": self.output_run_name,
                "overwrite_existing": self.output_overwrite_existing,
            },
        }

    def _validate_config(self) -> None:
        if self.board_width_m <= 0.0 or self.board_height_m <= 0.0:
            raise ValueError("Board dimensions must be positive")
        if self.grid_points_x <= 0 or self.grid_points_y <= 0:
            raise ValueError("grid.points_x and grid.points_y must be >= 1")
        if self.homing_velocity_mps <= 0.0:
            raise ValueError("motion.homing_velocity_mps must be > 0")
        if self.position_kp <= 0.0:
            raise ValueError("motion.position_kp must be > 0")
        if self.control_rate_hz <= 0.0:
            raise ValueError("motion.control_rate_hz must be > 0")
        if self.position_tolerance_m <= 0.0:
            raise ValueError("motion.position_tolerance_m must be > 0")
        if self.goal_timeout_s <= 0.0 or self.calibration_timeout_s <= 0.0:
            raise ValueError("Action timeouts must be > 0")
        if self.background_count < 0 or self.ball_count < 0:
            raise ValueError("Capture counts must be >= 0")
        if self.frame_freshness_timeout_s <= 0.0:
            raise ValueError("timing.frame_freshness_timeout_s must be > 0")
        if self.homing_wait_timeout_s <= 0.0 or self.homing_poll_interval_s <= 0.0:
            raise ValueError("Homing wait timeout and poll interval must be > 0")
        if self.capture_player not in {"left", "right", "both"}:
            raise ValueError("capture.player must be one of: left, right, both")

    def _image_callback(self, msg: CompressedImage) -> None:
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().warn(f"Failed to decode compressed image frame: {exc}")
            return

        with self._data_lock:
            self._latest_image = frame
            self._latest_image_seq += 1
            self._latest_image_time = time.monotonic()

    def _state_callback(self, msg: State) -> None:
        with self._data_lock:
            self._latest_state = msg

    def _run_workflow(self) -> None:
        try:
            self.get_logger().info("Waiting for action servers...")
            self._wait_for_action_servers()
            self.get_logger().info("Waiting for first image and board state...")
            self._wait_for_initial_data(timeout_s=15.0)

            self._setup_output_dirs()
            self._initialize_labels()

            if self.run_home_and_calibrate:
                self.get_logger().info(f"Running startup calibration for mode: {self.capture_player}")
                if self.capture_player in {"left", "both"}:
                    self._send_home_and_calibrate(
                        self.left_calibrate_client,
                        "left_player",
                        self.calibration_timeout_s,
                    )
                if self.capture_player in {"right", "both"}:
                    self._send_home_and_calibrate(
                        self.right_calibrate_client,
                        "right_player",
                        self.calibration_timeout_s,
                    )
            else:
                self.get_logger().info("Skipping HomeAndCalibrate in sprite node; waiting for players to report homed")
                self._wait_for_homed_players()

            if self.capture_player == "both":
                left_waypoints = self._generate_half_grid("left")
                right_waypoints = self._generate_half_grid("right")

                if len(left_waypoints) != len(right_waypoints):
                    raise RuntimeError("Left and right waypoint count mismatch")

                for index, (left_target, right_target) in enumerate(zip(left_waypoints, right_waypoints)):
                    self.get_logger().info(
                        f"Grid point {index + 1}/{len(left_waypoints)}: "
                        f"left=({left_target[0]:.3f},{left_target[1]:.3f}) "
                        f"right=({right_target[0]:.3f},{right_target[1]:.3f})"
                    )

                    left_result, right_result = self._move_with_retry(
                        left_target=left_target,
                        right_target=right_target,
                    )
                    time.sleep(self.settle_time_s)

                    self._capture_and_record(
                        sample_type="grid",
                        subdir_key="grid",
                        grid_index=index,
                        left_target=left_target,
                        right_target=right_target,
                        left_result=left_result,
                        right_result=right_result,
                    )
            elif self.capture_player == "left":
                left_waypoints = self._generate_half_grid("left")
                for index, left_target in enumerate(left_waypoints):
                    self.get_logger().info(
                        f"Grid point {index + 1}/{len(left_waypoints)}: "
                        f"left=({left_target[0]:.3f},{left_target[1]:.3f})"
                    )
                    left_result, _ = self._move_with_retry(left_target=left_target)
                    time.sleep(self.settle_time_s)
                    self._capture_and_record(
                        sample_type="grid",
                        subdir_key="grid",
                        grid_index=index,
                        left_target=left_target,
                        left_result=left_result,
                    )
            else:
                right_waypoints = self._generate_half_grid("right")
                for index, right_target in enumerate(right_waypoints):
                    self.get_logger().info(
                        f"Grid point {index + 1}/{len(right_waypoints)}: "
                        f"right=({right_target[0]:.3f},{right_target[1]:.3f})"
                    )
                    _, right_result = self._move_with_retry(right_target=right_target)
                    time.sleep(self.settle_time_s)
                    self._capture_and_record(
                        sample_type="grid",
                        subdir_key="grid",
                        grid_index=index,
                        right_target=right_target,
                        right_result=right_result,
                    )

            self._wait_for_enter("Remove both pegs and press Enter to capture background images.")
            for i in range(self.background_count):
                self.get_logger().info(f"Capturing background image {i + 1}/{self.background_count}")
                self._capture_and_record(sample_type="background", subdir_key="background", grid_index=i)

            ball_positions = self._generate_ball_positions(self.ball_count)
            for i, ball_target in enumerate(ball_positions):
                self._wait_for_enter(f"Place the ball at ({ball_target[0]:.3f}, {ball_target[1]:.3f}) and press Enter.")
                self._capture_and_record(
                    sample_type="ball",
                    subdir_key="ball",
                    grid_index=i,
                    ball_target=ball_target,
                )

            self.get_logger().info("Sprite dataset generation completed successfully")
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f"Sprite generation failed: {exc}")
        finally:
            self._write_labels()
            rclpy.try_shutdown()

    def _wait_for_action_servers(self) -> None:
        action_clients = []
        if self.run_home_and_calibrate:
            if self.capture_player in {"left", "both"}:
                action_clients.append((self.left_calibrate_client, self.home_and_calibrate_left_action))
            if self.capture_player in {"right", "both"}:
                action_clients.append((self.right_calibrate_client, self.home_and_calibrate_right_action))

        for client, name in action_clients:
            if not client.wait_for_server(timeout_sec=10.0):
                raise RuntimeError(f"Action server '{name}' not available")

        if not self.is_player_homed_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f"Service '{self.is_player_homed_service}' not available")

    def _wait_for_initial_data(self, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            with self._data_lock:
                has_image = self._latest_image is not None
                has_state = self._latest_state is not None
            if has_image and has_state:
                return
            time.sleep(0.05)
        raise TimeoutError("Timed out waiting for image/state data")

    def _wait_for_homed_players(self) -> None:
        players: List[str] = []
        if self.capture_player in {"left", "both"}:
            players.append("left_player")
        if self.capture_player in {"right", "both"}:
            players.append("right_player")

        for player in players:
            self._wait_until_player_homed(player)

    def _wait_until_player_homed(self, player: str) -> None:
        deadline = time.monotonic() + self.homing_wait_timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            request = IsPlayerHomed.Request()
            request.player = player
            future = self.is_player_homed_client.call_async(request)
            response = self._wait_for_future(future, timeout_s=min(5.0, self.homing_poll_interval_s + 2.0))

            if response is not None and response.is_homed:
                self.get_logger().info(f"{player} is homed (distance={response.distance:.4f} m)")
                return

            if response is not None:
                self.get_logger().info(f"Waiting for {player} to home... distance={response.distance:.4f} m")

            time.sleep(self.homing_poll_interval_s)

        raise TimeoutError(f"Timed out waiting for {player} to report homed")

    def _move_with_retry(
        self,
        left_target: Optional[Tuple[float, float]] = None,
        right_target: Optional[Tuple[float, float]] = None,
    ) -> Tuple[Optional[Dict], Optional[Dict]]:
        last_error: Optional[Exception] = None
        for attempt in range(self.retry_count + 1):
            try:
                left_result, right_result = self._move_players_closed_loop(left_target, right_target)
                return left_result, right_result
            except Exception as exc:  # pylint: disable=broad-except
                last_error = exc
                self.get_logger().warn(f"Move attempt {attempt + 1}/{self.retry_count + 1} failed: {exc}")
                time.sleep(0.1)
        raise RuntimeError(f"Failed to move pegs after retries: {last_error}")

    def _send_home_and_calibrate(
        self,
        client: ActionClient,
        player_name: str,
        timeout_s: float,
    ) -> None:
        goal_future = client.send_goal_async(HomeAndCalibrate.Goal())
        goal_handle = self._wait_for_future(goal_future, timeout_s)
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"HomeAndCalibrate goal rejected for {player_name}")

        result_future = goal_handle.get_result_async()
        result = self._wait_for_future(result_future, timeout_s)
        if result.status != GoalStatus.STATUS_SUCCEEDED or not result.result.success:
            raise RuntimeError(f"HomeAndCalibrate failed for {player_name}: {result.result.message}")

    def _move_players_closed_loop(
        self,
        left_target: Optional[Tuple[float, float]],
        right_target: Optional[Tuple[float, float]],
    ) -> Tuple[Optional[Dict], Optional[Dict]]:
        deadline = time.monotonic() + self.goal_timeout_s
        dt = 1.0 / self.control_rate_hz
        left_done = left_target is None
        right_done = right_target is None
        left_final: Optional[Tuple[float, float]] = None
        right_final: Optional[Tuple[float, float]] = None

        try:
            while rclpy.ok() and time.monotonic() < deadline:
                state = self._get_latest_state_or_raise()

                if left_target is not None and not left_done:
                    left_pos = (state.left_peg.position.x, state.left_peg.position.y)
                    left_final = left_pos
                    lvx, lvy, ldist = self._compute_velocity_to_target(left_pos, left_target)
                    if ldist <= self.position_tolerance_m:
                        left_done = True
                        self._publish_cmd_vel("left_player", 0.0, 0.0)
                    else:
                        self._publish_cmd_vel("left_player", lvx, lvy)

                if right_target is not None and not right_done:
                    right_pos = (state.right_peg.position.x, state.right_peg.position.y)
                    right_final = right_pos
                    rvx, rvy, rdist = self._compute_velocity_to_target(right_pos, right_target)
                    if rdist <= self.position_tolerance_m:
                        right_done = True
                        self._publish_cmd_vel("right_player", 0.0, 0.0)
                    else:
                        self._publish_cmd_vel("right_player", rvx, rvy)

                if left_done and right_done:
                    break

                time.sleep(dt)

            if not (left_done and right_done):
                raise TimeoutError("Closed-loop move timed out")

            left_result = None
            right_result = None
            if left_target is not None:
                left_result = {
                    "success": True,
                    "message": "Reached target with closed-loop cmd_vel",
                    "final_xy": list(left_final) if left_final is not None else None,
                }
            if right_target is not None:
                right_result = {
                    "success": True,
                    "message": "Reached target with closed-loop cmd_vel",
                    "final_xy": list(right_final) if right_final is not None else None,
                }
            return left_result, right_result
        finally:
            if left_target is not None:
                self._publish_cmd_vel("left_player", 0.0, 0.0)
            if right_target is not None:
                self._publish_cmd_vel("right_player", 0.0, 0.0)

    def _compute_velocity_to_target(
        self,
        current_xy: Tuple[float, float],
        target_xy: Tuple[float, float],
    ) -> Tuple[float, float, float]:
        dx = target_xy[0] - current_xy[0]
        dy = target_xy[1] - current_xy[1]
        dist = math.sqrt(dx * dx + dy * dy)
        if dist <= 1e-9:
            return 0.0, 0.0, 0.0

        vx = self.position_kp * dx
        vy = self.position_kp * dy
        speed = math.sqrt(vx * vx + vy * vy)
        if speed > self.homing_velocity_mps:
            scale = self.homing_velocity_mps / speed
            vx *= scale
            vy *= scale

        return vx, vy, dist

    def _publish_cmd_vel(self, player: str, vx: float, vy: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        if player == "left_player":
            self.cmd_vel_left_pub.publish(msg)
        elif player == "right_player":
            self.cmd_vel_right_pub.publish(msg)

    def _wait_for_future(self, future, timeout_s: float):
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            if future.done():
                return future.result()
            time.sleep(0.05)
        raise TimeoutError("Timed out waiting for action future")

    def _capture_and_record(
        self,
        sample_type: str,
        subdir_key: str,
        grid_index: int,
        left_target: Optional[Tuple[float, float]] = None,
        right_target: Optional[Tuple[float, float]] = None,
        left_result: Optional[Dict] = None,
        right_result: Optional[Dict] = None,
        ball_target: Optional[Tuple[float, float]] = None,
    ) -> None:
        previous_seq = self._latest_image_seq
        image, image_seq = self._wait_for_next_fresh_image(previous_seq, self.frame_freshness_timeout_s)
        estimated_state = self._get_latest_state_or_raise()
        sample_name = f"{sample_type}_{self._sample_id:06d}.png"
        image_path = os.path.join(self._output_dirs[subdir_key], sample_name)
        if not cv2.imwrite(image_path, image):
            raise RuntimeError(f"Failed to save image to {image_path}")

        sample_entry = {
            "image_relpath": os.path.relpath(image_path, self._output_dirs["run"]),
        }

        if sample_type == "grid":
            if self.capture_player in {"left", "both"}:
                sample_entry["left_peg_xy"] = [
                    estimated_state.left_peg.position.x,
                    estimated_state.left_peg.position.y,
                ]
            if self.capture_player in {"right", "both"}:
                sample_entry["right_peg_xy"] = [
                    estimated_state.right_peg.position.x,
                    estimated_state.right_peg.position.y,
                ]
        elif sample_type == "ball":
            sample_entry["ball_xy"] = [
                estimated_state.ball.position.x,
                estimated_state.ball.position.y,
            ]

        self._labels["samples"].append(sample_entry)
        self._sample_id += 1
        self._write_labels()

    def _wait_for_next_fresh_image(self, previous_seq: int, timeout_s: float) -> Tuple[np.ndarray, int]:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            with self._data_lock:
                seq = self._latest_image_seq
                image = None if self._latest_image is None else self._latest_image.copy()
                image_time = self._latest_image_time
            if image is not None and seq > previous_seq:
                if time.monotonic() - image_time <= timeout_s:
                    return image, seq
            time.sleep(0.01)
        raise TimeoutError("Timed out waiting for a fresh image frame")

    def _get_latest_state_or_raise(self) -> State:
        with self._data_lock:
            if self._latest_state is None:
                raise RuntimeError("No board_state available")
            return self._latest_state

    def _generate_half_grid(self, side: str) -> List[Tuple[float, float]]:
        half_width = self.board_width_m / 2.0
        if side == "left":
            x_min = 0.0
            x_max = half_width
            offsets = self.left_offsets
        elif side == "right":
            x_min = half_width
            x_max = self.board_width_m
            offsets = self.right_offsets
        else:
            raise ValueError(f"Unknown side '{side}'")

        usable_x_min = x_min + offsets["left"]
        usable_x_max = x_max - offsets["right"]
        usable_y_min = 0.0 + offsets["top"]
        usable_y_max = self.board_height_m - offsets["bottom"]

        if usable_x_min > usable_x_max or usable_y_min > usable_y_max:
            raise ValueError(f"Invalid offsets for {side} half")

        x_values = self._linspace_by_count(usable_x_min, usable_x_max, self.grid_points_x)
        y_values = self._linspace_by_count(usable_y_min, usable_y_max, self.grid_points_y)

        waypoints: List[Tuple[float, float]] = []
        starts_left_to_right = side == "left"

        for row, y in enumerate(y_values):
            if starts_left_to_right:
                row_x = x_values if row % 2 == 0 else list(reversed(x_values))
            else:
                row_x = list(reversed(x_values)) if row % 2 == 0 else x_values
            for x in row_x:
                waypoints.append((x, y))
        return waypoints

    def _generate_ball_positions(self, count: int) -> List[Tuple[float, float]]:
        if count <= 0:
            return []

        x_min = self.ball_region["x_min"] + self.ball_region["margin"]
        x_max = self.ball_region["x_max"] - self.ball_region["margin"]
        y_min = self.ball_region["y_min"] + self.ball_region["margin"]
        y_max = self.ball_region["y_max"] - self.ball_region["margin"]

        if x_min > x_max or y_min > y_max:
            raise ValueError("Invalid ball region and margin configuration")

        cols = int(math.ceil(math.sqrt(count)))
        rows = int(math.ceil(count / cols))

        x_values = self._linspace_by_count(x_min, x_max, cols)
        y_values = self._linspace_by_count(y_min, y_max, rows)

        positions: List[Tuple[float, float]] = []
        for y in y_values:
            for x in x_values:
                positions.append((x, y))
                if len(positions) >= count:
                    return positions
        return positions

    @staticmethod
    def _linspace_by_count(start: float, end: float, count: int) -> List[float]:
        if count <= 1:
            return [start]
        step = (end - start) / float(count - 1)
        return [start + i * step for i in range(count)]

    def _setup_output_dirs(self) -> None:
        run_name = self.output_run_name
        if run_name == "auto_timestamp":
            run_name = datetime.now().strftime("run_%Y%m%d_%H%M%S")

        run_dir = os.path.join(self.output_root_dir, run_name)
        if os.path.exists(run_dir):
            if not self.output_overwrite_existing:
                raise FileExistsError(f"Run directory exists: {run_dir}")
        os.makedirs(run_dir, exist_ok=True)

        grid_dir = os.path.join(run_dir, "grid")
        background_dir = os.path.join(run_dir, "background")
        ball_dir = os.path.join(run_dir, "ball")
        os.makedirs(grid_dir, exist_ok=True)
        os.makedirs(background_dir, exist_ok=True)
        os.makedirs(ball_dir, exist_ok=True)

        self._output_dirs = {
            "run": run_dir,
            "grid": grid_dir,
            "background": background_dir,
            "ball": ball_dir,
        }
        self._labels_path = os.path.join(run_dir, "labels.json")

        self.get_logger().info(f"Writing dataset to {run_dir}")

    def _initialize_labels(self) -> None:
        self._labels = {"samples": []}
        self._write_labels()

    def _write_labels(self) -> None:
        if not self._labels_path:
            return
        with open(self._labels_path, "w", encoding="utf-8") as f:
            json.dump(self._labels, f, indent=2)

    def _wait_for_enter(self, message: str) -> None:
        self.get_logger().info(message)
        try:
            if sys.stdin is not None and sys.stdin.isatty():
                termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)
                input("Press Enter to continue... ")
                return

            # Under ros2 launch, stdin is often not interactive; /dev/tty still captures terminal input.
            with open("/dev/tty", "r", encoding="utf-8") as tty:
                termios.tcflush(tty.fileno(), termios.TCIFLUSH)
                line = tty.readline()
                if line == "":
                    raise EOFError("/dev/tty returned EOF")
        except (EOFError, OSError):
            self.get_logger().warn("No interactive terminal input available, continuing without prompt wait")


def main(args=None):
    """Run the sprite generator node."""
    rclpy.init(args=args)
    node = SpriteGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
