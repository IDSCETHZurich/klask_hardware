"""Velocity profile trajectory node for Klask robot system identification.

Drives pegs through configurable trajectory patterns (x_direction,
y_direction, diagonal_top, diagonal_bottom, circle) at randomized velocities.
Records rosbags for each run for post-processing actuator model fitting.
"""

import math
import os
import random
import subprocess
import time
from datetime import datetime
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from klask_interfaces.msg import State
from klask_interfaces.srv import IsPlayerHomed
from std_srvs.srv import SetBool


ALL_PATTERNS = ["x_direction", "y_direction", "diagonal_top", "diagonal_bottom", "circle"]


class ProfileState(Enum):
    """State machine states for the velocity profile test."""

    WAITING_FOR_HOMING = auto()
    MOVING_TO_HOME = auto()
    SETTLING = auto()
    RUNNING_PATTERN = auto()
    RETURNING_HOME = auto()
    DONE = auto()


class VelocityProfileNode(Node):
    """Node that drives pegs through trajectory patterns for system identification."""

    def __init__(self):
        """Initialize the node, parameters, publishers, subscribers, and state."""
        super().__init__("velocity_profile_node")

        # === Declare parameters ===
        self.declare_parameter("player", "both")
        self.declare_parameter("pattern", "all")
        self.declare_parameter("runs_per_pattern", 6)
        self.declare_parameter("run_duration", 30.0)
        self.declare_parameter("control_rate", 200.0)

        self.declare_parameter("cmd_vel_right_topic", "/cmd_vel/right_player")
        self.declare_parameter("cmd_vel_left_topic", "/cmd_vel/left_player")
        self.declare_parameter("board_state_topic", "/board_state")

        self.declare_parameter("v_min", 0.05)
        self.declare_parameter("v_max", 0.2)

        self.declare_parameter("circle_radius", 0.01)
        self.declare_parameter("circle_period", 1.0)

        self.declare_parameter("settle_time", 1.0)
        self.declare_parameter("move_to_home_velocity", 0.05)
        self.declare_parameter("position_tolerance", 0.01)
        self.declare_parameter("boundary_margin", 0.015)
        self.declare_parameter("drift_margin", 0.03)

        self.declare_parameter("right_player_x_min", 0.22)
        self.declare_parameter("right_player_x_max", 0.42)
        self.declare_parameter("right_player_y_min", 0.005)
        self.declare_parameter("right_player_y_max", 0.305)
        self.declare_parameter("left_player_x_min", 0.0)
        self.declare_parameter("left_player_x_max", 0.20)
        self.declare_parameter("left_player_y_min", 0.005)
        self.declare_parameter("left_player_y_max", 0.305)

        self.declare_parameter("right_player_home_x", 0.31)
        self.declare_parameter("right_player_home_y", 0.16)
        self.declare_parameter("left_player_home_x", 0.11)
        self.declare_parameter("left_player_home_y", 0.16)

        self.declare_parameter("stall_threshold", 30)
        self.declare_parameter("stall_vel_command_threshold", 0.001)
        self.declare_parameter("stall_vel_measured_threshold", 0.003)

        self.declare_parameter("homing_poll_interval", 1.0)
        self.declare_parameter("homing_timeout", 60.0)

        self.declare_parameter("bag_output_dir", "vel_profile_bags")

        # === Load parameters ===
        self.player = self.get_parameter("player").value
        pattern_param = self.get_parameter("pattern").value
        self.runs_per_pattern = self.get_parameter("runs_per_pattern").value
        self.run_duration = self.get_parameter("run_duration").value
        control_rate = self.get_parameter("control_rate").value

        cmd_vel_right_topic = self.get_parameter("cmd_vel_right_topic").value
        cmd_vel_left_topic = self.get_parameter("cmd_vel_left_topic").value
        board_state_topic = self.get_parameter("board_state_topic").value

        self.v_min = self.get_parameter("v_min").value
        self.v_max = self.get_parameter("v_max").value

        self.circle_radius = self.get_parameter("circle_radius").value
        self.circle_period = self.get_parameter("circle_period").value

        self.settle_time = self.get_parameter("settle_time").value
        self.move_to_home_vel = self.get_parameter("move_to_home_velocity").value
        self.position_tolerance = self.get_parameter("position_tolerance").value
        self.boundary_margin = self.get_parameter("boundary_margin").value
        self.drift_margin = self.get_parameter("drift_margin").value

        self.right_x_min = self.get_parameter("right_player_x_min").value
        self.right_x_max = self.get_parameter("right_player_x_max").value
        self.right_y_min = self.get_parameter("right_player_y_min").value
        self.right_y_max = self.get_parameter("right_player_y_max").value
        self.left_x_min = self.get_parameter("left_player_x_min").value
        self.left_x_max = self.get_parameter("left_player_x_max").value
        self.left_y_min = self.get_parameter("left_player_y_min").value
        self.left_y_max = self.get_parameter("left_player_y_max").value

        self.right_home = (
            self.get_parameter("right_player_home_x").value,
            self.get_parameter("right_player_home_y").value,
        )
        self.left_home = (
            self.get_parameter("left_player_home_x").value,
            self.get_parameter("left_player_home_y").value,
        )

        self.stall_threshold = self.get_parameter("stall_threshold").value
        self.stall_cmd_thresh = self.get_parameter("stall_vel_command_threshold").value
        self.stall_meas_thresh = self.get_parameter("stall_vel_measured_threshold").value

        self.homing_poll_interval = self.get_parameter("homing_poll_interval").value
        self.homing_timeout = self.get_parameter("homing_timeout").value

        self.bag_output_dir = self.get_parameter("bag_output_dir").value

        # Determine which players are active
        self.use_right = self.player in ("both", "right_player")
        self.use_left = self.player in ("both", "left_player")

        # Build pattern list
        if pattern_param == "all":
            self.pattern_list = list(ALL_PATTERNS)
        else:
            if pattern_param not in ALL_PATTERNS:
                self.get_logger().error(f"Unknown pattern '{pattern_param}'. " f"Valid: {ALL_PATTERNS} or 'all'")
                raise SystemExit
            self.pattern_list = [pattern_param]

        # === Publishers ===
        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.right_cmd_pub = None
        self.left_cmd_pub = None
        if self.use_right:
            self.right_cmd_pub = self.create_publisher(Twist, cmd_vel_right_topic, cmd_vel_qos)
        if self.use_left:
            self.left_cmd_pub = self.create_publisher(Twist, cmd_vel_left_topic, cmd_vel_qos)

        # === Subscriber ===
        self.latest_board_state = None
        self.board_state_sub = self.create_subscription(State, board_state_topic, self._board_state_callback, 10)

        # === Service clients ===
        self.homing_client = self.create_client(IsPlayerHomed, "is_player_homed")
        self.decel_client = self.create_client(SetBool, "set_boundary_deceleration")

        # === State machine ===
        self.state = ProfileState.WAITING_FOR_HOMING
        self.current_pattern_index = 0
        self.current_run = 0
        self.current_velocity = 0.0

        # Direction flags for bounce patterns
        self.right_forward_x = True
        self.right_forward_y = True
        self.left_forward_x = True
        self.left_forward_y = True

        # Timing
        self.run_start_time = None
        self.pattern_start_time = None
        self.settle_start_time = None

        # Stall detection
        self.right_stall_counter = 0
        self.left_stall_counter = 0
        self._right_cmd_speed = 0.0
        self._left_cmd_speed = 0.0

        # Settling/return action tracking
        self._post_settle_action = "start_first_run"
        self._post_return_action = "cycle_complete"

        # Homing state
        self.homing_start_time = time.monotonic()
        self.last_homing_poll = 0.0
        self._homing_future = None
        self._players_to_home = []
        self._current_homing_player = None
        self._decel_disabled = False

        # Bag recording
        self._bag_process = None
        self._bag_running = False

        # Build list of players to check homing for
        if self.use_right:
            self._players_to_home.append("right_player")
        if self.use_left:
            self._players_to_home.append("left_player")
        self._homed_players = set()

        # Create control loop timer
        self.control_timer = self.create_timer(1.0 / control_rate, self._control_loop)

        # Log test plan
        total_runs = len(self.pattern_list) * self.runs_per_pattern
        self.get_logger().info(
            f"Velocity profile plan: patterns={self.pattern_list}, "
            f"{self.runs_per_pattern} runs/pattern, "
            f"{self.run_duration:.0f}s/run, "
            f"{total_runs} total runs, "
            f"player={self.player}"
        )

    # ── Callbacks ──────────────────────────────────────────────

    def _board_state_callback(self, msg):
        self.latest_board_state = msg

    # ── Helpers ────────────────────────────────────────────────

    def _publish_velocities(self, left_vx, left_vy, right_vx, right_vy):
        if self.right_cmd_pub is not None:
            twist = Twist()
            twist.linear.x = right_vx
            twist.linear.y = right_vy
            self.right_cmd_pub.publish(twist)
        if self.left_cmd_pub is not None:
            twist = Twist()
            twist.linear.x = left_vx
            twist.linear.y = left_vy
            self.left_cmd_pub.publish(twist)
        self._right_cmd_speed = math.sqrt(right_vx**2 + right_vy**2)
        self._left_cmd_speed = math.sqrt(left_vx**2 + left_vy**2)

    def _stop(self):
        self._publish_velocities(0.0, 0.0, 0.0, 0.0)

    def _get_positions(self):
        """Return ((left_x, left_y), (right_x, right_y)) or None."""
        if self.latest_board_state is None:
            return None
        lp = self.latest_board_state.left_peg.position
        rp = self.latest_board_state.right_peg.position
        return ((lp.x, lp.y), (rp.x, rp.y))

    def _randomize_velocity(self):
        self.current_velocity = random.uniform(self.v_min, self.v_max)

    def _reset_direction_flags(self):
        self.right_forward_x = True
        self.right_forward_y = True
        self.left_forward_x = True
        self.left_forward_y = True

    def _start_bag(self, pattern_name, run_index):
        os.makedirs(self.bag_output_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_path = os.path.join(
            self.bag_output_dir,
            f"vel_profile_{pattern_name}_{timestamp}_run{run_index}",
        )
        self._bag_process = subprocess.Popen(
            ["ros2", "bag", "record", "-a", "-o", bag_path],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        self._bag_running = True
        self.get_logger().info(f"Started bag recording: {bag_path}")

    def _stop_bag(self):
        if self._bag_process is not None:
            self._bag_process.terminate()
            try:
                self._bag_process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self._bag_process.kill()
            self._bag_process = None
        self._bag_running = False
        self.get_logger().info("Stopped bag recording.")

    def _disable_boundary_deceleration(self):
        """Call the set_boundary_deceleration service to disable deceleration."""
        if not self.decel_client.service_is_ready():
            self.get_logger().warn(
                "set_boundary_deceleration service not available, skipping.",
                throttle_duration_sec=5.0,
            )
            return
        req = SetBool.Request()
        req.data = False
        future = self.decel_client.call_async(req)
        future.add_done_callback(self._decel_service_done)

    def _enable_boundary_deceleration(self):
        """Call the set_boundary_deceleration service to re-enable deceleration."""
        if not self.decel_client.service_is_ready():
            return
        req = SetBool.Request()
        req.data = True
        future = self.decel_client.call_async(req)
        future.add_done_callback(self._decel_service_done)

    def _decel_service_done(self, future):
        try:
            result = future.result()
            self.get_logger().info(f"Boundary deceleration service: {result.message}")
        except Exception as e:
            self.get_logger().warn(f"Boundary deceleration service call failed: {e}")

    # ── Stall detection ───────────────────────────────────────

    def _check_stall(self):
        if self.latest_board_state is None:
            return False

        stalled = False

        if self.use_right:
            rv = self.latest_board_state.right_peg.velocity
            right_vel = math.sqrt(rv.x**2 + rv.y**2)
            if self._right_cmd_speed > self.stall_cmd_thresh:
                if right_vel < self.stall_meas_thresh:
                    self.right_stall_counter += 1
                else:
                    self.right_stall_counter = max(0, self.right_stall_counter - 1)
            else:
                self.right_stall_counter = max(0, self.right_stall_counter - 1)
            if self.right_stall_counter >= self.stall_threshold:
                self.get_logger().warn("Right peg stall detected!")
                stalled = True

        if self.use_left:
            lv = self.latest_board_state.left_peg.velocity
            left_vel = math.sqrt(lv.x**2 + lv.y**2)
            if self._left_cmd_speed > self.stall_cmd_thresh:
                if left_vel < self.stall_meas_thresh:
                    self.left_stall_counter += 1
                else:
                    self.left_stall_counter = max(0, self.left_stall_counter - 1)
            else:
                self.left_stall_counter = max(0, self.left_stall_counter - 1)
            if self.left_stall_counter >= self.stall_threshold:
                self.get_logger().warn("Left peg stall detected!")
                stalled = True

        return stalled

    # ── Boundary helpers ──────────────────────────────────────

    def _right_in_field(self, rx, ry):
        m = self.boundary_margin
        return self.right_x_min + m <= rx <= self.right_x_max - m and self.right_y_min + m <= ry <= self.right_y_max - m

    def _left_in_field(self, lx, ly):
        m = self.boundary_margin
        return self.left_x_min + m <= lx <= self.left_x_max - m and self.left_y_min + m <= ly <= self.left_y_max - m

    # ── State machine ─────────────────────────────────────────

    def _control_loop(self):
        if self.state == ProfileState.WAITING_FOR_HOMING:
            self._handle_waiting_for_homing()
        elif self.state == ProfileState.MOVING_TO_HOME:
            self._handle_moving_to_home()
        elif self.state == ProfileState.SETTLING:
            self._handle_settling()
        elif self.state == ProfileState.RUNNING_PATTERN:
            self._handle_running_pattern()
        elif self.state == ProfileState.RETURNING_HOME:
            self._handle_returning_home()
        elif self.state == ProfileState.DONE:
            self._handle_done()

    # ── WAITING_FOR_HOMING ────────────────────────────────────

    def _handle_waiting_for_homing(self):
        elapsed = time.monotonic() - self.homing_start_time
        if elapsed > self.homing_timeout:
            self.get_logger().error(f"Homing timeout after {self.homing_timeout:.0f}s. Aborting.")
            self.state = ProfileState.DONE
            return

        # Check pending async result
        if self._homing_future is not None:
            if self._homing_future.done():
                try:
                    response = self._homing_future.result()
                    if response.is_homed:
                        self._homed_players.add(self._current_homing_player)
                        self.get_logger().info(
                            f"{self._current_homing_player} homed " f"(distance: {response.distance:.4f}m)"
                        )
                    else:
                        self.get_logger().info(
                            f"Waiting for {self._current_homing_player}... " f"distance: {response.distance:.4f}m"
                        )
                except Exception as e:
                    self.get_logger().warn(f"Homing service call failed: {e}")
                self._homing_future = None
                self.last_homing_poll = time.monotonic()

                # Check if all players are homed
                if all(p in self._homed_players for p in self._players_to_home):
                    self.get_logger().info("All players homed. Disabling boundary deceleration.")
                    self._disable_boundary_deceleration()
                    self._decel_disabled = True
                    self._post_settle_action = "start_first_run"
                    self.state = ProfileState.MOVING_TO_HOME
                    return
            else:
                return

        # Rate-limit polling
        if time.monotonic() - self.last_homing_poll < self.homing_poll_interval:
            return

        if not self.homing_client.service_is_ready():
            self.get_logger().warn(
                "is_player_homed service not available yet...",
                throttle_duration_sec=5.0,
            )
            self.last_homing_poll = time.monotonic()
            return

        # Find next player to check
        for p in self._players_to_home:
            if p not in self._homed_players:
                self._current_homing_player = p
                break

        request = IsPlayerHomed.Request()
        request.player = self._current_homing_player
        self._homing_future = self.homing_client.call_async(request)

    # ── MOVING_TO_HOME / RETURNING_HOME ───────────────────────

    def _drive_to_home(self):
        """Drive active pegs toward home. Returns True when all are within tolerance."""
        positions = self._get_positions()
        if positions is None:
            self.get_logger().warn("Waiting for board_state...", throttle_duration_sec=5.0)
            return False

        (lx, ly), (rx, ry) = positions
        right_done = True
        left_done = True
        l_vx, l_vy, r_vx, r_vy = 0.0, 0.0, 0.0, 0.0

        if self.use_right:
            dx = self.right_home[0] - rx
            dy = self.right_home[1] - ry
            dist = math.sqrt(dx * dx + dy * dy)
            if dist > self.position_tolerance:
                r_vx = (dx / dist) * self.move_to_home_vel
                r_vy = (dy / dist) * self.move_to_home_vel
                right_done = False

        if self.use_left:
            dx = self.left_home[0] - lx
            dy = self.left_home[1] - ly
            dist = math.sqrt(dx * dx + dy * dy)
            if dist > self.position_tolerance:
                l_vx = (dx / dist) * self.move_to_home_vel
                l_vy = (dy / dist) * self.move_to_home_vel
                left_done = False

        self._publish_velocities(l_vx, l_vy, r_vx, r_vy)
        return right_done and left_done

    def _handle_moving_to_home(self):
        if self._drive_to_home():
            self._stop()
            self.settle_start_time = None
            self.state = ProfileState.SETTLING

    def _handle_returning_home(self):
        if self._drive_to_home():
            self._stop()
            if self._post_return_action == "run_complete":
                self._stop_bag()
                self.current_run += 1
                if self.current_run >= self.runs_per_pattern:
                    self.current_run = 0
                    self.current_pattern_index += 1
                    if self.current_pattern_index >= len(self.pattern_list):
                        self._post_settle_action = "all_done"
                    else:
                        self._post_settle_action = "start_first_run"
                else:
                    self._post_settle_action = "start_next_run"
            else:
                # cycle_complete: still within run duration
                self._post_settle_action = "resume_after_cycle"
            self.settle_start_time = None
            self.state = ProfileState.SETTLING

    # ── SETTLING ──────────────────────────────────────────────

    def _handle_settling(self):
        self._stop()
        now = time.monotonic()
        if self.settle_start_time is None:
            self.settle_start_time = now
            return
        if now - self.settle_start_time < self.settle_time:
            return

        action = self._post_settle_action

        if action == "start_first_run" or action == "start_next_run":
            self._start_new_run()
        elif action == "resume_after_cycle":
            self.pattern_start_time = None
            self.state = ProfileState.RUNNING_PATTERN
        elif action == "all_done":
            self.state = ProfileState.DONE

    def _start_new_run(self):
        pattern_name = self.pattern_list[self.current_pattern_index]
        self._start_bag(pattern_name, self.current_run)
        self.run_start_time = time.monotonic()
        self.pattern_start_time = None
        self._reset_direction_flags()
        self.right_stall_counter = 0
        self.left_stall_counter = 0
        self.get_logger().info(
            f"Starting run {self.current_run + 1}/{self.runs_per_pattern} " f"of pattern '{pattern_name}'"
        )
        self.state = ProfileState.RUNNING_PATTERN

    # ── RUNNING_PATTERN ───────────────────────────────────────

    def _handle_running_pattern(self):
        now = time.monotonic()

        # First tick of a cycle: randomize velocity
        if self.pattern_start_time is None:
            self.pattern_start_time = now
            self._randomize_velocity()
            self.right_stall_counter = 0
            self.left_stall_counter = 0
            pattern_name = self.pattern_list[self.current_pattern_index]
            self.get_logger().info(f"Pattern '{pattern_name}' cycle, v={self.current_velocity:.3f} m/s")

        # Check run duration
        if now - self.run_start_time >= self.run_duration:
            self.get_logger().info("Run duration elapsed.")
            self._stop()
            self._post_return_action = "run_complete"
            self.state = ProfileState.RETURNING_HOME
            return

        # Execute pattern
        pattern_name = self.pattern_list[self.current_pattern_index]
        boundary_violation = False

        if pattern_name == "x_direction":
            boundary_violation = self._pattern_x_direction()
        elif pattern_name == "y_direction":
            boundary_violation = self._pattern_y_direction()
        elif pattern_name == "diagonal_top":
            boundary_violation = self._pattern_diagonal("top")
        elif pattern_name == "diagonal_bottom":
            boundary_violation = self._pattern_diagonal("bottom")
        elif pattern_name == "circle":
            boundary_violation = self._pattern_circle()

        stall = self._check_stall()

        if boundary_violation or stall:
            reason = "boundary violation" if boundary_violation else "stall detected"
            self.get_logger().info(f"Cycle ended: {reason}. Returning home.")
            self._stop()
            self._post_return_action = "cycle_complete"
            self.state = ProfileState.RETURNING_HOME

    # ── Pattern implementations ───────────────────────────────

    def _pattern_x_direction(self):
        positions = self._get_positions()
        if positions is None:
            return False
        (lx, ly), (rx, ry) = positions

        v = self.current_velocity
        r_vx, r_vy = 0.0, 0.0
        l_vx, l_vy = 0.0, 0.0
        violation = False

        if self.use_right:
            if rx <= self.right_x_min + self.boundary_margin:
                self.right_forward_x = True
            elif rx >= self.right_x_max - self.boundary_margin:
                self.right_forward_x = False
            r_vx = v if self.right_forward_x else -v
            # Y drift check
            if abs(ry - self.right_home[1]) > self.drift_margin:
                violation = True

        if self.use_left:
            if lx <= self.left_x_min + self.boundary_margin:
                self.left_forward_x = True
            elif lx >= self.left_x_max - self.boundary_margin:
                self.left_forward_x = False
            l_vx = v if self.left_forward_x else -v
            if abs(ly - self.left_home[1]) > self.drift_margin:
                violation = True

        self._publish_velocities(l_vx, l_vy, r_vx, r_vy)
        return violation

    def _pattern_y_direction(self):
        positions = self._get_positions()
        if positions is None:
            return False
        (lx, ly), (rx, ry) = positions

        v = self.current_velocity
        r_vx, r_vy = 0.0, 0.0
        l_vx, l_vy = 0.0, 0.0
        violation = False

        if self.use_right:
            if ry <= self.right_y_min + self.boundary_margin:
                self.right_forward_y = True
            elif ry >= self.right_y_max - self.boundary_margin:
                self.right_forward_y = False
            r_vy = v if self.right_forward_y else -v
            if abs(rx - self.right_home[0]) > self.drift_margin:
                violation = True

        if self.use_left:
            if ly <= self.left_y_min + self.boundary_margin:
                self.left_forward_y = True
            elif ly >= self.left_y_max - self.boundary_margin:
                self.left_forward_y = False
            l_vy = v if self.left_forward_y else -v
            if abs(lx - self.left_home[0]) > self.drift_margin:
                violation = True

        self._publish_velocities(l_vx, l_vy, r_vx, r_vy)
        return violation

    def _pattern_diagonal(self, direction):
        positions = self._get_positions()
        if positions is None:
            return False
        (lx, ly), (rx, ry) = positions

        v = self.current_velocity
        r_vx, r_vy = 0.0, 0.0
        l_vx, l_vy = 0.0, 0.0
        violation = False

        # Y sign: "top" means both pegs' Y goes negative when moving inward
        y_sign = -1.0 if direction == "top" else 1.0

        if self.use_right:
            if rx <= self.right_x_min + self.boundary_margin:
                self.right_forward_x = True
            elif rx >= self.right_x_max - self.boundary_margin:
                self.right_forward_x = False
            if self.right_forward_x:
                r_vx = v
                r_vy = -y_sign * v
            else:
                r_vx = -v
                r_vy = y_sign * v
            if not self._right_in_field(rx, ry):
                violation = True

        if self.use_left:
            if lx <= self.left_x_min + self.boundary_margin:
                self.left_forward_x = True
            elif lx >= self.left_x_max - self.boundary_margin:
                self.left_forward_x = False
            if self.left_forward_x:
                l_vx = v
                l_vy = y_sign * v
            else:
                l_vx = -v
                l_vy = -y_sign * v
            if not self._left_in_field(lx, ly):
                violation = True

        self._publish_velocities(l_vx, l_vy, r_vx, r_vy)
        return violation

    def _pattern_circle(self):
        t = time.monotonic() - self.pattern_start_time
        r = self.circle_radius
        T = self.circle_period
        omega_r = r * 2.0 * math.pi / T
        phase = 2.0 * math.pi * t / T

        l_vx = omega_r * math.sin(phase)
        l_vy = omega_r * math.cos(phase)
        r_vx = -omega_r * math.sin(phase)
        r_vy = omega_r * math.cos(phase)

        if not self.use_left:
            l_vx, l_vy = 0.0, 0.0
        if not self.use_right:
            r_vx, r_vy = 0.0, 0.0

        self._publish_velocities(l_vx, l_vy, r_vx, r_vy)

        # Check radial drift from home
        positions = self._get_positions()
        if positions is None:
            return False
        (lx, ly), (rx, ry) = positions

        max_drift = self.circle_radius * 3.0 + self.boundary_margin
        violation = False
        if self.use_right:
            rd = math.sqrt((rx - self.right_home[0]) ** 2 + (ry - self.right_home[1]) ** 2)
            if rd > max_drift:
                violation = True
        if self.use_left:
            ld = math.sqrt((lx - self.left_home[0]) ** 2 + (ly - self.left_home[1]) ** 2)
            if ld > max_drift:
                violation = True
        return violation

    # ── DONE ──────────────────────────────────────────────────

    def _handle_done(self):
        self._stop()
        self._stop_bag()
        if self._decel_disabled:
            self.get_logger().info("Re-enabling boundary deceleration.")
            self._enable_boundary_deceleration()
            self._decel_disabled = False
        self.control_timer.cancel()
        self.get_logger().info("Velocity profile test complete. Shutting down.")
        raise SystemExit


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = VelocityProfileNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node._stop()
        node._stop_bag()
        if node._decel_disabled:
            node._enable_boundary_deceleration()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
