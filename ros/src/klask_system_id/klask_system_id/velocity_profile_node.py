"""Velocity profile trajectory node for Klask robot system identification.

Drives pegs through config-defined geometric patterns at randomized velocities.
Patterns are either lines (bounce between two endpoints) or circles (open-loop
circular velocity profile). Boundary deceleration is disabled during runs for
raw trajectories. Records rosbags per run for actuator model fitting.
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


PARAMS_PER_PATTERN = 4


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
        """Initialize the node, declare parameters, and set up publishers/subscribers."""
        super().__init__("velocity_profile_node")

        # === Declare parameters ===
        self.declare_parameter("player", "both")
        self.declare_parameter("pattern_names", ["x_line"])
        self.declare_parameter("pattern_types", ["line"])
        self.declare_parameter("pattern_params", [0.05, 0.16, 0.18, 0.16])
        self.declare_parameter("runs_per_pattern", 6)
        self.declare_parameter("run_duration", 30.0)
        self.declare_parameter("control_rate", 200.0)

        self.declare_parameter("cmd_vel_right_topic", "/cmd_vel/right_player")
        self.declare_parameter("cmd_vel_left_topic", "/cmd_vel/left_player")
        self.declare_parameter("board_state_topic", "/board_state")

        self.declare_parameter("v_min", 0.05)
        self.declare_parameter("v_max", 0.2)
        self.declare_parameter("velocity_mode", "random")  # "random" or "linear"
        self.declare_parameter("validation_runs_per_pattern", 1)

        self.declare_parameter("settle_time", 1.0)
        self.declare_parameter("move_to_home_velocity", 0.05)
        self.declare_parameter("position_tolerance", 0.01)
        self.declare_parameter("circle_correction_gain", 100.0)

        self.declare_parameter("right_player_home_x", 0.31)
        self.declare_parameter("right_player_home_y", 0.16)
        self.declare_parameter("left_player_home_x", 0.11)
        self.declare_parameter("left_player_home_y", 0.16)

        self.declare_parameter("homing_poll_interval", 1.0)
        self.declare_parameter("homing_timeout", 60.0)

        self.declare_parameter("bag_output_dir", "vel_profile_bags")

        # === Load parameters ===
        self.player = self.get_parameter("player").value
        self.runs_per_pattern = self.get_parameter("runs_per_pattern").value
        self.run_duration = self.get_parameter("run_duration").value
        control_rate = self.get_parameter("control_rate").value

        names = list(self.get_parameter("pattern_names").value)
        types = list(self.get_parameter("pattern_types").value)
        raw_params = list(self.get_parameter("pattern_params").value)

        if len(names) != len(types):
            self.get_logger().error("pattern_names and pattern_types must have the same length.")
            raise SystemExit
        expected_len = len(names) * PARAMS_PER_PATTERN
        if len(raw_params) != expected_len:
            self.get_logger().error(
                f"pattern_params must have {expected_len} values "
                f"({PARAMS_PER_PATTERN} per pattern), got {len(raw_params)}."
            )
            raise SystemExit
        if len(names) == 0:
            self.get_logger().error("At least one pattern must be specified.")
            raise SystemExit
        for t in types:
            if t not in ("line", "circle"):
                self.get_logger().error(f"Unknown pattern type '{t}'. Valid: 'line', 'circle'.")
                raise SystemExit

        # Parse into structured list
        self.patterns = []
        for i, (name, ptype) in enumerate(zip(names, types)):
            p = raw_params[i * PARAMS_PER_PATTERN : (i + 1) * PARAMS_PER_PATTERN]
            if ptype == "line":
                self.patterns.append(
                    {
                        "name": name,
                        "type": "line",
                        "min_pos": (p[0], p[1]),
                        "max_pos": (p[2], p[3]),
                    }
                )
            else:
                self.patterns.append(
                    {
                        "name": name,
                        "type": "circle",
                        "center": (p[0], p[1]),
                        "radius": p[2],
                    }
                )

        cmd_vel_right_topic = self.get_parameter("cmd_vel_right_topic").value
        cmd_vel_left_topic = self.get_parameter("cmd_vel_left_topic").value
        board_state_topic = self.get_parameter("board_state_topic").value

        self.v_min = self.get_parameter("v_min").value
        self.v_max = self.get_parameter("v_max").value
        self.velocity_mode = self.get_parameter("velocity_mode").value
        if self.velocity_mode not in ("random", "linear"):
            self.get_logger().error(f"Unknown velocity_mode '{self.velocity_mode}'. " "Valid: 'random', 'linear'.")
            raise SystemExit
        self.validation_runs_per_pattern = self.get_parameter("validation_runs_per_pattern").value
        if self.validation_runs_per_pattern < 0:
            self.get_logger().error("validation_runs_per_pattern must be >= 0.")
            raise SystemExit

        self.settle_time = self.get_parameter("settle_time").value
        self.move_to_home_vel = self.get_parameter("move_to_home_velocity").value
        self.position_tolerance = self.get_parameter("position_tolerance").value
        self.circle_kp = self.get_parameter("circle_correction_gain").value

        self.right_home = (
            self.get_parameter("right_player_home_x").value,
            self.get_parameter("right_player_home_y").value,
        )
        self.left_home = (
            self.get_parameter("left_player_home_x").value,
            self.get_parameter("left_player_home_y").value,
        )

        self.homing_poll_interval = self.get_parameter("homing_poll_interval").value
        self.homing_timeout = self.get_parameter("homing_timeout").value

        self.bag_output_dir = self.get_parameter("bag_output_dir").value

        # Determine which players are active
        self.use_right = self.player in ("both", "right_player")
        self.use_left = self.player in ("both", "left_player")

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
        self.current_validation_run = 0
        self._in_validation_phase = False
        self.current_velocity = 0.0

        # Timing
        self.run_start_time = None
        self.settle_start_time = None

        # Pattern state
        self._right_forward = True
        self._left_forward = True

        # Settling/return action tracking
        self._post_settle_action = "start_first_run"
        self._return_target = None

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
        n_patterns = len(self.patterns)
        train_total = n_patterns * self.runs_per_pattern
        val_total = n_patterns * self.validation_runs_per_pattern
        pattern_summary = [(p["name"], p["type"]) for p in self.patterns]
        self.get_logger().info(
            f"Velocity profile plan: {pattern_summary}, "
            f"{self.runs_per_pattern} train + {self.validation_runs_per_pattern} val "
            f"runs/pattern, "
            f"{self.run_duration:.0f}s/run, "
            f"{train_total} train + {val_total} val = "
            f"{train_total + val_total} total runs, "
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

    def _stop(self):
        self._publish_velocities(0.0, 0.0, 0.0, 0.0)

    def _get_positions(self):
        """Return ((left_x, left_y), (right_x, right_y)) or None."""
        if self.latest_board_state is None:
            return None
        lp = self.latest_board_state.left_peg.position
        rp = self.latest_board_state.right_peg.position
        return ((lp.x, lp.y), (rp.x, rp.y))

    def _select_velocity(self):
        if self.velocity_mode == "linear":
            n = self.runs_per_pattern
            if n <= 1:
                self.current_velocity = self.v_min
            else:
                self.current_velocity = self.v_min + (self.v_max - self.v_min) * self.current_run / (n - 1)
        else:
            self.current_velocity = random.uniform(self.v_min, self.v_max)

    def _select_validation_velocity(self):
        # Validation always uses uniform sampling, independent of velocity_mode,
        # so validation speeds don't coincide with the linear training grid.
        self.current_velocity = random.uniform(self.v_min, self.v_max)

    def _start_bag(self, pattern_name, run_index, is_validation):
        split_dir = "validation" if is_validation else "train"
        out_dir = os.path.join(self.bag_output_dir, split_dir)
        os.makedirs(out_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_path = os.path.join(
            out_dir,
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

    def _get_drive_target(self):
        """Return the target position based on the upcoming pattern.

        For circle patterns the target is a point on the perimeter so the peg
        starts on the circle rather than at center. For everything else it
        falls back to the configured home position.
        """
        if self.current_pattern_index < len(self.patterns):
            pattern = self.patterns[self.current_pattern_index]
            if pattern["type"] == "circle":
                cx, cy = pattern["center"]
                r = pattern["radius"]
                return (cx + r, cy)
        return None

    def _drive_to_target(self, target_override=None):
        """Drive active pegs toward a target. Returns True when all are within tolerance.

        If target_override is set, both pegs drive there. Otherwise they drive
        to their respective configured home positions.
        """
        positions = self._get_positions()
        if positions is None:
            self.get_logger().warn("Waiting for board_state...", throttle_duration_sec=5.0)
            return False

        (lx, ly), (rx, ry) = positions
        right_done = True
        left_done = True
        l_vx, l_vy, r_vx, r_vy = 0.0, 0.0, 0.0, 0.0

        right_target = target_override or self.right_home
        left_target = target_override or self.left_home

        if self.use_right:
            dx = right_target[0] - rx
            dy = right_target[1] - ry
            dist = math.sqrt(dx * dx + dy * dy)
            if dist > self.position_tolerance:
                r_vx = (dx / dist) * self.move_to_home_vel
                r_vy = (dy / dist) * self.move_to_home_vel
                right_done = False

        if self.use_left:
            dx = left_target[0] - lx
            dy = left_target[1] - ly
            dist = math.sqrt(dx * dx + dy * dy)
            if dist > self.position_tolerance:
                l_vx = (dx / dist) * self.move_to_home_vel
                l_vy = (dy / dist) * self.move_to_home_vel
                left_done = False

        self._publish_velocities(l_vx, l_vy, r_vx, r_vy)
        return right_done and left_done

    def _handle_moving_to_home(self):
        if self._drive_to_target(self._get_drive_target()):
            self._stop()
            self.settle_start_time = None
            self.state = ProfileState.SETTLING

    def _handle_returning_home(self):
        if self._drive_to_target(self._return_target):
            self._stop()
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

        if action in ("start_first_run", "start_next_run"):
            self._start_new_run()
        elif action == "all_done":
            self.state = ProfileState.DONE

    def _start_new_run(self):
        pattern = self.patterns[self.current_pattern_index]
        if self._in_validation_phase:
            run_index = self.current_validation_run
            total_runs = self.validation_runs_per_pattern
            self._select_validation_velocity()
            phase_tag = "val"
        else:
            run_index = self.current_run
            total_runs = self.runs_per_pattern
            self._select_velocity()
            phase_tag = "train"
        self._start_bag(pattern["name"], run_index, self._in_validation_phase)
        self.run_start_time = time.monotonic()
        self._right_forward = True
        self._left_forward = True
        self.get_logger().info(
            f"Starting [{phase_tag}] run {run_index + 1}/{total_runs} "
            f"of pattern '{pattern['name']}' ({pattern['type']}), "
            f"v={self.current_velocity:.3f} m/s"
        )
        self.state = ProfileState.RUNNING_PATTERN

    def _advance_run_counters(self):
        """Advance counters after a run completes; pick next phase/pattern."""
        if self._in_validation_phase:
            self.current_validation_run += 1
            if self.current_validation_run >= self.validation_runs_per_pattern:
                self._finish_pattern()
            else:
                self._post_settle_action = "start_next_run"
        else:
            self.current_run += 1
            if self.current_run >= self.runs_per_pattern:
                if self.validation_runs_per_pattern > 0:
                    self._in_validation_phase = True
                    self.current_validation_run = 0
                    self._post_settle_action = "start_next_run"
                else:
                    self._finish_pattern()
            else:
                self._post_settle_action = "start_next_run"

    def _finish_pattern(self):
        """Reset counters and advance to the next pattern (or mark all done)."""
        self.current_run = 0
        self.current_validation_run = 0
        self._in_validation_phase = False
        self.current_pattern_index += 1
        if self.current_pattern_index >= len(self.patterns):
            self._post_settle_action = "all_done"
        else:
            self._post_settle_action = "start_first_run"

    # ── RUNNING_PATTERN ───────────────────────────────────────

    def _handle_running_pattern(self):
        now = time.monotonic()

        # Check run duration
        if now - self.run_start_time >= self.run_duration:
            self.get_logger().info("Run duration elapsed.")
            self._stop()
            self._stop_bag()
            self._advance_run_counters()
            # Drive to the next pattern's start position
            self._return_target = self._get_drive_target()
            self.state = ProfileState.RETURNING_HOME
            return

        pattern = self.patterns[self.current_pattern_index]
        if pattern["type"] == "line":
            self._run_line_pattern(pattern)
        else:
            self._run_circle_pattern(pattern)

    # ── Pattern implementations ───────────────────────────────

    def _run_line_pattern(self, pattern):
        """Drive peg(s) back and forth between min_pos and max_pos."""
        positions = self._get_positions()
        if positions is None:
            return

        (lx, ly), (rx, ry) = positions
        min_pos = pattern["min_pos"]
        max_pos = pattern["max_pos"]

        # Direction unit vector along the line (min → max)
        dx = max_pos[0] - min_pos[0]
        dy = max_pos[1] - min_pos[1]
        length = math.sqrt(dx * dx + dy * dy)
        if length < 1e-6:
            return
        dir_x = dx / length
        dir_y = dy / length

        v = self.current_velocity
        r_vx, r_vy = 0.0, 0.0
        l_vx, l_vy = 0.0, 0.0

        if self.use_right:
            if self._right_forward:
                # Past max_pos when dot(pos - max_pos, dir) >= 0
                if (rx - max_pos[0]) * dir_x + (ry - max_pos[1]) * dir_y >= 0:
                    self._right_forward = False
            else:
                # Past min_pos when dot(pos - min_pos, dir) <= 0
                if (rx - min_pos[0]) * dir_x + (ry - min_pos[1]) * dir_y <= 0:
                    self._right_forward = True
            sign = 1.0 if self._right_forward else -1.0
            r_vx = sign * dir_x * v
            r_vy = sign * dir_y * v

        if self.use_left:
            if self._left_forward:
                if (lx - max_pos[0]) * dir_x + (ly - max_pos[1]) * dir_y >= 0:
                    self._left_forward = False
            else:
                if (lx - min_pos[0]) * dir_x + (ly - min_pos[1]) * dir_y <= 0:
                    self._left_forward = True
            sign = 1.0 if self._left_forward else -1.0
            l_vx = sign * dir_x * v
            l_vy = sign * dir_y * v

        self._publish_velocities(l_vx, l_vy, r_vx, r_vy)

    def _run_circle_pattern(self, pattern):
        """Drive peg(s) around a circle using position feedback.

        At each tick the peg's angle and distance from center are computed.
        Tangential velocity (v) drives the peg around the circle, while a
        proportional radial correction keeps it at the desired radius.
        """
        positions = self._get_positions()
        if positions is None:
            return

        cx, cy = pattern["center"]
        radius = pattern["radius"]
        v = self.current_velocity

        if radius < 1e-6:
            return

        (lx, ly), (rx, ry) = positions
        r_vx, r_vy = 0.0, 0.0
        l_vx, l_vy = 0.0, 0.0

        if self.use_right:
            r_vx, r_vy = self._circle_velocity(rx, ry, cx, cy, radius, v)

        if self.use_left:
            l_vx, l_vy = self._circle_velocity(lx, ly, cx, cy, radius, v)

        self._publish_velocities(l_vx, l_vy, r_vx, r_vy)

    def _circle_velocity(self, px, py, cx, cy, radius, v):
        """Compute velocity for one peg to track a circle.

        Returns (vx, vy) combining tangential drive and radial correction.
        """
        dx = px - cx
        dy = py - cy
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < 1e-6:
            # Peg is at center — push outward in +x to get on the circle
            return (self.circle_kp * v * radius, 0.0)

        # Radial unit vector (center → peg)
        rad_x = dx / dist
        rad_y = dy / dist

        # Tangential unit vector (90° CCW from radial)
        tan_x = -rad_y
        tan_y = rad_x

        # Tangential: drive around the circle at speed v
        # Radial: proportional correction scaled by v so that tracking
        # tightness is consistent across different velocity magnitudes
        radial_error = radius - dist
        vr = self.circle_kp * v * radial_error

        vx = v * tan_x + vr * rad_x
        vy = v * tan_y + vr * rad_y

        return (vx, vy)

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
