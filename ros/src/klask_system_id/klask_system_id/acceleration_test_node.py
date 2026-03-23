"""Acceleration test node for Klask robot system identification.

Drives the gantry robot in triangle patterns at increasing speeds to
characterize the system's dynamic behavior. Records commanded velocities
and actual positions via /board_state for post-processing analysis.
"""

import math
import os
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


class TestState(Enum):
    """State machine states for the acceleration test."""

    WAITING_FOR_HOMING = auto()
    MOVING_TO_START = auto()
    SETTLING = auto()
    RUNNING_LEG = auto()
    DONE = auto()


class AccelerationTestNode(Node):
    """Node that runs triangle-pattern acceleration tests at increasing speeds."""

    def __init__(self):
        super().__init__("acceleration_test_node")

        # Declare parameters
        self.declare_parameter("player", "left_player")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel/left_player")
        self.declare_parameter("board_state_topic", "/board_state")
        self.declare_parameter("triangle_side_length", 0.10)
        self.declare_parameter("start_position_x", 0.05)
        self.declare_parameter("start_position_y", 0.08)
        self.declare_parameter("min_velocity", 0.10)
        self.declare_parameter("max_velocity", 1.0)
        self.declare_parameter("velocity_increment", 0.05)
        self.declare_parameter("repetitions_per_speed", 5)
        self.declare_parameter("settle_time", 1.0)
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("move_to_start_velocity", 0.05)
        self.declare_parameter("homing_poll_interval", 1.0)
        self.declare_parameter("homing_timeout", 60.0)
        self.declare_parameter("bag_output_dir", "acceleration_test_bags")

        # Get parameter values
        self.player = self.get_parameter("player").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        board_state_topic = self.get_parameter("board_state_topic").value
        self.side_length = self.get_parameter("triangle_side_length").value
        self.start_x = self.get_parameter("start_position_x").value
        self.start_y = self.get_parameter("start_position_y").value
        self.min_velocity = self.get_parameter("min_velocity").value
        self.max_velocity = self.get_parameter("max_velocity").value
        self.velocity_increment = self.get_parameter("velocity_increment").value
        self.repetitions_per_speed = self.get_parameter("repetitions_per_speed").value
        self.settle_time = self.get_parameter("settle_time").value
        control_rate = self.get_parameter("control_rate").value
        self.move_to_start_vel = self.get_parameter("move_to_start_velocity").value
        self.homing_poll_interval = self.get_parameter("homing_poll_interval").value
        self.homing_timeout = self.get_parameter("homing_timeout").value
        self.bag_output_dir = self.get_parameter("bag_output_dir").value

        # Publisher for velocity commands
        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, cmd_vel_qos)

        # Subscriber for board state (position monitoring)
        self.latest_board_state = None
        self.board_state_sub = self.create_subscription(State, board_state_topic, self.board_state_callback, 10)

        # Service client for homing check
        self.homing_client = self.create_client(IsPlayerHomed, "is_player_homed")
        self._homing_future = None

        # State machine
        self.state = TestState.WAITING_FOR_HOMING
        self.current_velocity = self.min_velocity
        self.current_repetition = 0
        self.current_leg = 0  # 0=+X, 1=+Y, 2=diagonal
        self.leg_start_time = None
        self.leg_duration = 0.0
        self.settle_start_time = None
        self.homing_start_time = time.monotonic()
        self.last_homing_poll = 0.0
        self.position_tolerance = 0.01  # 1cm tolerance for move-to-start
        self._bag_process = None
        self._bag_running = False

        # Create control loop timer
        timer_period = 1.0 / control_rate
        self.control_timer = self.create_timer(timer_period, self.control_loop_callback)

        # Log test plan
        num_speeds = int((self.max_velocity - self.min_velocity) / self.velocity_increment) + 1
        total_triangles = num_speeds * self.repetitions_per_speed
        self.get_logger().info(
            f"Acceleration test plan: {num_speeds} speed levels "
            f"({self.min_velocity:.2f} to {self.max_velocity:.2f} m/s, "
            f"step {self.velocity_increment:.2f}), "
            f"{self.repetitions_per_speed} reps each, "
            f"{total_triangles} total triangles, "
            f"side length {self.side_length:.3f}m"
        )

    def board_state_callback(self, msg):
        """Store latest board state for position monitoring."""
        self.latest_board_state = msg

    def publish_velocity(self, vx, vy):
        """Publish a velocity command."""
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        self.cmd_vel_pub.publish(twist)

    def get_peg_position(self):
        """Get current peg position from board state. Returns (x, y) or None."""
        if self.latest_board_state is None:
            return None
        if self.player == "left_player":
            pos = self.latest_board_state.left_peg.position
        else:
            pos = self.latest_board_state.right_peg.position
        return (pos.x, pos.y)

    def _start_bag(self, velocity):
        """Start a new rosbag recording for the given velocity step."""
        os.makedirs(self.bag_output_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_path = os.path.join(self.bag_output_dir, f"accel_test_{timestamp}_v{velocity:.2f}")
        self._bag_process = subprocess.Popen(
            ["ros2", "bag", "record", "-a", "-o", bag_path],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        self._bag_running = True
        self.get_logger().info(f"Started bag recording: {bag_path}")

    def _stop_bag(self):
        """Stop the current rosbag recording."""
        if self._bag_process is not None:
            self._bag_process.terminate()
            try:
                self._bag_process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self._bag_process.kill()
            self._bag_process = None
        self._bag_running = False
        self.get_logger().info("Stopped bag recording.")

    def control_loop_callback(self):
        """Main state machine tick."""
        if self.state == TestState.WAITING_FOR_HOMING:
            self._handle_waiting_for_homing()
        elif self.state == TestState.MOVING_TO_START:
            self._handle_moving_to_start()
        elif self.state == TestState.SETTLING:
            self._handle_settling()
        elif self.state == TestState.RUNNING_LEG:
            self._handle_running_leg()
        elif self.state == TestState.DONE:
            self._handle_done()

    def _handle_waiting_for_homing(self):
        """Poll is_player_homed service until homing is complete."""
        elapsed = time.monotonic() - self.homing_start_time
        if elapsed > self.homing_timeout:
            self.get_logger().error(f"Homing timeout after {self.homing_timeout:.0f}s. Aborting test.")
            self.state = TestState.DONE
            return

        # Check if we have a pending async call
        if self._homing_future is not None:
            if self._homing_future.done():
                try:
                    response = self._homing_future.result()
                    if response.is_homed:
                        self.get_logger().info(
                            f"Player {self.player} is homed "
                            f"(distance: {response.distance:.4f}m). "
                            f"Moving to start position."
                        )
                        self.state = TestState.MOVING_TO_START
                    else:
                        self.get_logger().info(f"Waiting for homing... " f"distance: {response.distance:.4f}m")
                except Exception as e:
                    self.get_logger().warn(f"Homing service call failed: {e}")
                self._homing_future = None
                self.last_homing_poll = time.monotonic()
            return

        # Send a new poll if enough time has passed
        if time.monotonic() - self.last_homing_poll < self.homing_poll_interval:
            return

        if not self.homing_client.service_is_ready():
            self.get_logger().warn(
                "is_player_homed service not available yet...",
                throttle_duration_sec=5.0,
            )
            self.last_homing_poll = time.monotonic()
            return

        request = IsPlayerHomed.Request()
        request.player = self.player
        self._homing_future = self.homing_client.call_async(request)

    def _handle_moving_to_start(self):
        """Drive the peg to the configured start position using board_state feedback."""
        pos = self.get_peg_position()
        if pos is None:
            self.get_logger().warn(
                "Waiting for board_state to get current position...",
                throttle_duration_sec=5.0,
            )
            return

        current_x, current_y = pos
        dx = self.start_x - current_x
        dy = self.start_y - current_y
        distance = math.sqrt(dx * dx + dy * dy)

        if distance <= self.position_tolerance:
            self.publish_velocity(0.0, 0.0)
            self.get_logger().info(
                f"Reached start position ({self.start_x:.3f}, {self.start_y:.3f}). " f"Starting triangle tests."
            )
            self.settle_start_time = None
            self.state = TestState.SETTLING
            return

        # Compute velocity direction toward target
        vx = (dx / distance) * self.move_to_start_vel
        vy = (dy / distance) * self.move_to_start_vel
        self.publish_velocity(vx, vy)

    def _handle_settling(self):
        """Wait with zero velocity for settle_time before the next action."""
        self.publish_velocity(0.0, 0.0)

        now = time.monotonic()
        if self.settle_start_time is None:
            self.settle_start_time = now
            return

        if now - self.settle_start_time >= self.settle_time:
            # Settling complete, determine next action
            if self.current_leg < 3:
                # Start next leg
                self.leg_start_time = None
                self.state = TestState.RUNNING_LEG
            else:
                # Triangle complete
                self.current_leg = 0
                self.current_repetition += 1

                if self.current_repetition >= self.repetitions_per_speed:
                    # Speed level complete, stop bag and increment velocity
                    self._stop_bag()
                    self.current_repetition = 0
                    self.current_velocity += self.velocity_increment

                    if self.current_velocity > self.max_velocity + 1e-6:
                        self.get_logger().info("All speed levels complete.")
                        self.state = TestState.DONE
                        return

                    self.get_logger().info(f"Increasing velocity to {self.current_velocity:.2f} m/s")

                # Start next triangle
                self.settle_start_time = None
                self.leg_start_time = None
                self.state = TestState.RUNNING_LEG

    def _handle_running_leg(self):
        """Execute one leg of the triangle pattern."""
        v = self.current_velocity

        # Compute velocity and duration for current leg
        if self.current_leg == 0:
            # Leg 0: +X
            vx, vy = v, 0.0
            duration = self.side_length / v
        elif self.current_leg == 1:
            # Leg 1: +Y
            vx, vy = 0.0, v
            duration = self.side_length / v
        else:
            # Leg 2: diagonal back to start (-X, -Y)
            vx = -v / math.sqrt(2)
            vy = -v / math.sqrt(2)
            duration = math.sqrt(2) * self.side_length / v

        now = time.monotonic()
        if self.leg_start_time is None:
            if self.current_leg == 0 and self.current_repetition == 0 and not self._bag_running:
                self._start_bag(v)
            self.leg_start_time = now
            self.leg_duration = duration
            leg_names = ["+X", "+Y", "diagonal"]
            self.get_logger().info(
                f"Leg {leg_names[self.current_leg]} | "
                f"v={v:.2f} m/s | "
                f"rep {self.current_repetition + 1}/{self.repetitions_per_speed} | "
                f"duration {duration:.3f}s"
            )

        # Publish velocity command
        self.publish_velocity(vx, vy)

        # Check if leg is complete
        if now - self.leg_start_time >= self.leg_duration:
            self.publish_velocity(0.0, 0.0)
            self.current_leg += 1
            self.leg_start_time = None
            self.settle_start_time = None
            self.state = TestState.SETTLING

    def _handle_done(self):
        """Send zero velocity and shut down."""
        self.publish_velocity(0.0, 0.0)
        self._stop_bag()
        self.control_timer.cancel()
        self.get_logger().info("Acceleration test complete. Shutting down.")
        raise SystemExit


def main(args=None):
    """Entry point for the acceleration test node."""
    rclpy.init(args=args)
    node = AccelerationTestNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node._stop_bag()
        node.publish_velocity(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
