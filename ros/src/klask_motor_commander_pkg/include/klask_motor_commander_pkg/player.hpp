#ifndef KLASK_MOTOR_COMMANDER_PKG__PLAYER_HPP_
#define KLASK_MOTOR_COMMANDER_PKG__PLAYER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <odrive_can/srv/axis_state.hpp>
#include <odrive_can/msg/control_message.hpp>
#include <odrive_can/msg/controller_status.hpp>
#include "klask_motor_commander_pkg/player_side.hpp"
#include <atomic>
#include <cmath>
#include <algorithm>
#include <functional>
#include <vector>

namespace klask_motor_commander
{

/**
 * @brief ODrive motor control mode constants.
 */
enum class ODriveControlMode : int
{
    VELOCITY_CONTROL = 2 ///< Velocity control mode
};

/**
 * @brief ODrive input mode constants.
 */
enum class ODriveInputMode : int
{
    VEL_RAMP = 1 ///< Velocity ramp input mode
};

/**
 * @brief ODrive axis state constants.
 */
enum class ODriveAxisState : int
{
    IDLE = 1,               ///< Motor idle state
    CLOSED_LOOP_CONTROL = 8 ///< Closed loop control state
};

/**
 * @brief Controls a single player's peg movement using two ODrive motors.
 *
 * This class manages the motor control for one player (either right_player or left_player),
 * handling peg synchronization, wall collision detection, velocity profiling, and
 * coordinated movement between two motors in a differential drive configuration.
 *
 * The system uses encoder feedback and torque sensing to:
 * - Detect when the magnetic peg and physical peg are synchronized
 * - Prevent collisions with table edges
 * - Navigate the peg to home position after synchronization
 * - Execute smooth velocity commands with deceleration profiles
 */
class Player : public rclcpp::Node
{
public:
    /// Type alias for shared pointer to Player
    using SharedPtr = std::shared_ptr<Player>;

    /**
     * @brief Construct a new Player object.
     *
     * @param side Player side identifier (RIGHT_PLAYER or LEFT_PLAYER).
     *             Determines motor indices, home position, and field boundaries.
     */
    Player(PlayerSide side);

    /**
     * @brief Get the player side.
     *
     * @return PlayerSide The side this player controls.
     */
    PlayerSide get_side() const
    {
        return side_;
    }

    /**
     * @brief Send velocity commands to motors.
     *
     * @param v_x Target velocity in x-direction (m/s).
     * @param v_y Target velocity in y-direction (m/s).
     */
    void send_commands(float v_x, float v_y);

    /**
     * @brief Calibrate motor position with peg position from camera.
     *
     * @param peg_pos Peg position from camera/vision system.
     */
    void calibrate(const geometry_msgs::msg::Point& peg_pos);

    /**
     * @brief Change motor state (idle, closed loop control, etc.).
     *
     * @param des_state Desired motor state.
     */
    void change_motor_state(const int& des_state);

    /**
     * @brief Enable or disable the boundary deceleration profile.
     *
     * When disabled, no velocity clamping near field edges is applied.
     * Enabled by default.
     *
     * @param enabled True to apply deceleration near boundaries, false to skip it.
     */
    void set_deceleration_enabled(bool enabled)
    {
        deceleration_enabled_.store(enabled);
    }

    /**
     * @brief Check whether boundary deceleration is currently enabled.
     * @return true if enabled, false otherwise.
     */
    bool get_deceleration_enabled() const
    {
        return deceleration_enabled_.load();
    }

    /**
     * @brief Set a callback to be invoked on a fatal motor error.
     *
     * The callback is responsible for idling all motors and triggering shutdown.
     *
     * @param cb Callable invoked when a fatal motor error is detected.
     */
    void set_fatal_error_callback(std::function<void()> cb)
    {
        fatal_error_callback_ = std::move(cb);
    }

    // Public state variables

    /// Synchronized reference state [encoder_r, encoder_l, cam_x, cam_y]
    std::vector<float> synchronized_state;

    /// Estimated magnet position [x, y]
    std::vector<float> magnet_position;

private:
    // === Node Configuration ===

    /// Player side (RIGHT_PLAYER or LEFT_PLAYER)
    PlayerSide side_;

    /// Direction multiplier (+1 for player, -1 for opponent)
    float sign;

    // === ROS2 Communication ===

    /// Reentrant callback group for concurrent processing
    rclcpp::CallbackGroup::SharedPtr group_;

    /// Publisher for left motor control commands
    rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr motor_left_pub_;

    /// Publisher for right motor control commands
    rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr motor_right_pub_;

    /// Service client for right motor axis state control
    rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr request_axis_r_state_client_;

    /// Service client for left motor axis state control
    rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr request_axis_l_state_client_;

    /// Subscriber for right motor controller status
    rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr controller_r_subscriber;

    /// Subscriber for left motor controller status
    rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr controller_l_subscriber;

    /// Publisher for estimated magnet position and velocity
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr position_magnet_publisher_;

    // === Motor State ===

    /// Current encoder position - right motor
    float encoder_right;

    /// Current encoder position - left motor
    float encoder_left;

    /// Field boundary edges [left, right, bottom, top]
    std::vector<float> EDGE;

    /// Callback counter for frequency monitoring
    int callback_count;

    /// Last time for frequency calculation
    rclcpp::Time last_time;

    // === Parameters (loaded from ROS parameters) ===

    /// Distance traveled per motor revolution (m)
    float distance_per_revolution_;

    /// Maximum allowed velocity (m/s)
    float max_velocity_;

    /// Deceleration distance from edge (m)
    float deceleration_distance_;

    /// Peg radius (m)
    float peg_radius_;

    /// Minimum clearance factor (multiplied by peg_radius)
    float min_clearance_factor_;

    /// Corner margins for unreachable areas (m)
    float corner_x_margin_;
    float corner_y_margin_;

    /// Monitoring parameters
    int frequency_log_interval_;
    int error_throttle_duration_;

    /// Service timeout parameters
    int service_wait_timeout_;
    int max_service_wait_attempts_;

    /// Whether boundary deceleration profile is active
    std::atomic<bool> deceleration_enabled_;

    /// Callback invoked on fatal motor error (set from main to idle all motors and shutdown)
    std::function<void()> fatal_error_callback_;

    // === Private Methods ===

    /**
     * @brief Apply deceleration profile near field boundaries.
     *
     * @param v_x Reference to x-velocity, modified in place.
     * @param v_y Reference to y-velocity, modified in place.
     */
    void deacceleration_profile(float& v_x, float& v_y);

    /**
     * @brief Update magnet position estimate from encoder values.
     */
    void update_magnet_pos();

    /**
     * @brief Callback for right motor controller status updates.
     *
     * @param msg Controller status message.
     */
    void controller_status_callback_right(const odrive_can::msg::ControllerStatus::SharedPtr msg);

    /**
     * @brief Callback for left motor controller status updates.
     *
     * @param msg Controller status message.
     */
    void controller_status_callback_left(const odrive_can::msg::ControllerStatus::SharedPtr msg);

    /**
     * @brief Request motor axis state change.
     *
     * @param state Desired axis state (1=idle, 8=closed_loop_control).
     */
    void set_motor_state(int state);
};

} // namespace klask_motor_commander

#endif // KLASK_MOTOR_COMMANDER_PKG__PLAYER_HPP_
