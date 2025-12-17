#ifndef KLASK_MOTOR_COMMANDER_PKG__PLAYER_HPP_
#define KLASK_MOTOR_COMMANDER_PKG__PLAYER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <klask_interfaces/msg/stamped_polygon.hpp>
#include <odrive_can/srv/axis_state.hpp>
#include <odrive_can/msg/control_message.hpp>
#include <odrive_can/msg/controller_status.hpp>
#include <random>
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>
#include <sstream>

/**
 * @brief Enum to identify which side of the table the player controls.
 */
enum class PlayerSide {
    RIGHT_PLAYER,  ///< Right side player (motors 0 and 1)
    LEFT_PLAYER    ///< Left side player (motors 2 and 3)
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
class Player : public rclcpp::Node {
public:
    /**
     * @brief Construct a new Player object.
     * 
     * @param side Player side identifier (RIGHT_PLAYER or LEFT_PLAYER).
     *             Determines motor indices, home position, and field boundaries.
     */
    Player(PlayerSide side);

    /**
     * @brief Process velocity command targets.
     * 
     * If synchronized, publishes the velocity. Otherwise, initiates peg finding routine.
     * 
     * @param v_x Target velocity in x-direction (m/s).
     * @param v_y Target velocity in y-direction (m/s).
     */
    void velocity_targets(float v_x, float v_y);
    
    /**
     * @brief Check and update synchronization state between peg and magnet.
     * 
     * Uses velocity buffer analysis to determine if the physical peg is following
     * the magnetic peg correctly. Updates peg_mag_synchronized flag.
     */
    void is_synchronized();

    // Public state variables - accessible by ODriveController
    
    /// Flag indicating if currently searching for peg
    bool finding_peg;
    
    /// Flag indicating if peg and magnet are moving together
    bool peg_mag_synchronized;
    
    /// Flag indicating if synchronization has been achieved at least once
    bool synchronized_once;
    
    /// Current peg position [x, y] in camera coordinates
    std::vector<float> position;
    
    /// Current peg velocity [vx, vy] in camera coordinates
    std::vector<float> velocity;
    
    /// Synchronized reference state [encoder_r, encoder_l, cam_x, cam_y]
    std::vector<float> synchronized_state;
    
    /// Motor encoder values [pos_r, pos_l, vel_r, vel_l]
    std::vector<float> encoder_values;

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
    
    // === Synchronization State ===
    
    /// Buffer storing recent synchronization status (true = synchronized)
    std::vector<bool> synch_buffer;
    
    /// Adaptive parameter for determining buffer elements to check
    float sync_elements_to_check;
    
    // === Collision Detection ===
    
    /// Flag indicating if motors are experiencing high torque (at wall)
    bool motor_at_wall;
    
    /// Flag for calibration-level torque detection
    bool motor_at_wall_calib;
    
    /// Buffer for wall collision detection on each edge [4 edges x 7 samples]
    std::vector<std::vector<bool>> motor_at_wall_buffer;
    
    /// Flags indicating wall protection is active for each edge
    std::vector<bool> wall_prot_activated;
    
    /// Current torque estimate from left motor
    float torque_left_motor;
    
    /// Current torque estimate from right motor
    float torque_right_motor;
    
    // === Position References ===
    
    /// Home position coordinates [x, y]
    std::vector<float> home;
    
    /// Field boundary edges [left, right, bottom, top]
    std::vector<float> EDGE;
    
    /// Flag indicating if peg is at home position
    bool at_home;
    
    /// Estimated magnet position [x, y] from encoder odometry
    std::vector<float> magnet_position;
    
    /// Estimated magnet velocity [vx, vy] from encoder data
    std::vector<float> magnet_velocity;
    
    // === Physical Constants ===
    
    /// Distance traveled per motor revolution (m)
    static constexpr float DISTANCE_PER_REVOLUTION = 0.04f;
    
    /// Conversion factor from meters to camera x-coordinates
    static constexpr float REAL_TO_CAM_FACTOR_X = 1.0f / 0.0008285f;
    
    /// Conversion factor from meters to camera y-coordinates
    static constexpr float REAL_TO_CAM_FACTOR_Y = 1150.0f;
    
    /// Maximum allowed torque before wall detection (N·m)
    static constexpr float MAX_TORQUE = 0.25f;
    
    /// Torque threshold for calibration wall detection (N·m)
    static constexpr float CALIB_TORQUE = 0.225f;
    
    /// Critical torque threshold triggering emergency shutdown (N·m)
    static constexpr float SHUTDOWN_TORQUE = 1.5f;
    
    // === Private Methods ===
    
    /**
     * @brief Execute peg finding routine.
     * 
     * Moves peg to corner, performs search pattern, and navigates to home.
     */
    void find_peg();
    
    /**
     * @brief Navigate peg to table corner for initial synchronization.
     */
    void move_to_corner();
    
    /**
     * @brief Execute search pattern to locate and synchronize with peg.
     */
    void move_pattern();
    
    /**
     * @brief Navigate peg to home position after synchronization.
     */
    void find_home();
    
    /**
     * @brief Calculate number of buffer elements to check based on velocity.
     * 
     * @param x Velocity magnitude.
     * @return unsigned int Number of elements to check in synchronization buffer.
     */
    unsigned int interpolate_elements(float x) const;
    
    /**
     * @brief Publish velocity with safety checks and wall avoidance.
     * 
     * @param v_x Desired x-velocity (m/s).
     * @param v_y Desired y-velocity (m/s).
     */
    void publish_velocity(float v_x, float v_y);
    
    /**
     * @brief Apply deceleration profile near field boundaries.
     * 
     * @param v_x Reference to x-velocity, modified in place.
     * @param v_y Reference to y-velocity, modified in place.
     */
    void deaceleration_profile(float& v_x, float& v_y);
    
    /**
     * @brief Modify velocity to prevent movement into walls.
     * 
     * @param v_x Reference to x-velocity, modified in place.
     * @param v_y Reference to y-velocity, modified in place.
     */
    void move_away_from_wall(float& v_x, float& v_y);
    
    /**
     * @brief Send motor control commands.
     * 
     * @param v_x X-velocity command (m/s).
     * @param v_y Y-velocity command (m/s).
     */
    void send_commands(float v_x, float v_y);
    
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
     * @brief Update estimated magnet position from encoder values.
     * 
     * @param encoder_values Current encoder position and velocity [pos_r, pos_l, vel_r, vel_l].
     */
    void update_magnet_position(const std::vector<float>& encoder_values);
    
    /**
     * @brief Request motor axis state change.
     * 
     * @param state Desired axis state (1=idle, 8=closed_loop_control).
     */
    void set_motor_state(int state);
    
    /**
     * @brief Calculate combined torque magnitude from both motors.
     * 
     * @return float Total torque magnitude (N·m).
     */
    float calculate_total_torque() const;
    
    /**
     * @brief Process torque-based collision detection and state changes.
     * 
     * @param total_torque Current total torque magnitude.
     */
    void handle_torque_limits(float total_torque);
};

#endif  // KLASK_MOTOR_COMMANDER_PKG__PLAYER_HPP_
