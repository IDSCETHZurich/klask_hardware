#ifndef KLASK_MOTOR_COMMANDER_PKG__ODRIVE_CONTROLLER_HPP_
#define KLASK_MOTOR_COMMANDER_PKG__ODRIVE_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <klask_interfaces/msg/state.hpp>
#include <klask_interfaces/srv/calibrate_encoders.hpp>
#include <klask_interfaces/srv/get_calibration_status.hpp>
#include <klask_interfaces/srv/is_player_homed.hpp>
#include <klask_interfaces/srv/set_motor_state.hpp>
#include <klask_interfaces/action/home_and_calibrate.hpp>
#include <klask_interfaces/action/home_peg.hpp>
#include "klask_motor_commander_pkg/player.hpp"
#include "klask_motor_commander_pkg/player_side.hpp"
#include <memory>
#include <map>
#include <string>
#include <atomic>

namespace klask_motor_commander
{

/**
 * @brief Main controller node for managing ODrive motors and player pegs.
 *
 * This node coordinates Player instances based on configuration, subscribes to
 * ball/peg state information and velocity commands, and dispatches these to the
 * appropriate Player nodes.
 *
 * Convention: Right side is the right_player, left side is the left_player.
 * Top side is right motor for the right side and left motor for the left side.
 */
class ODriveController : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new ODriveController object.
     *
     * Initializes player nodes based on configuration, sets up callback groups,
     * and creates subscriptions for state and velocity data.
     *
     * @param player_config PlayerSide enum indicating which players to create
     */
    ODriveController(PlayerSide player_config = PlayerSide::BOTH_PLAYERS);

    /**
     * @brief Get a player node by name.
     * @param player_name Name of the player ("right_player" or "left_player")
     * @return Player::SharedPtr Shared pointer to the player node, or nullptr if not found
     */
    Player::SharedPtr get_player_node(const std::string& player_name) const;

    /**
     * @brief Get all active player nodes.
     * @return const std::map<std::string, Player::SharedPtr>& Map of player name to player node
     */
    const std::map<std::string, Player::SharedPtr>& get_all_players() const;

    /**
     * @brief Remove internal pointers to player nodes.
     *
     * Sets all player node shared pointers to nullptr.
     */
    void remove_pointers()
    {
        players_.clear();
    }

    /**
     * @brief Enable or disable external cmd_vel commands.
     *
     * When disabled, cmd_vel commands from external sources will be ignored.
     * Useful during startup sequences like homing and calibration.
     *
     * @param enabled True to accept cmd_vel commands, false to ignore them.
     */
    void set_external_commands_enabled(bool enabled)
    {
        external_commands_enabled_.store(enabled);
        if (enabled)
        {
            RCLCPP_INFO(this->get_logger(), "External cmd_vel commands enabled");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "External cmd_vel commands disabled");
        }
    }

    /**
     * @brief Get external commands enabled status.
     *
     * @return true if external commands are enabled, false otherwise.
     */
    bool get_external_commands_enabled() const
    {
        return external_commands_enabled_.load();
    }

    /**
     * @brief Get system calibration status.
     *
     * @return true if system has been calibrated, false otherwise.
     */
    bool is_calibrated() const
    {
        return is_calibrated_.load();
    }

private:
    /// Map of active player nodes (key: "right_player" or "left_player")
    std::map<std::string, Player::SharedPtr> players_;

    /// Reentrant callback group for concurrent callback execution
    rclcpp::CallbackGroup::SharedPtr group_;

    /// Map of velocity subscriptions for active players
    std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> velocity_subscribers_;

    /// Service server for encoder calibration
    rclcpp::Service<klask_interfaces::srv::CalibrateEncoders>::SharedPtr calibrate_encoders_service_;

    /// Service server for motor state changes
    rclcpp::Service<klask_interfaces::srv::SetMotorState>::SharedPtr set_motor_state_service_;

    /// Service server for calibration status query
    rclcpp::Service<klask_interfaces::srv::GetCalibrationStatus>::SharedPtr get_calibration_status_service_;

    /// Action server for right player home and calibrate sequence
    rclcpp_action::Server<klask_interfaces::action::HomeAndCalibrate>::SharedPtr
        right_player_home_and_calibrate_server_;

    /// Action server for left player home and calibrate sequence
    rclcpp_action::Server<klask_interfaces::action::HomeAndCalibrate>::SharedPtr left_player_home_and_calibrate_server_;

    /// Service server for checking if player is homed
    rclcpp::Service<klask_interfaces::srv::IsPlayerHomed>::SharedPtr is_player_homed_service_;

    /// Subscription for ball and peg state information
    rclcpp::Subscription<klask_interfaces::msg::State>::SharedPtr states_subscriber_;

    /// Current motor state
    int motor_state;

    /// Flag indicating if external cmd_vel commands are enabled
    std::atomic<bool> external_commands_enabled_;

    /// Flag indicating if system has been calibrated
    std::atomic<bool> is_calibrated_;

    /// Action client for right player homing
    rclcpp_action::Client<klask_interfaces::action::HomePeg>::SharedPtr right_player_homing_client_;

    /// Action client for left player homing
    rclcpp_action::Client<klask_interfaces::action::HomePeg>::SharedPtr left_player_homing_client_;

    /// Homing action timeout in seconds
    int homing_action_timeout_;

    /// Action server wait timeout in seconds
    int action_server_wait_timeout_;

    /// Delay between homing different players (ms)
    int inter_homing_delay_;

    /**
     * @brief Service callback for encoder calibration.
     *
     * Calibrates both player encoders with camera peg positions.
     *
     * @param request Shared pointer to request containing peg positions.
     * @param response Shared pointer to response with success status.
     */
    void calibrate_encoders_callback(const std::shared_ptr<klask_interfaces::srv::CalibrateEncoders::Request> request,
                                     std::shared_ptr<klask_interfaces::srv::CalibrateEncoders::Response> response);

    /**
     * @brief Service callback for motor state change.
     *
     * Changes the state of all motors (both players).
     *
     * @param request Shared pointer to request containing desired state.
     * @param response Shared pointer to response with success status.
     */
    void set_motor_state_callback(const std::shared_ptr<klask_interfaces::srv::SetMotorState::Request> request,
                                  std::shared_ptr<klask_interfaces::srv::SetMotorState::Response> response);

    /**
     * @brief Service callback for calibration status query.
     *
     * Returns whether the system has been calibrated.
     *
     * @param request Shared pointer to request (empty).
     * @param response Shared pointer to response with calibration status.
     */
    void get_calibration_status_callback(
        const std::shared_ptr<klask_interfaces::srv::GetCalibrationStatus::Request> request,
        std::shared_ptr<klask_interfaces::srv::GetCalibrationStatus::Response> response);

    /**
     * @brief Action goal callback for home and calibrate sequence.
     */
    rclcpp_action::GoalResponse handle_home_and_calibrate_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const klask_interfaces::action::HomeAndCalibrate::Goal> goal,
        const std::string& player_name);

    /**
     * @brief Action cancel callback for home and calibrate sequence.
     */
    rclcpp_action::CancelResponse handle_home_and_calibrate_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<klask_interfaces::action::HomeAndCalibrate>> goal_handle);

    /**
     * @brief Action accepted callback - executes homing for specified player followed by calibration.
     */
    void handle_home_and_calibrate_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<klask_interfaces::action::HomeAndCalibrate>> goal_handle,
        const std::string& player_name);

    /**
     * @brief Service callback to check if a player is at home position.
     *
     * Checks if the specified player is within position tolerance of their home position.
     *
     * @param request Shared pointer to request containing player name.
     * @param response Shared pointer to response with homing status and distance.
     */
    void is_player_homed_callback(const std::shared_ptr<klask_interfaces::srv::IsPlayerHomed::Request> request,
                                  std::shared_ptr<klask_interfaces::srv::IsPlayerHomed::Response> response);

    /**
     * @brief Generic callback for player velocity commands.
     *
     * Extracts velocity from Twist message and forwards to specified player.
     *
     * @param msg Twist message containing velocity commands.
     * @param player_name Name of the player ("right_player" or "left_player")
     */
    void player_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg, const std::string& player_name);
};

} // namespace klask_motor_commander

#endif // KLASK_MOTOR_COMMANDER_PKG__ODRIVE_CONTROLLER_HPP_
