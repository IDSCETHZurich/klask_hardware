#ifndef KLASK_MOTOR_COMMANDER_PKG__ODRIVE_CONTROLLER_HPP_
#define KLASK_MOTOR_COMMANDER_PKG__ODRIVE_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <klask_interfaces/msg/state.hpp>
#include <klask_interfaces/srv/calibrate_encoders.hpp>
#include <klask_interfaces/srv/set_motor_state.hpp>
#include "klask_motor_commander_pkg/player.hpp"
#include "klask_motor_commander_pkg/player_side.hpp"
#include <memory>
#include <map>
#include <string>

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

    /// Subscription for ball and peg state information
    rclcpp::Subscription<klask_interfaces::msg::State>::SharedPtr states_subscriber_;

    /// Current motor state
    int motor_state;

    /**
     * @brief Service callback for encoder calibration.
     *
     * Calibrates both player encoders with camera peg positions.
     *
     * @param request Shared pointer to request containing peg positions.
     * @param response Shared pointer to response with success status.
     */
    void calibrate_encoders_callback(
        const std::shared_ptr<klask_interfaces::srv::CalibrateEncoders::Request> request,
        std::shared_ptr<klask_interfaces::srv::CalibrateEncoders::Response> response);

    /**
     * @brief Service callback for motor state change.
     *
     * Changes the state of all motors (both players).
     *
     * @param request Shared pointer to request containing desired state.
     * @param response Shared pointer to response with success status.
     */
    void set_motor_state_callback(
        const std::shared_ptr<klask_interfaces::srv::SetMotorState::Request> request,
        std::shared_ptr<klask_interfaces::srv::SetMotorState::Response> response);

    /**
     * @brief Generic callback for player velocity commands.
     *
     * Extracts velocity from Twist message and forwards to specified player.
     *
     * @param msg Twist message containing velocity commands.
     * @param player_name Name of the player ("right_player" or "left_player")
     */
    void player_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg, 
                                  const std::string& player_name);
};

#endif // KLASK_MOTOR_COMMANDER_PKG__ODRIVE_CONTROLLER_HPP_
