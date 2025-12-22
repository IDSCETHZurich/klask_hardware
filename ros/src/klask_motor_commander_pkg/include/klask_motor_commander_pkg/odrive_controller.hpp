#ifndef KLASK_MOTOR_COMMANDER_PKG__ODRIVE_CONTROLLER_HPP_
#define KLASK_MOTOR_COMMANDER_PKG__ODRIVE_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <klask_interfaces/msg/state.hpp>
#include <klask_interfaces/msg/motor_communication.hpp>
#include "klask_motor_commander_pkg/player.hpp"
#include <memory>

/**
 * @brief Main controller node for managing ODrive motors and player pegs.
 *
 * This node coordinates two Player instances (player and opponent), subscribes to
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
     * Initializes both player and opponent nodes, sets up callback groups,
     * and creates subscriptions for state and velocity data.
     */
    ODriveController();

    /**
     * @brief Get the right player node shared pointer.
     * @return Player::SharedPtr Shared pointer to the right player node.
     */
    Player::SharedPtr get_right_player_node() const;

    /**
     * @brief Get the left player node shared pointer.
     * @return Player::SharedPtr Shared pointer to the left player node.
     */
    Player::SharedPtr get_left_player_node() const;

    /**
     * @brief Remove internal pointers to player nodes.
     *
     * Sets the shared pointers for both player nodes to nullptr.
     */
    void remove_pointers()
    {
        right_player_ = nullptr;
        left_player_ = nullptr;
    }

private:
    /// Right player node instance (right side)
    std::shared_ptr<Player> right_player_;

    /// Left player node instance (left side)
    std::shared_ptr<Player> left_player_;

    /// Reentrant callback group for concurrent callback execution
    rclcpp::CallbackGroup::SharedPtr group_;

    /// Subscription for right player velocity commands
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr right_player_velocity_subscriber_;

    /// Subscription for left player velocity commands
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr left_player_velocity_subscriber_;

    /// Subscription for motor communication messages
    rclcpp::Subscription<klask_interfaces::msg::MotorCommunication>::SharedPtr motor_communication_sub_;

    /// Subscription for ball and peg state information
    rclcpp::Subscription<klask_interfaces::msg::State>::SharedPtr states_subscriber_;

    // TODO: Remove this once estimator is correct
    const float REAL_TO_CAM_FACTOR_X = 1.0f / 0.0008285f;
    const float REAL_TO_CAM_FACTOR_Y = 1150.0f;

    int motor_state = 8;

    void env_callback(const klask_interfaces::msg::MotorCommunication::SharedPtr msg);

    /**
     * @brief Callback for processing right player velocity commands.
     *
     * Dispatches velocity targets to the right player node.
     *
     * @param msg Shared pointer to Twist message containing velocity commands.
     */
    void right_player_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    /**
     * @brief Callback for processing left player velocity commands.
     *
     * Dispatches velocity targets to the left player node.
     *
     * @param msg Shared pointer to Twist message containing velocity commands.
     */
    void left_player_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
};

#endif // KLASK_MOTOR_COMMANDER_PKG__ODRIVE_CONTROLLER_HPP_
