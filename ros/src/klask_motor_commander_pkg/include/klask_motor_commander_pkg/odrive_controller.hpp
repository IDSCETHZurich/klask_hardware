#ifndef KLASK_MOTOR_COMMANDER_PKG__ODRIVE_CONTROLLER_HPP_
#define KLASK_MOTOR_COMMANDER_PKG__ODRIVE_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <klask_interfaces/msg/state.hpp>
#include "klask_motor_commander_pkg/player.hpp"
#include <memory>

/**
 * @brief Main controller node for managing ODrive motors and player pegs.
 * 
 * This node coordinates two Player instances (player and opponent), subscribes to
 * ball/peg state information and velocity commands, and dispatches these to the
 * appropriate Player nodes.
 * 
 * Convention: Right side is the player, left side is the opponent.
 * Top side is right motor for the right side and left motor for the left side.
 */
class ODriveController : public rclcpp::Node {
public:
    /**
     * @brief Construct a new ODriveController object.
     * 
     * Initializes both player and opponent nodes, sets up callback groups,
     * and creates subscriptions for state and velocity data.
     */
    ODriveController();
    
    /**
     * @brief Get the player node shared pointer.
     * @return Player::SharedPtr Shared pointer to the player node.
     */
    Player::SharedPtr get_player_node() const;
    
    /**
     * @brief Get the opponent node shared pointer.
     * @return Player::SharedPtr Shared pointer to the opponent node.
     */
    Player::SharedPtr get_opponent_node() const;

private:
    /// Player node instance (right side)
    std::shared_ptr<Player> player_;
    
    /// Opponent node instance (left side)
    std::shared_ptr<Player> opponent_;

    /// Reentrant callback group for concurrent callback execution
    rclcpp::CallbackGroup::SharedPtr group_;

    /// Subscription for velocity command requests
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr velocity_subscriber_;
    
    /// Subscription for ball and peg state information
    rclcpp::Subscription<klask_interfaces::msg::State>::SharedPtr states_subscriber_;

    /**
     * @brief Callback for processing ball and peg state updates.
     * 
     * Updates position and velocity for both player and opponent pegs,
     * checks synchronization state, and updates synchronized state if needed.
     * 
     * @param msg Shared pointer to State message containing state data.
     */
    void state_callback(const klask_interfaces::msg::State::SharedPtr msg);
    
    /**
     * @brief Callback for processing velocity command requests.
     * 
     * Dispatches velocity targets to player and opponent nodes.
     * Expected format: [opponent_vx, opponent_vy, player_vx, player_vy]
     * 
     * @param msg Shared pointer to Float32MultiArray containing velocity commands.
     */
    void velocity_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
};

#endif  // KLASK_MOTOR_COMMANDER_PKG__ODRIVE_CONTROLLER_HPP_
