#include "klask_motor_commander_pkg/odrive_controller.hpp"

ODriveController::ODriveController() : Node("odrive_controller")
{
    RCLCPP_INFO(this->get_logger(), "Initializing ODriveController node...");

    // Create reentrant callback group for concurrent processing
    group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // Initialize player nodes
    RCLCPP_INFO(this->get_logger(), "Creating left_player and right_player nodes...");
    right_player_ = std::make_shared<Player>(PlayerSide::RIGHT_PLAYER);
    left_player_ = std::make_shared<Player>(PlayerSide::LEFT_PLAYER);

    // Setup subscription options with callback group
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = group_;

    // Create subscription for ball/peg states with reliable QoS
    auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    motor_communication_sub_ = this->create_subscription<klask_interfaces::msg::MotorCommunication>(
        "/motor_communication", 1, std::bind(&ODriveController::env_callback, this, std::placeholders::_1));

    // Create subscriptions for velocity commands with reliable QoS
    right_player_velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel/right_player_checked",
        qos_reliable,
        std::bind(&ODriveController::right_player_velocity_callback, this, std::placeholders::_1),
        sub_options);
    RCLCPP_INFO(this->get_logger(), "Subscribed to cmd_vel/right_player_checked");

    left_player_velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel/left_player_checked",
        qos_reliable,
        std::bind(&ODriveController::left_player_velocity_callback, this, std::placeholders::_1),
        sub_options);
    RCLCPP_INFO(this->get_logger(), "Subscribed to cmd_vel/left_player_checked");

    RCLCPP_INFO(this->get_logger(), "ODriveController initialization complete");
}

Player::SharedPtr ODriveController::get_right_player_node() const
{
    return this->right_player_;
}

Player::SharedPtr ODriveController::get_left_player_node() const
{
    return this->left_player_;
}

void ODriveController::env_callback(const klask_interfaces::msg::MotorCommunication::SharedPtr msg)
{
    if (msg->calibration_flag.data == true)
    {
        right_player_->calibrate(msg->player_peg_pos);
        left_player_->calibrate(msg->opponent_peg_pos);
    }
    if (motor_state != msg->desired_motor_state.data)
    {
        right_player_->change_motor_state(msg->desired_motor_state.data);
        left_player_->change_motor_state(msg->desired_motor_state.data);
        motor_state = msg->desired_motor_state.data;
    }
}

void ODriveController::right_player_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // Extract linear x and y velocities from Twist message
    // Twist.linear.x corresponds to forward/backward velocity
    // Twist.linear.y corresponds to left/right velocity
    right_player_->send_commands(static_cast<float>(msg->linear.x), static_cast<float>(msg->linear.y));
}

void ODriveController::left_player_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // Extract linear x and y velocities from Twist message
    // Twist.linear.x corresponds to forward/backward velocity
    // Twist.linear.y corresponds to left/right velocity
    left_player_->send_commands(static_cast<float>(msg->linear.x), static_cast<float>(msg->linear.y));
}
