#include "klask_motor_commander_pkg/odrive_controller.hpp"

ODriveController::ODriveController() : Node("odrive_controller") {
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
    states_subscriber_ = this->create_subscription<klask_interfaces::msg::State>(
        "/board_state", 
        qos_reliable,
        std::bind(&ODriveController::state_callback, this, std::placeholders::_1), 
        sub_options);
    RCLCPP_INFO(this->get_logger(), "Subscribed to /board_state");

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

Player::SharedPtr ODriveController::get_right_player_node() const {
    return this->right_player_;
}

Player::SharedPtr ODriveController::get_left_player_node() const {
    return this->left_player_;
}

void ODriveController::state_callback(const klask_interfaces::msg::State::SharedPtr msg) {
    
    // Update left player position and velocity
    left_player_->position = {static_cast<float>(msg->left_peg.position.x), static_cast<float>(msg->left_peg.position.y)};
    left_player_->velocity = {static_cast<float>(msg->left_peg.velocity.x), static_cast<float>(msg->left_peg.velocity.y)};
    
    // Update right player position and velocity
    right_player_->position = {static_cast<float>(msg->right_peg.position.x), static_cast<float>(msg->right_peg.position.y)};
    right_player_->velocity = {static_cast<float>(msg->right_peg.velocity.x), static_cast<float>(msg->right_peg.velocity.y)};
    
    // Check synchronization for right player if already synchronized once
    if (right_player_->synchronized_once) {
        right_player_->is_synchronized();
    }
    
    // Check synchronization for left player if already synchronized once
    if (left_player_->synchronized_once) {
        left_player_->is_synchronized();
    }
    
    // Update right player synchronized state if peg is synchronized
    if (right_player_->peg_mag_synchronized && !right_player_->synchronized_state.empty()) {
        right_player_->synchronized_state[0] = right_player_->encoder_values[0];
        right_player_->synchronized_state[1] = right_player_->encoder_values[1];
        right_player_->synchronized_state[2] = right_player_->position[0];
        right_player_->synchronized_state[3] = right_player_->position[1];
    }
    
    // Update left player synchronized state if peg is synchronized
    if (left_player_->peg_mag_synchronized && !left_player_->synchronized_state.empty()) {
        left_player_->synchronized_state[0] = left_player_->encoder_values[0];
        left_player_->synchronized_state[1] = left_player_->encoder_values[1];
        left_player_->synchronized_state[2] = left_player_->position[0];
        left_player_->synchronized_state[3] = left_player_->position[1];
    }
}

void ODriveController::right_player_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Extract linear x and y velocities from Twist message
    // Twist.linear.x corresponds to forward/backward velocity
    // Twist.linear.y corresponds to left/right velocity
    right_player_->velocity_targets(static_cast<float>(msg->linear.x), static_cast<float>(msg->linear.y));
}

void ODriveController::left_player_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Extract linear x and y velocities from Twist message
    // Twist.linear.x corresponds to forward/backward velocity
    // Twist.linear.y corresponds to left/right velocity
    left_player_->velocity_targets(static_cast<float>(msg->linear.x), static_cast<float>(msg->linear.y));
}
