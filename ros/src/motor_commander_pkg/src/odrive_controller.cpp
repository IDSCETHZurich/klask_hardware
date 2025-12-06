#include "motor_commander_pkg/odrive_controller.hpp"

ODriveController::ODriveController() : Node("odrive_controller") {
    RCLCPP_INFO(this->get_logger(), "Initializing ODriveController node...");
    
    // Create reentrant callback group for concurrent processing
    group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // Initialize player nodes
    RCLCPP_INFO(this->get_logger(), "Creating player and opponent nodes...");
    player_ = std::make_shared<Player>("player");
    opponent_ = std::make_shared<Player>("opponent");

    // Setup subscription options with callback group
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = group_;

    // Create subscription for ball/peg states with reliable QoS
    auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    states_subscriber_ = this->create_subscription<klask_interfaces::msg::StampedPolygon>(
        "/ball_peg_states", 
        qos_reliable,
        std::bind(&ODriveController::state_callback, this, std::placeholders::_1), 
        sub_options);
    RCLCPP_INFO(this->get_logger(), "Subscribed to /ball_peg_states");

    // Create subscription for velocity commands with reliable QoS
    velocity_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "velocity_requests_checked", 
        qos_reliable,
        std::bind(&ODriveController::velocity_callback, this, std::placeholders::_1), 
        sub_options);
    RCLCPP_INFO(this->get_logger(), "Subscribed to velocity_requests_checked");
    
    RCLCPP_INFO(this->get_logger(), "ODriveController initialization complete");
}

Player::SharedPtr ODriveController::get_player_node() const {
    return this->player_;
}

Player::SharedPtr ODriveController::get_opponent_node() const {
    return this->opponent_;
}

void ODriveController::state_callback(const klask_interfaces::msg::StampedPolygon::SharedPtr msg) {
    // Validate message has sufficient points
    if (msg->polygon.points.size() < 6) {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,  // Log at most once every 5 seconds
            "Invalid state message: expected at least 6 points, got %zu",
            msg->polygon.points.size());
        return;
    }
    
    // Update opponent position and velocity (points 2 and 3)
    opponent_->position = {msg->polygon.points[2].x, msg->polygon.points[2].y};
    opponent_->velocity = {msg->polygon.points[3].x, msg->polygon.points[3].y};
    
    // Update player position and velocity (points 4 and 5)
    player_->position = {msg->polygon.points[4].x, msg->polygon.points[4].y};
    player_->velocity = {msg->polygon.points[5].x, msg->polygon.points[5].y};
    
    // Check synchronization for player if already synchronized once
    if (player_->synchronized_once) {
        player_->is_synchronized();
    }
    
    // Check synchronization for opponent if already synchronized once
    if (opponent_->synchronized_once) {
        opponent_->is_synchronized();
    }
    
    // Update player synchronized state if peg is synchronized
    if (player_->peg_mag_synchronized && !player_->synchronized_state.empty()) {
        player_->synchronized_state[0] = player_->encoder_values[0];
        player_->synchronized_state[1] = player_->encoder_values[1];
        player_->synchronized_state[2] = player_->position[0];
        player_->synchronized_state[3] = player_->position[1];
    }
    
    // Update opponent synchronized state if peg is synchronized
    if (opponent_->peg_mag_synchronized && !opponent_->synchronized_state.empty()) {
        opponent_->synchronized_state[0] = opponent_->encoder_values[0];
        opponent_->synchronized_state[1] = opponent_->encoder_values[1];
        opponent_->synchronized_state[2] = opponent_->position[0];
        opponent_->synchronized_state[3] = opponent_->position[1];
    }
}

void ODriveController::velocity_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    // Validate message has expected number of velocity commands
    // Expected format: [opponent_vx, opponent_vy, player_vx, player_vy]
    if (msg->data.size() < 4) {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,  // Log at most once every 5 seconds
            "Invalid velocity message: expected 4 values, got %zu",
            msg->data.size());
        return;
    }
    
    // Dispatch velocity commands to respective nodes
    // Indices 0, 1: opponent (vx, vy)
    // Indices 2, 3: player (vx, vy)
    opponent_->velocity_targets(msg->data[0], msg->data[1]);
    player_->velocity_targets(msg->data[2], msg->data[3]);
}
