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

    // Create service servers for calibration and motor state control
    calibrate_encoders_service_ = this->create_service<klask_interfaces::srv::CalibrateEncoders>(
        "calibrate_encoders",
        std::bind(&ODriveController::calibrate_encoders_callback, this,
                  std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Created service: calibrate_encoders");

    set_motor_state_service_ = this->create_service<klask_interfaces::srv::SetMotorState>(
        "set_motor_state",
        std::bind(&ODriveController::set_motor_state_callback, this,
                  std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Created service: set_motor_state");

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

void ODriveController::calibrate_encoders_callback(
    const std::shared_ptr<klask_interfaces::srv::CalibrateEncoders::Request> request,
    std::shared_ptr<klask_interfaces::srv::CalibrateEncoders::Response> response)
{
    try
    {
        // Extract peg positions from State message
        const auto& right_peg_pos = request->state.right_peg.position;
        const auto& left_peg_pos = request->state.left_peg.position;

        right_player_->calibrate(right_peg_pos);
        left_player_->calibrate(left_peg_pos);

        response->success = true;
        response->message = "Encoders calibrated successfully for both players";
        RCLCPP_INFO(this->get_logger(), "Encoders calibrated: right=[%.3f, %.3f], left=[%.3f, %.3f]",
                    right_peg_pos.x, right_peg_pos.y,
                    left_peg_pos.x, left_peg_pos.y);
    }
    catch (const std::exception &e)
    {
        response->success = false;
        response->message = std::string("Calibration failed: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
}

void ODriveController::set_motor_state_callback(
    const std::shared_ptr<klask_interfaces::srv::SetMotorState::Request> request,
    std::shared_ptr<klask_interfaces::srv::SetMotorState::Response> response)
{
    try
    {
        if (motor_state != request->desired_state)
        {
            right_player_->change_motor_state(request->desired_state);
            left_player_->change_motor_state(request->desired_state);
            motor_state = request->desired_state;

            response->success = true;
            response->message = "Motor state changed to " + std::to_string(request->desired_state);
            RCLCPP_INFO(this->get_logger(), "Motor state changed to: %d", request->desired_state);
        }
        else
        {
            response->success = true;
            response->message = "Motor already in state " + std::to_string(request->desired_state);
            RCLCPP_DEBUG(this->get_logger(), "Motor already in state: %d", request->desired_state);
        }
    }
    catch (const std::exception &e)
    {
        response->success = false;
        response->message = std::string("Motor state change failed: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
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
