#include "klask_motor_commander_pkg/odrive_controller.hpp"

ODriveController::ODriveController() : Node("odrive_controller")
{
    RCLCPP_INFO(this->get_logger(), "Initializing ODriveController node...");

    // === Declare and load ROS parameters ===
    
    // Topic names
    this->declare_parameter("cmd_vel_right_player", "cmd_vel/right_player_checked");
    this->declare_parameter("cmd_vel_left_player", "cmd_vel/left_player_checked");
    
    // Service names
    this->declare_parameter("calibrate_encoders_service", "calibrate_encoders");
    this->declare_parameter("set_motor_state_service", "set_motor_state");
    
    // QoS settings
    this->declare_parameter("cmd_vel_qos_depth", 1);
    
    // Initial motor state
    this->declare_parameter("initial_motor_state", 8);
    
    // Load parameters
    std::string cmd_vel_right_topic = this->get_parameter("cmd_vel_right_player").as_string();
    std::string cmd_vel_left_topic = this->get_parameter("cmd_vel_left_player").as_string();
    std::string calibrate_service = this->get_parameter("calibrate_encoders_service").as_string();
    std::string motor_state_service = this->get_parameter("set_motor_state_service").as_string();
    int qos_depth = this->get_parameter("cmd_vel_qos_depth").as_int();
    motor_state = this->get_parameter("initial_motor_state").as_int();

    RCLCPP_INFO(this->get_logger(), "Loaded parameters: initial_motor_state=%d", motor_state);

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
    auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable();

    // Create service servers for calibration and motor state control
    calibrate_encoders_service_ = this->create_service<klask_interfaces::srv::CalibrateEncoders>(
        calibrate_service,
        std::bind(&ODriveController::calibrate_encoders_callback, this,
                  std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Created service: %s", calibrate_service.c_str());

    set_motor_state_service_ = this->create_service<klask_interfaces::srv::SetMotorState>(
        motor_state_service,
        std::bind(&ODriveController::set_motor_state_callback, this,
                  std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Created service: %s", motor_state_service.c_str());

    // Create subscriptions for velocity commands with reliable QoS
    right_player_velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_right_topic,
        qos_reliable,
        std::bind(&ODriveController::right_player_velocity_callback, this, std::placeholders::_1),
        sub_options);
    RCLCPP_INFO(this->get_logger(), "Subscribed to %s", cmd_vel_right_topic.c_str());

    left_player_velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_left_topic,
        qos_reliable,
        std::bind(&ODriveController::left_player_velocity_callback, this, std::placeholders::_1),
        sub_options);
    RCLCPP_INFO(this->get_logger(), "Subscribed to %s", cmd_vel_left_topic.c_str());

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
