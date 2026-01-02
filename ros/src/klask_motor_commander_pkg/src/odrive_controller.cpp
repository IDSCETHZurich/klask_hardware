#include "klask_motor_commander_pkg/odrive_controller.hpp"

ODriveController::ODriveController(PlayerSide player_config) : Node("odrive_controller")
{
    RCLCPP_INFO(this->get_logger(), "Initializing ODriveController node with player config: %s", 
                player_side_to_string(player_config).c_str());

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

    // Initialize player nodes based on configuration using bitwise checks
    if (has_player(player_config, PlayerSide::RIGHT_PLAYER))
    {
        RCLCPP_INFO(this->get_logger(), "Creating right_player node...");
        players_["right_player"] = std::make_shared<Player>(PlayerSide::RIGHT_PLAYER);
    }
    
    if (has_player(player_config, PlayerSide::LEFT_PLAYER))
    {
        RCLCPP_INFO(this->get_logger(), "Creating left_player node...");
        players_["left_player"] = std::make_shared<Player>(PlayerSide::LEFT_PLAYER);
    }

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

    // Create subscriptions for velocity commands with reliable QoS (only for active players)
    if (players_.find("right_player") != players_.end())
    {
        velocity_subscribers_["right_player"] = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_right_topic,
            qos_reliable,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                this->player_velocity_callback(msg, "right_player");
            },
            sub_options);
        RCLCPP_INFO(this->get_logger(), "Subscribed to %s for right_player", cmd_vel_right_topic.c_str());
    }

    if (players_.find("left_player") != players_.end())
    {
        velocity_subscribers_["left_player"] = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_left_topic,
            qos_reliable,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                this->player_velocity_callback(msg, "left_player");
            },
            sub_options);
        RCLCPP_INFO(this->get_logger(), "Subscribed to %s for left_player", cmd_vel_left_topic.c_str());
    }

    RCLCPP_INFO(this->get_logger(), "ODriveController initialization complete with %zu active player(s)",
                players_.size());
}

Player::SharedPtr ODriveController::get_player_node(const std::string& player_name) const
{
    auto it = players_.find(player_name);
    if (it != players_.end())
    {
        return it->second;
    }
    return nullptr;
}

const std::map<std::string, Player::SharedPtr>& ODriveController::get_all_players() const
{
    return players_;
}

void ODriveController::calibrate_encoders_callback(
    const std::shared_ptr<klask_interfaces::srv::CalibrateEncoders::Request> request,
    std::shared_ptr<klask_interfaces::srv::CalibrateEncoders::Response> response)
{
    try
    {
        // Extract peg positions from State message and calibrate active players
        std::string calibrated_players;
        
        if (players_.find("right_player") != players_.end())
        {
            const auto& right_peg_pos = request->state.right_peg.position;
            players_["right_player"]->calibrate(right_peg_pos);
            calibrated_players += "right=[" + std::to_string(right_peg_pos.x) + ", " + 
                                 std::to_string(right_peg_pos.y) + "] ";
        }
        
        if (players_.find("left_player") != players_.end())
        {
            const auto& left_peg_pos = request->state.left_peg.position;
            players_["left_player"]->calibrate(left_peg_pos);
            calibrated_players += "left=[" + std::to_string(left_peg_pos.x) + ", " + 
                                 std::to_string(left_peg_pos.y) + "] ";
        }

        response->success = true;
        response->message = "Encoders calibrated successfully for active players";
        RCLCPP_INFO(this->get_logger(), "Encoders calibrated: %s", calibrated_players.c_str());
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
            // Change motor state for all active players
            for (const auto& [player_name, player_node] : players_)
            {
                player_node->change_motor_state(request->desired_state);
            }
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

void ODriveController::player_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg,
                                                const std::string& player_name)
{
    // Extract linear x and y velocities from Twist message
    // Twist.linear.x corresponds to forward/backward velocity
    // Twist.linear.y corresponds to left/right velocity
    auto it = players_.find(player_name);
    if (it != players_.end())
    {
        it->second->send_commands(static_cast<float>(msg->linear.x), static_cast<float>(msg->linear.y));
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Received velocity command for inactive player: %s", 
                    player_name.c_str());
    }
}
