#include "klask_motor_commander_pkg/odrive_controller.hpp"
#include "klask_motor_commander_pkg/open_loop_controller.hpp"

namespace klask_motor_commander
{

ODriveController::ODriveController(PlayerSide player_config)
    : Node("odrive_controller")
    , external_commands_enabled_(true)
    , is_calibrated_(false)
{
    RCLCPP_INFO(this->get_logger(),
                "Initializing ODriveController node with player config: %s",
                player_side_to_string(player_config).c_str());

    // === Declare and load ROS parameters ===

    // Topic names
    this->declare_parameter("cmd_vel_right_player", "cmd_vel/right_player");
    this->declare_parameter("cmd_vel_left_player", "cmd_vel/left_player");

    // Service names
    this->declare_parameter("calibrate_encoders_service", "calibrate_encoders");
    this->declare_parameter("get_calibration_status_service", "get_calibration_status");
    this->declare_parameter("home_and_calibrate_service", "home_and_calibrate");
    this->declare_parameter("set_motor_state_service", "set_motor_state");

    // Action names for homing
    this->declare_parameter("right_player_action_name", "home_peg_right_player");
    this->declare_parameter("left_player_action_name", "home_peg_left_player");

    // Homing timeouts
    this->declare_parameter("action_server_wait_timeout", 5);
    this->declare_parameter("homing_action_timeout", 30);
    this->declare_parameter("inter_homing_delay", 500);

    // QoS settings
    this->declare_parameter("cmd_vel_qos_depth", 1);

    // Initial motor state
    this->declare_parameter("initial_motor_state", 8);

    // Load parameters
    std::string cmd_vel_right_topic = this->get_parameter("cmd_vel_right_player").as_string();
    std::string cmd_vel_left_topic = this->get_parameter("cmd_vel_left_player").as_string();
    std::string calibrate_service = this->get_parameter("calibrate_encoders_service").as_string();
    std::string calibration_status_service = this->get_parameter("get_calibration_status_service").as_string();
    std::string home_calibrate_service = this->get_parameter("home_and_calibrate_service").as_string();
    std::string motor_state_service = this->get_parameter("set_motor_state_service").as_string();
    std::string right_action_name = this->get_parameter("right_player_action_name").as_string();
    std::string left_action_name = this->get_parameter("left_player_action_name").as_string();
    int qos_depth = this->get_parameter("cmd_vel_qos_depth").as_int();
    motor_state = this->get_parameter("initial_motor_state").as_int();
    action_server_wait_timeout_ = this->get_parameter("action_server_wait_timeout").as_int();
    homing_action_timeout_ = this->get_parameter("homing_action_timeout").as_int();
    inter_homing_delay_ = this->get_parameter("inter_homing_delay").as_int();

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
        std::bind(&ODriveController::calibrate_encoders_callback, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Created service: %s", calibrate_service.c_str());

    set_motor_state_service_ = this->create_service<klask_interfaces::srv::SetMotorState>(
        motor_state_service,
        std::bind(&ODriveController::set_motor_state_callback, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Created service: %s", motor_state_service.c_str());

    get_calibration_status_service_ = this->create_service<klask_interfaces::srv::GetCalibrationStatus>(
        calibration_status_service,
        std::bind(
            &ODriveController::get_calibration_status_callback, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Created service: %s", calibration_status_service.c_str());

    home_and_calibrate_service_ = this->create_service<klask_interfaces::srv::HomeAndCalibrate>(
        home_calibrate_service,
        std::bind(&ODriveController::home_and_calibrate_callback, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Created service: %s", home_calibrate_service.c_str());

    // Create action clients for homing
    if (players_.find("right_player") != players_.end())
    {
        right_player_homing_client_ =
            rclcpp_action::create_client<klask_interfaces::action::HomePeg>(this, right_action_name);
        RCLCPP_INFO(this->get_logger(), "Created action client for right_player homing: %s", right_action_name.c_str());
    }

    if (players_.find("left_player") != players_.end())
    {
        left_player_homing_client_ =
            rclcpp_action::create_client<klask_interfaces::action::HomePeg>(this, left_action_name);
        RCLCPP_INFO(this->get_logger(), "Created action client for left_player homing: %s", left_action_name.c_str());
    }

    // Create subscriptions for velocity commands with reliable QoS (only for active players)
    if (players_.find("right_player") != players_.end())
    {
        velocity_subscribers_["right_player"] = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_right_topic,
            qos_reliable,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg)
            { this->player_velocity_callback(msg, "right_player"); },
            sub_options);
        RCLCPP_INFO(this->get_logger(), "Subscribed to %s for right_player", cmd_vel_right_topic.c_str());
    }

    if (players_.find("left_player") != players_.end())
    {
        velocity_subscribers_["left_player"] = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_left_topic,
            qos_reliable,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg)
            { this->player_velocity_callback(msg, "left_player"); },
            sub_options);
        RCLCPP_INFO(this->get_logger(), "Subscribed to %s for left_player", cmd_vel_left_topic.c_str());
    }

    RCLCPP_INFO(
        this->get_logger(), "ODriveController initialization complete with %zu active player(s)", players_.size());
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
            calibrated_players +=
                "right=[" + std::to_string(right_peg_pos.x) + ", " + std::to_string(right_peg_pos.y) + "] ";
        }

        if (players_.find("left_player") != players_.end())
        {
            const auto& left_peg_pos = request->state.left_peg.position;
            players_["left_player"]->calibrate(left_peg_pos);
            calibrated_players +=
                "left=[" + std::to_string(left_peg_pos.x) + ", " + std::to_string(left_peg_pos.y) + "] ";
        }

        response->success = true;
        response->message = "Encoders calibrated successfully for active players";
        is_calibrated_.store(true);
        RCLCPP_INFO(this->get_logger(), "Encoders calibrated: %s", calibrated_players.c_str());
    }
    catch (const std::exception& e)
    {
        response->success = false;
        response->message = std::string("Calibration failed: ") + e.what();
        is_calibrated_.store(false);
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
    catch (const std::exception& e)
    {
        response->success = false;
        response->message = std::string("Motor state change failed: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
}

void ODriveController::get_calibration_status_callback(
    const std::shared_ptr<klask_interfaces::srv::GetCalibrationStatus::Request> request,
    std::shared_ptr<klask_interfaces::srv::GetCalibrationStatus::Response> response)
{
    (void)request; // Unused parameter
    response->is_calibrated = is_calibrated_.load();
    if (response->is_calibrated)
    {
        response->message = "System is calibrated and ready";
    }
    else
    {
        response->message = "System is not calibrated";
    }
    RCLCPP_DEBUG(this->get_logger(),
                 "Calibration status queried: %s",
                 response->is_calibrated ? "calibrated" : "not calibrated");
}

void ODriveController::home_and_calibrate_callback(
    const std::shared_ptr<klask_interfaces::srv::HomeAndCalibrate::Request> request,
    std::shared_ptr<klask_interfaces::srv::HomeAndCalibrate::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Home and calibrate service called for player: %s", request->player.c_str());

    // Parse player selection
    std::vector<std::string> selected_players;
    if (request->player == "both")
    {
        if (players_.count("right_player"))
            selected_players.push_back("right_player");
        if (players_.count("left_player"))
            selected_players.push_back("left_player");
    }
    else if (request->player == "right_player" || request->player == "left_player")
    {
        if (players_.count(request->player))
        {
            selected_players.push_back(request->player);
        }
        else
        {
            response->success = false;
            response->message = request->player + " is not active in this configuration";
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }
    }
    else
    {
        response->success = false;
        response->message =
            "Invalid player selection: '" + request->player + "'. Must be 'right_player', 'left_player', or 'both'";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }

    if (selected_players.empty())
    {
        response->success = false;
        response->message = "No active players to home";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }

    // Disable external commands during homing/calibration
    bool previously_enabled = external_commands_enabled_.load();
    set_external_commands_enabled(false);

    // Structure to hold player-specific data (matching motor_commander.cpp)
    struct PlayerHomingData
    {
        std::string name;
        Player::SharedPtr player_node;
        std::shared_ptr<OpenLoopController> homing_controller;
        rclcpp_action::Client<klask_interfaces::action::HomePeg>::SharedPtr homing_client;
        geometry_msgs::msg::Point final_position;
    };

    std::vector<PlayerHomingData> players_to_home;

    try
    {
        using HomePeg = klask_interfaces::action::HomePeg;

        // Create open-loop controllers for homing (action servers) for selected players
        RCLCPP_INFO(this->get_logger(), "Creating open-loop controllers for peg homing...");
        for (const auto& player_name : selected_players)
        {
            PlayerHomingData player_data;
            player_data.name = player_name;
            player_data.player_node = players_[player_name];
            player_data.homing_controller =
                std::make_shared<OpenLoopController>(std::dynamic_pointer_cast<Player>(player_data.player_node));

            RCLCPP_INFO(this->get_logger(), "%s homing controller created", player_name.c_str());
            players_to_home.push_back(player_data);
        }

        RCLCPP_INFO(this->get_logger(), "=== Starting Peg Homing Sequence ===");

        // Create action clients for selected players and wait for servers
        for (auto& player_data : players_to_home)
        {
            std::string action_name = "home_peg_" + player_data.name;
            player_data.homing_client = rclcpp_action::create_client<HomePeg>(shared_from_this(), action_name);

            if (!player_data.homing_client->wait_for_action_server(std::chrono::seconds(action_server_wait_timeout_)))
            {
                throw std::runtime_error(player_data.name + " homing action server not available");
            }
        }

        bool homing_success = true;

        // Home all selected players
        for (size_t i = 0; i < players_to_home.size(); i++)
        {
            auto& player_data = players_to_home[i];

            RCLCPP_INFO(this->get_logger(), "Sending homing goal for %s...", player_data.name.c_str());

            auto goal = HomePeg::Goal();
            goal.home_position.x = 0.0; // Use default
            goal.home_position.y = 0.0;
            goal.home_position.z = 0.0;

            auto goal_handle_future = player_data.homing_client->async_send_goal(goal);

            // Wait for goal to be accepted
            if (goal_handle_future.wait_for(std::chrono::seconds(action_server_wait_timeout_)) !=
                std::future_status::ready)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to send %s homing goal - timeout", player_data.name.c_str());
                homing_success = false;
            }
            else
            {
                auto goal_handle = goal_handle_future.get();
                if (!goal_handle)
                {
                    RCLCPP_ERROR(this->get_logger(), "%s homing goal was rejected", player_data.name.c_str());
                    homing_success = false;
                }
                else
                {
                    // Wait for result
                    auto result_future = player_data.homing_client->async_get_result(goal_handle);
                    if (result_future.wait_for(std::chrono::seconds(homing_action_timeout_)) !=
                        std::future_status::ready)
                    {
                        RCLCPP_ERROR(this->get_logger(), "%s homing action timed out", player_data.name.c_str());
                        homing_success = false;
                    }
                    else
                    {
                        auto result = result_future.get();
                        if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success)
                        {
                            player_data.final_position = result.result->final_position;
                            RCLCPP_INFO(this->get_logger(),
                                        "%s peg homed at [%.3f, %.3f]",
                                        player_data.name.c_str(),
                                        player_data.final_position.x,
                                        player_data.final_position.y);
                        }
                        else
                        {
                            RCLCPP_ERROR(this->get_logger(),
                                         "%s homing failed: %s",
                                         player_data.name.c_str(),
                                         result.result->message.c_str());
                            homing_success = false;
                        }
                    }
                }
            }

            if (!homing_success)
            {
                throw std::runtime_error(player_data.name + " homing failed");
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(inter_homing_delay_));
        }

        RCLCPP_INFO(this->get_logger(), "=== Peg Homing Complete ===");

        // Construct State message from homed positions (only for selected players)
        klask_interfaces::msg::State calibration_state;
        for (const auto& player_data : players_to_home)
        {
            if (player_data.name == "right_player")
            {
                calibration_state.right_peg.position = player_data.final_position;
            }
            else if (player_data.name == "left_player")
            {
                calibration_state.left_peg.position = player_data.final_position;
            }
        }

        // Call calibration with final homed state
        RCLCPP_INFO(this->get_logger(), "Calling calibration service with homed state...");

        auto calib_request = std::make_shared<klask_interfaces::srv::CalibrateEncoders::Request>();
        calib_request->state = calibration_state;
        auto calib_response = std::make_shared<klask_interfaces::srv::CalibrateEncoders::Response>();

        calibrate_encoders_callback(calib_request, calib_response);

        if (!calib_response->success)
        {
            throw std::runtime_error("Calibration failed: " + calib_response->message);
        }

        response->success = true;
        response->message = "Homing and calibration completed successfully";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }
    catch (const std::exception& e)
    {
        response->success = false;
        response->message = std::string("Home and calibrate failed: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        is_calibrated_.store(false);
    }

    // Restore external commands state
    set_external_commands_enabled(previously_enabled);
}

void ODriveController::player_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg,
                                                const std::string& player_name)
{
    // Ignore velocity commands if external commands are disabled
    if (!external_commands_enabled_.load())
    {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(),
                              *this->get_clock(),
                              1000,
                              "Ignoring cmd_vel command for %s - external commands disabled",
                              player_name.c_str());
        return;
    }

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
        RCLCPP_WARN(this->get_logger(), "Received velocity command for inactive player: %s", player_name.c_str());
    }
}

} // namespace klask_motor_commander
