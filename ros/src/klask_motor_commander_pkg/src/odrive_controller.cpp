#include "klask_motor_commander_pkg/odrive_controller.hpp"
#include "klask_motor_commander_pkg/open_loop_controller.hpp"
#include <thread>

namespace klask_motor_commander
{

ODriveController::ODriveController(PlayerSide player_config)
    : Node("odrive_controller")
    , external_commands_enabled_(true)
    , left_is_calibrated_(false)
    , right_is_calibrated_(false)
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
    this->declare_parameter("is_player_homed_service", "is_player_homed");
    this->declare_parameter("set_motor_state_service", "set_motor_state");

    // Action names for homing
    this->declare_parameter("right_player_action_name", "home_peg_right_player");
    this->declare_parameter("left_player_action_name", "home_peg_left_player");
    this->declare_parameter("right_player_home_calibrate_action", "home_and_calibrate_right_player");
    this->declare_parameter("left_player_home_calibrate_action", "home_and_calibrate_left_player");

    // Homing timeouts
    this->declare_parameter("action_server_wait_timeout", 5);
    this->declare_parameter("homing_action_timeout", 30);
    this->declare_parameter("inter_homing_delay", 500);

    // Home position parameters
    this->declare_parameter("right_player_home_x", 0.31);
    this->declare_parameter("right_player_home_y", 0.16);
    this->declare_parameter("left_player_home_x", 0.11);
    this->declare_parameter("left_player_home_y", 0.16);
    this->declare_parameter("position_tolerance", 0.01);
    this->declare_parameter("homing_velocity", 0.03);

    // QoS settings
    this->declare_parameter("cmd_vel_qos_depth", 1);

    // Initial motor state
    this->declare_parameter("initial_motor_state", 8);

    // Heartbeat parameters
    this->declare_parameter("heartbeat_topic", "heartbeat");
    this->declare_parameter("heartbeat_frequency_hz", 5.0);

    // Load parameters
    std::string cmd_vel_right_topic = this->get_parameter("cmd_vel_right_player").as_string();
    std::string cmd_vel_left_topic = this->get_parameter("cmd_vel_left_player").as_string();
    std::string calibrate_service = this->get_parameter("calibrate_encoders_service").as_string();
    std::string calibration_status_service = this->get_parameter("get_calibration_status_service").as_string();
    std::string is_player_homed_service = this->get_parameter("is_player_homed_service").as_string();
    std::string motor_state_service = this->get_parameter("set_motor_state_service").as_string();
    std::string right_action_name = this->get_parameter("right_player_action_name").as_string();
    std::string left_action_name = this->get_parameter("left_player_action_name").as_string();
    std::string right_home_calibrate_action = this->get_parameter("right_player_home_calibrate_action").as_string();
    std::string left_home_calibrate_action = this->get_parameter("left_player_home_calibrate_action").as_string();
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

    // Create home and calibrate action servers for active players
    if (players_.find("right_player") != players_.end())
    {
        right_player_home_and_calibrate_server_ =
            rclcpp_action::create_server<klask_interfaces::action::HomeAndCalibrate>(
                this,
                right_home_calibrate_action,
                [this](const rclcpp_action::GoalUUID& uuid,
                       std::shared_ptr<const klask_interfaces::action::HomeAndCalibrate::Goal> goal)
                { return this->handle_home_and_calibrate_goal(uuid, goal, "right_player"); },
                std::bind(&ODriveController::handle_home_and_calibrate_cancel, this, std::placeholders::_1),
                [this](
                    const std::shared_ptr<rclcpp_action::ServerGoalHandle<klask_interfaces::action::HomeAndCalibrate>>
                        goal_handle) { this->handle_home_and_calibrate_accepted(goal_handle, "right_player"); });
        RCLCPP_INFO(this->get_logger(), "Created action server: %s", right_home_calibrate_action.c_str());
    }

    if (players_.find("left_player") != players_.end())
    {
        left_player_home_and_calibrate_server_ =
            rclcpp_action::create_server<klask_interfaces::action::HomeAndCalibrate>(
                this,
                left_home_calibrate_action,
                [this](const rclcpp_action::GoalUUID& uuid,
                       std::shared_ptr<const klask_interfaces::action::HomeAndCalibrate::Goal> goal)
                { return this->handle_home_and_calibrate_goal(uuid, goal, "left_player"); },
                std::bind(&ODriveController::handle_home_and_calibrate_cancel, this, std::placeholders::_1),
                [this](
                    const std::shared_ptr<rclcpp_action::ServerGoalHandle<klask_interfaces::action::HomeAndCalibrate>>
                        goal_handle) { this->handle_home_and_calibrate_accepted(goal_handle, "left_player"); });
        RCLCPP_INFO(this->get_logger(), "Created action server: %s", left_home_calibrate_action.c_str());
    }

    is_player_homed_service_ = this->create_service<klask_interfaces::srv::IsPlayerHomed>(
        is_player_homed_service,
        std::bind(&ODriveController::is_player_homed_callback, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Created service: %s", is_player_homed_service.c_str());

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

    // Create heartbeat publisher with best-effort QoS for periodic liveness signal
    std::string heartbeat_topic = this->get_parameter("heartbeat_topic").as_string();
    double heartbeat_freq = this->get_parameter("heartbeat_frequency_hz").as_double();
    auto heartbeat_qos = rclcpp::QoS(1).best_effort().durability_volatile();
    heartbeat_publisher_ = this->create_publisher<std_msgs::msg::Empty>(heartbeat_topic, heartbeat_qos);

    auto heartbeat_period = std::chrono::duration<double>(1.0 / heartbeat_freq);
    heartbeat_timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(heartbeat_period),
                                               std::bind(&ODriveController::heartbeat_timer_callback, this),
                                               group_);
    RCLCPP_INFO(
        this->get_logger(), "Heartbeat publisher created on '%s' at %.1f Hz", heartbeat_topic.c_str(), heartbeat_freq);

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
        // Validate player parameter
        if (request->player != "right_player" && request->player != "left_player" && request->player != "both")
        {
            response->success = false;
            response->message =
                "Invalid player: '" + request->player + "'. Must be 'right_player', 'left_player', or 'both'";
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        // Extract peg positions from State message and calibrate specified player(s)
        std::string calibrated_players;
        bool calibrated_any = false;

        if ((request->player == "right_player" || request->player == "both") &&
            players_.find("right_player") != players_.end())
        {
            const auto& right_peg_pos = request->state.right_peg.position;
            players_["right_player"]->calibrate(right_peg_pos);
            calibrated_players +=
                "right=[" + std::to_string(right_peg_pos.x) + ", " + std::to_string(right_peg_pos.y) + "] ";
            right_is_calibrated_.store(true);
            calibrated_any = true;
        }

        if ((request->player == "left_player" || request->player == "both") &&
            players_.find("left_player") != players_.end())
        {
            const auto& left_peg_pos = request->state.left_peg.position;
            players_["left_player"]->calibrate(left_peg_pos);
            calibrated_players +=
                "left=[" + std::to_string(left_peg_pos.x) + ", " + std::to_string(left_peg_pos.y) + "] ";
            left_is_calibrated_.store(true);
            calibrated_any = true;
        }

        if (calibrated_any)
        {
            response->success = true;
            response->message = "Encoders calibrated successfully for " + calibrated_players;
            RCLCPP_INFO(this->get_logger(), "Encoders calibrated: %s", calibrated_players.c_str());
        }
        else
        {
            response->success = false;
            response->message = "Player '" + request->player + "' is not active in this configuration";
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }
    catch (const std::exception& e)
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
    // Validate player parameter
    if (request->player != "right_player" && request->player != "left_player" && request->player != "both")
    {
        response->is_calibrated = false;
        response->message =
            "Invalid player: '" + request->player + "'. Must be 'right_player', 'left_player', or 'both'";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }

    bool all_calibrated = true;
    std::string status_details;

    if (request->player == "right_player" || request->player == "both")
    {
        if (players_.find("right_player") != players_.end())
        {
            bool right_cal = right_is_calibrated_.load();
            all_calibrated &= right_cal;
            status_details += "right_player: " + std::string(right_cal ? "calibrated" : "not calibrated");
            if (request->player == "both")
                status_details += " ";
        }
        else if (request->player == "right_player")
        {
            response->is_calibrated = false;
            response->message = "right_player is not active in this configuration";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }
    }

    if (request->player == "left_player" || request->player == "both")
    {
        if (players_.find("left_player") != players_.end())
        {
            bool left_cal = left_is_calibrated_.load();
            all_calibrated &= left_cal;
            status_details += "left_player: " + std::string(left_cal ? "calibrated" : "not calibrated");
        }
        else if (request->player == "left_player")
        {
            response->is_calibrated = false;
            response->message = "left_player is not active in this configuration";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }
    }

    response->is_calibrated = all_calibrated;
    response->message = status_details;
    RCLCPP_DEBUG(
        this->get_logger(), "Calibration status queried for %s: %s", request->player.c_str(), status_details.c_str());
}

void ODriveController::is_player_homed_callback(
    const std::shared_ptr<klask_interfaces::srv::IsPlayerHomed::Request> request,
    std::shared_ptr<klask_interfaces::srv::IsPlayerHomed::Response> response)
{
    RCLCPP_DEBUG(this->get_logger(), "Is player homed service called for player: %s", request->player.c_str());

    // Validate player name
    if (request->player != "right_player" && request->player != "left_player")
    {
        response->is_homed = false;
        response->message = "Invalid player name: '" + request->player + "'. Must be 'right_player' or 'left_player'";
        response->distance = -1.0f;
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }

    // Check if player exists
    auto it = players_.find(request->player);
    if (it == players_.end())
    {
        response->is_homed = false;
        response->message = request->player + " is not active in this configuration";
        response->distance = -1.0f;
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }

    try
    {
        // Get home position for the player
        float home_x, home_y;
        if (request->player == "right_player")
        {
            home_x = static_cast<float>(this->get_parameter("right_player_home_x").as_double());
            home_y = static_cast<float>(this->get_parameter("right_player_home_y").as_double());
        }
        else
        {
            home_x = static_cast<float>(this->get_parameter("left_player_home_x").as_double());
            home_y = static_cast<float>(this->get_parameter("left_player_home_y").as_double());
        }

        // Get position tolerance (slightly increased tolerance to avoid borderline cases)
        float tolerance = static_cast<float>(this->get_parameter("position_tolerance").as_double()) * 1.1f;

        // Get current magnet position from player
        auto player_node = it->second;
        if (player_node->magnet_position.size() < 2)
        {
            response->is_homed = false;
            response->message = "Player position data not available";
            response->distance = -1.0f;
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        float current_x = player_node->magnet_position[0];
        float current_y = player_node->magnet_position[1];

        // Calculate distance from home
        float dx = current_x - home_x;
        float dy = current_y - home_y;
        float distance = std::sqrt(dx * dx + dy * dy);

        response->distance = distance;
        response->is_homed = (distance <= tolerance);

        if (response->is_homed)
        {
            response->message = request->player + " is at home position (distance: " + std::to_string(distance) +
                                "m, tolerance: " + std::to_string(tolerance) + "m)";
            RCLCPP_DEBUG(this->get_logger(), "%s", response->message.c_str());
        }
        else
        {
            response->message = request->player + " is NOT at home position (distance: " + std::to_string(distance) +
                                "m, tolerance: " + std::to_string(tolerance) + "m)";
            RCLCPP_DEBUG(this->get_logger(), "%s", response->message.c_str());
        }
    }
    catch (const std::exception& e)
    {
        response->is_homed = false;
        response->message = std::string("Error checking home status: ") + e.what();
        response->distance = -1.0f;
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
}

rclcpp_action::GoalResponse ODriveController::handle_home_and_calibrate_goal(
    const rclcpp_action::GoalUUID& /* uuid */,
    std::shared_ptr<const klask_interfaces::action::HomeAndCalibrate::Goal> /* goal */,
    const std::string& player_name)
{
    RCLCPP_INFO(this->get_logger(), "Home and calibrate action goal received for player: %s", player_name.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ODriveController::handle_home_and_calibrate_cancel(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<klask_interfaces::action::HomeAndCalibrate>> /* goal_handle */)
{
    RCLCPP_INFO(this->get_logger(), "Received cancel request for home and calibrate action");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ODriveController::handle_home_and_calibrate_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<klask_interfaces::action::HomeAndCalibrate>> goal_handle,
    const std::string& player_name)
{
    // Execute the homing sequence in a separate thread to allow the action server
    // to return immediately and respond to the goal acceptance request
    std::thread([this, goal_handle, player_name]() { this->execute_home_and_calibrate(goal_handle, player_name); })
        .detach();
}

void ODriveController::execute_home_and_calibrate(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<klask_interfaces::action::HomeAndCalibrate>> goal_handle,
    const std::string& player_name)
{
    using HomeAndCalibrate = klask_interfaces::action::HomeAndCalibrate;
    auto result = std::make_shared<HomeAndCalibrate::Result>();
    auto feedback = std::make_shared<HomeAndCalibrate::Feedback>();

    RCLCPP_INFO(this->get_logger(), "Home and calibrate action executing for player: %s", player_name.c_str());

    // Validate that player exists
    if (players_.count(player_name) == 0)
    {
        result->success = false;
        result->message = player_name + " is not active in this configuration";
        RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
        goal_handle->abort(result);
        return;
    }

    // Reset calibration flag for this specific player at the start of homing sequence
    if (player_name == "right_player")
    {
        right_is_calibrated_.store(false);
    }
    else if (player_name == "left_player")
    {
        left_is_calibrated_.store(false);
    }
    RCLCPP_INFO(this->get_logger(), "Calibration flag reset for %s - starting homing sequence", player_name.c_str());

    // Disable external commands during homing/calibration
    // Only save the state if we're actually changing it (to avoid race conditions with multiple players)
    bool previously_enabled = external_commands_enabled_.load();
    bool we_disabled_commands = false;
    if (previously_enabled)
    {
        set_external_commands_enabled(false);
        we_disabled_commands = true;
    }

    // Variables to hold homing controller and executor
    std::shared_ptr<OpenLoopController> homing_controller;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor;
    std::unique_ptr<std::thread> spin_thread;
    rclcpp_action::Client<klask_interfaces::action::HomePeg>::SharedPtr homing_client;
    geometry_msgs::msg::Point final_position;

    try
    {
        using HomePeg = klask_interfaces::action::HomePeg;

        // Create open-loop controller for homing (action server)
        RCLCPP_INFO(this->get_logger(), "Creating open-loop controller for %s peg homing...", player_name.c_str());

        homing_controller =
            std::make_shared<OpenLoopController>(std::dynamic_pointer_cast<Player>(players_[player_name]));

        // Create an executor for the homing controller and spin it in a separate thread
        executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor->add_node(homing_controller);
        spin_thread = std::make_unique<std::thread>([executor]() { executor->spin(); });

        RCLCPP_INFO(this->get_logger(), "%s homing controller created", player_name.c_str());

        RCLCPP_INFO(this->get_logger(), "=== Starting Peg Homing Sequence ===");

        // Create action client and wait for server
        std::string action_name = "home_peg_" + player_name;
        homing_client = rclcpp_action::create_client<HomePeg>(shared_from_this(), action_name);

        if (!homing_client->wait_for_action_server(std::chrono::seconds(action_server_wait_timeout_)))
        {
            throw std::runtime_error(player_name + " homing action server not available");
        }

        // Check if action was cancelled
        if (goal_handle->is_canceling())
        {
            result->success = false;
            result->message = "Home and calibrate action cancelled";
            RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());
            goal_handle->canceled(result);
            // Only restore external commands if we were the ones who disabled them
            if (we_disabled_commands)
            {
                set_external_commands_enabled(true);
            }
            // Stop executor before returning
            if (executor)
            {
                executor->cancel();
            }
            if (spin_thread && spin_thread->joinable())
            {
                spin_thread->join();
            }
            return;
        }

        // Publish feedback
        feedback->current_phase = "homing";
        feedback->status_message = std::string("Homing ") + player_name;
        goal_handle->publish_feedback(feedback);

        RCLCPP_INFO(this->get_logger(), "Sending homing goal for %s...", player_name.c_str());

        auto homing_goal = HomePeg::Goal();
        // Set home position from parameters
        if (player_name == "right_player")
        {
            homing_goal.home_position.x = this->get_parameter("right_player_home_x").as_double();
            homing_goal.home_position.y = this->get_parameter("right_player_home_y").as_double();
        }
        else
        {
            homing_goal.home_position.x = this->get_parameter("left_player_home_x").as_double();
            homing_goal.home_position.y = this->get_parameter("left_player_home_y").as_double();
        }
        homing_goal.home_position.z = 0.0;
        // Set homing parameters
        homing_goal.homing_velocity = static_cast<float>(this->get_parameter("homing_velocity").as_double());
        homing_goal.position_tolerance = static_cast<float>(this->get_parameter("position_tolerance").as_double());

        auto goal_handle_future = homing_client->async_send_goal(homing_goal);

        // Wait for goal to be accepted
        if (goal_handle_future.wait_for(std::chrono::seconds(action_server_wait_timeout_)) != std::future_status::ready)
        {
            throw std::runtime_error("Failed to send " + player_name + " homing goal - timeout");
        }

        auto homing_goal_handle = goal_handle_future.get();
        if (!homing_goal_handle)
        {
            throw std::runtime_error(player_name + " homing goal was rejected");
        }

        // Wait for result
        auto result_future = homing_client->async_get_result(homing_goal_handle);
        if (result_future.wait_for(std::chrono::seconds(homing_action_timeout_)) != std::future_status::ready)
        {
            throw std::runtime_error(player_name + " homing action timed out");
        }

        auto homing_result = result_future.get();
        if (homing_result.code != rclcpp_action::ResultCode::SUCCEEDED || !homing_result.result->success)
        {
            throw std::runtime_error(player_name + " homing failed: " + homing_result.result->message);
        }

        final_position = homing_result.result->final_position;
        RCLCPP_INFO(this->get_logger(),
                    "%s peg homed at [%.3f, %.3f]",
                    player_name.c_str(),
                    final_position.x,
                    final_position.y);

        RCLCPP_INFO(this->get_logger(), "=== Peg Homing Complete ===");

        // Publish calibration feedback
        feedback->current_phase = "calibrating";
        feedback->status_message = "Calibrating encoders";
        goal_handle->publish_feedback(feedback);

        // Construct State message from homed position
        klask_interfaces::msg::State calibration_state;
        if (player_name == "right_player")
        {
            calibration_state.right_peg.position = final_position;
        }
        else if (player_name == "left_player")
        {
            calibration_state.left_peg.position = final_position;
        }

        // Call calibration with final homed state for the specific player
        RCLCPP_INFO(this->get_logger(), "Calling calibration service for %s...", player_name.c_str());

        auto calib_request = std::make_shared<klask_interfaces::srv::CalibrateEncoders::Request>();
        calib_request->state = calibration_state;
        calib_request->player = player_name; // Specify which player to calibrate
        auto calib_response = std::make_shared<klask_interfaces::srv::CalibrateEncoders::Response>();

        calibrate_encoders_callback(calib_request, calib_response);

        if (!calib_response->success)
        {
            throw std::runtime_error("Calibration failed: " + calib_response->message);
        }

        result->success = true;
        result->message = "Homing and calibration completed successfully";
        RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());
        goal_handle->succeed(result);
    }
    catch (const std::exception& e)
    {
        result->success = false;
        result->message = std::string("Home and calibrate failed: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
        // Reset calibration flag for this specific player on failure
        if (player_name == "right_player")
        {
            right_is_calibrated_.store(false);
        }
        else if (player_name == "left_player")
        {
            left_is_calibrated_.store(false);
        }
        goal_handle->abort(result);
    }

    // Stop executor thread and clean up homing controller
    if (executor)
    {
        executor->cancel();
    }
    if (spin_thread && spin_thread->joinable())
    {
        spin_thread->join();
    }

    // Restore external commands state only if we were the ones who disabled them
    if (we_disabled_commands)
    {
        set_external_commands_enabled(true);
    }
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

void ODriveController::heartbeat_timer_callback()
{
    heartbeat_publisher_->publish(std_msgs::msg::Empty());
}

} // namespace klask_motor_commander
