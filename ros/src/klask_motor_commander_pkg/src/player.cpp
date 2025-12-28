#include "klask_motor_commander_pkg/player.hpp"

Player::Player(PlayerSide side)
    : Node(side == PlayerSide::RIGHT_PLAYER ? "right_player" : "left_player"),
      synchronized_state({}),
      magnet_position({0.0f, 0.0f}),
      side_(side),
      encoder_right(0.0f),
      encoder_left(0.0f),
      EDGE(4, 0.0f),
      callback_count(0),
      last_time(this->now())
{
    const char *side_name = this->get_name();
    RCLCPP_INFO(this->get_logger(), "Initializing Player node: %s", side_name);

    std::string motor_ind_r;
    std::string motor_ind_l;

    if (side == PlayerSide::RIGHT_PLAYER)
    {
        motor_ind_r = "0";
        motor_ind_l = "1";
        EDGE = {0.23f, 0.42f, 0.32f, 0.0f}; // [left, right, bottom, top] in image coords (y=0 is top)
        sign = 1.0f;
    }
    else
    { // LEFT_PLAYER
        motor_ind_r = "2";
        motor_ind_l = "3";
        EDGE = {0.0f, 0.19f, 0.32f, 0.0f}; // [left, right, bottom, top] in image coords (y=0 is top)
        sign = -1.0f;
    }

    group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::PublisherOptions pub_options;
    pub_options.callback_group = group_;

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = group_;

    // Use reliable QoS with KeepAll to match odrive_can_node expectations
    auto qos_control = rclcpp::QoS(rclcpp::KeepAll()).reliable();
    // Use reliable QoS for status updates (ensure delivery)
    auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    motor_left_pub_ = this->create_publisher<odrive_can::msg::ControlMessage>(
        "/odrive_axis" + motor_ind_l + "/control_message", qos_control, pub_options);
    motor_right_pub_ = this->create_publisher<odrive_can::msg::ControlMessage>(
        "/odrive_axis" + motor_ind_r + "/control_message", qos_control, pub_options);

    std::string topic_prefix = (side == PlayerSide::RIGHT_PLAYER) ? "right_player" : "left_player";
    position_magnet_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/" + topic_prefix + "_magnet_position", qos_reliable, pub_options);

    RCLCPP_INFO(this->get_logger(), "Created publishers for motors %s and %s",
                motor_ind_r.c_str(), motor_ind_l.c_str());

    request_axis_r_state_client_ = this->create_client<odrive_can::srv::AxisState>(
        "/odrive_axis" + motor_ind_r + "/request_axis_state",
        rmw_qos_profile_services_default,
        group_);
    request_axis_l_state_client_ = this->create_client<odrive_can::srv::AxisState>(
        "/odrive_axis" + motor_ind_l + "/request_axis_state",
        rmw_qos_profile_services_default,
        group_);

    RCLCPP_INFO(this->get_logger(), "Initializing motors to idle state...");
    set_motor_state(1);
    RCLCPP_INFO(this->get_logger(), "Setting motors to closed loop control...");
    set_motor_state(8);

    controller_r_subscriber = this->create_subscription<odrive_can::msg::ControllerStatus>(
        "/odrive_axis" + motor_ind_r + "/controller_status", qos_reliable,
        std::bind(&Player::controller_status_callback_right, this, std::placeholders::_1), sub_options);
    controller_l_subscriber = this->create_subscription<odrive_can::msg::ControllerStatus>(
        "/odrive_axis" + motor_ind_l + "/controller_status", qos_reliable,
        std::bind(&Player::controller_status_callback_left, this, std::placeholders::_1), sub_options);

    RCLCPP_INFO(this->get_logger(), "Player node %s initialization complete", side_name);
}

void Player::send_commands(float v_x, float v_y)
{
    // Clamp input velocities to safe range
    v_x = std::clamp(v_x, -MAX_VEL, MAX_VEL);
    v_y = std::clamp(v_y, -MAX_VEL, MAX_VEL);

    // Apply deceleration profile near boundaries if calibrated
    if (!synchronized_state.empty())
    {
        deacceleration_profile(v_x, v_y);
    }

    // Log frequency periodically
    constexpr int FREQ_LOG_INTERVAL = 100;
    if (++callback_count >= FREQ_LOG_INTERVAL)
    {
        const rclcpp::Time current_time = this->now();
        const double dt = (current_time - last_time).seconds();
        if (dt > 0.0)
        {
            const double freq = FREQ_LOG_INTERVAL / dt;
            RCLCPP_INFO(this->get_logger(), "Motor commands freq: %.2f Hz", freq);
        }
        last_time = current_time;
        callback_count = 0;
    }

    // Convert velocities to motor commands (differential drive)
    constexpr float METERS_TO_ROTATIONS = 2.0f / DISTANCE_PER_REVOLUTION;
    constexpr int VELOCITY_CONTROL_MODE = 2;
    constexpr int VELOCITY_INPUT_MODE = 1;

    odrive_can::msg::ControlMessage control_r_msg;
    control_r_msg.input_vel = sign * (v_x + v_y) * METERS_TO_ROTATIONS;
    control_r_msg.control_mode = VELOCITY_CONTROL_MODE;
    control_r_msg.input_mode = VELOCITY_INPUT_MODE;

    odrive_can::msg::ControlMessage control_l_msg;
    control_l_msg.input_vel = sign * (v_y - v_x) * METERS_TO_ROTATIONS;
    control_l_msg.control_mode = VELOCITY_CONTROL_MODE;
    control_l_msg.input_mode = VELOCITY_INPUT_MODE;

    motor_right_pub_->publish(control_r_msg);
    motor_left_pub_->publish(control_l_msg);
}

void Player::calibrate(const geometry_msgs::msg::Point &peg_pos)
{
    synchronized_state = {0.0f, 0.0f, 0.0f, 0.0f};
    synchronized_state[0] = encoder_right;
    synchronized_state[1] = encoder_left;
    synchronized_state[2] = static_cast<float>(peg_pos.x);
    synchronized_state[3] = static_cast<float>(peg_pos.y);

    RCLCPP_INFO(this->get_logger(),
                "Calibrated: encoders=[%.3f, %.3f], position=[%.3f, %.3f]",
                encoder_right, encoder_left, peg_pos.x, peg_pos.y);
}

void Player::change_motor_state(const int &des_state)
{
    set_motor_state(des_state);
}

void Player::deacceleration_profile(float &v_x, float &v_y)
{
    // Helper lambda to compute linear deceleration
    auto linear_decel = [](float distance) -> float
    {
        return (MAX_VEL / DEACCELERATION_DISTANCE) * distance;
    };

    const float safe_zone = DEACCELERATION_DISTANCE + PEG_RADIUS;
    const float min_clearance = PEG_RADIUS * 1.5f;

    // X-axis boundary checks
    const float dist_from_left = magnet_position[0] - EDGE[0];
    const float dist_from_right = EDGE[1] - magnet_position[0];

    // Near left boundary - limit leftward motion
    if (dist_from_left <= safe_zone)
    {
        if (dist_from_left <= min_clearance)
        {
            v_x = std::max(v_x, 0.0f); // Block leftward motion
        }
        else
        {
            v_x = std::max(v_x, -linear_decel(dist_from_left - min_clearance));
        }
    }

    // Near right boundary - limit rightward motion
    if (dist_from_right <= safe_zone)
    {
        if (dist_from_right <= min_clearance)
        {
            v_x = std::min(v_x, 0.0f); // Block rightward motion
        }
        else
        {
            v_x = std::min(v_x, linear_decel(dist_from_right - min_clearance));
        }
    }

    // Y-axis boundary checks (image coordinates: y=0 is top, y increases downward)
    const float dist_from_top = magnet_position[1] - EDGE[3];
    const float dist_from_bottom = EDGE[2] - magnet_position[1];

    // Near top boundary - limit upward motion (negative y velocity)
    if (dist_from_top <= safe_zone)
    {
        if (dist_from_top <= min_clearance)
        {
            v_y = std::max(v_y, 0.0f); // Block upward motion
        }
        else
        {
            v_y = std::max(v_y, -linear_decel(dist_from_top - min_clearance));
        }
    }
    // Near bottom boundary - limit downward motion (positive y velocity)
    if (dist_from_bottom <= safe_zone)
    {

        if (dist_from_bottom <= min_clearance)
        {
            v_y = std::min(v_y, 0.0f); // Block downward motion
        }
        else
        {
            v_y = std::min(v_y, linear_decel(dist_from_bottom - min_clearance));
        }
    }

    // Handle unreachable front corners (geometry constraints)
    constexpr float CORNER_X_MARGIN = 0.03f;
    constexpr float CORNER_Y_MARGIN = 0.01f;

    const bool near_top = magnet_position[1] >= EDGE[3] - CORNER_Y_MARGIN;
    const bool near_bottom = magnet_position[1] <= EDGE[2] + CORNER_Y_MARGIN;
    const bool in_corner_y = near_top || near_bottom;

    if (side_ == PlayerSide::LEFT_PLAYER &&
        magnet_position[0] >= EDGE[1] - CORNER_X_MARGIN && in_corner_y)
    {
        v_x = std::min(0.0f, v_x); // Prevent moving further right
        v_y = near_top ? std::min(v_y, 0.0f) : std::max(v_y, 0.0f);
    }
    else if (side_ == PlayerSide::RIGHT_PLAYER &&
             magnet_position[0] <= EDGE[0] + CORNER_X_MARGIN && in_corner_y)
    {
        v_x = std::max(0.0f, v_x); // Prevent moving further left
        v_y = near_top ? std::min(v_y, 0.0f) : std::max(v_y, 0.0f);
    }
}

void Player::controller_status_callback_right(const odrive_can::msg::ControllerStatus::SharedPtr msg)
{
    encoder_right = msg->pos_estimate;
    update_magnet_pos();

    if (msg->active_errors != 0)
    {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000, // Log at most once every 5 seconds
            "Right motor error detected: 0x%X. Attempting to recover.",
            msg->active_errors);
        // Note: Avoid blocking sleep in callbacks - let the system handle recovery
        set_motor_state(1); // Set to idle state
    }
}

void Player::controller_status_callback_left(const odrive_can::msg::ControllerStatus::SharedPtr msg)
{
    encoder_left = msg->pos_estimate;
    update_magnet_pos();

    if (msg->active_errors != 0)
    {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000, // Log at most once every 5 seconds
            "Left motor error detected: 0x%X. Attempting to recover.",
            msg->active_errors);
        set_motor_state(1); // Set to idle state
    }
}

void Player::update_magnet_pos()
{
    if (synchronized_state.empty())
    {
        return; // Early exit if not calibrated
    }

    // Calculate encoder deltas
    const float delta_encoder_r = (encoder_right - synchronized_state[0]) * DISTANCE_PER_REVOLUTION;
    const float delta_encoder_l = (encoder_left - synchronized_state[1]) * DISTANCE_PER_REVOLUTION;

    // Differential drive kinematics: x is differential, y is average
    constexpr float HALF = 0.5f;
    magnet_position.resize(2);
    magnet_position[0] = synchronized_state[2] + sign * HALF * (delta_encoder_r - delta_encoder_l);
    magnet_position[1] = synchronized_state[3] + sign * HALF * (delta_encoder_r + delta_encoder_l);

    // Publish estimated position
    std_msgs::msg::Float32MultiArray msg;
    msg.data = magnet_position;
    position_magnet_publisher_->publish(msg);
}

void Player::set_motor_state(int state)
{
    auto request_r = std::make_shared<odrive_can::srv::AxisState::Request>();
    request_r->axis_requested_state = state;
    auto request_l = std::make_shared<odrive_can::srv::AxisState::Request>();
    request_l->axis_requested_state = state;

    // Wait for services with timeout to prevent infinite blocking
    constexpr int MAX_WAIT_ATTEMPTS = 5;
    int wait_attempts = 0;

    while (!request_axis_r_state_client_->wait_for_service(std::chrono::seconds(1)))
    {
        if (++wait_attempts >= MAX_WAIT_ATTEMPTS)
        {
            RCLCPP_ERROR(this->get_logger(), "Right motor service not available after timeout");
            return;
        }
        RCLCPP_WARN(this->get_logger(), "Waiting for right motor service...");
    }

    wait_attempts = 0;
    while (!request_axis_l_state_client_->wait_for_service(std::chrono::seconds(1)))
    {
        if (++wait_attempts >= MAX_WAIT_ATTEMPTS)
        {
            RCLCPP_ERROR(this->get_logger(), "Left motor service not available after timeout");
            return;
        }
        RCLCPP_WARN(this->get_logger(), "Waiting for left motor service...");
    }

    // Send requests asynchronously
    auto future_r = request_axis_r_state_client_->async_send_request(request_r);
    auto future_l = request_axis_l_state_client_->async_send_request(request_l);

    RCLCPP_DEBUG(this->get_logger(), "Motor state change to %d requested", state);
}
