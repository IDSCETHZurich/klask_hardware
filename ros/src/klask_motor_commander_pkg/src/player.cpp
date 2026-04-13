#include "klask_motor_commander_pkg/player.hpp"

namespace klask_motor_commander
{

Player::Player(PlayerSide side)
    : Node(side == PlayerSide::RIGHT_PLAYER ? "right_player" : "left_player")
    , synchronized_state({})
    , magnet_position({0.0f, 0.0f})
    , side_(side)
    , encoder_right(0.0f)
    , encoder_left(0.0f)
    , EDGE(4, 0.0f)
    , callback_count(0)
    , last_time(this->now())
    , deceleration_enabled_(true)
{
    const char* side_name = this->get_name();
    RCLCPP_INFO(this->get_logger(), "Initializing Player node: %s", side_name);

    // === Declare and load ROS parameters ===

    // Physical constants
    this->declare_parameter("distance_per_revolution", 0.04);
    this->declare_parameter("max_velocity", 0.15);
    this->declare_parameter("deceleration_distance", 0.05);
    this->declare_parameter("peg_radius", 0.0075);

    // Motion control
    this->declare_parameter("min_clearance_factor", 1.8);
    this->declare_parameter("corner_x_margin", 0.03);
    this->declare_parameter("corner_y_margin", 0.01);

    // Monitoring
    this->declare_parameter("frequency_log_interval", 100);
    this->declare_parameter("error_throttle_duration", 5000);

    // Service timeouts
    this->declare_parameter("service_wait_timeout", 1);
    this->declare_parameter("max_service_wait_attempts", 5);

    // Load parameters
    distance_per_revolution_ = static_cast<float>(this->get_parameter("distance_per_revolution").as_double());
    max_velocity_ = static_cast<float>(this->get_parameter("max_velocity").as_double());
    deceleration_distance_ = static_cast<float>(this->get_parameter("deceleration_distance").as_double());
    peg_radius_ = static_cast<float>(this->get_parameter("peg_radius").as_double());
    min_clearance_factor_ = static_cast<float>(this->get_parameter("min_clearance_factor").as_double());
    corner_x_margin_ = static_cast<float>(this->get_parameter("corner_x_margin").as_double());
    corner_y_margin_ = static_cast<float>(this->get_parameter("corner_y_margin").as_double());
    frequency_log_interval_ = this->get_parameter("frequency_log_interval").as_int();
    error_throttle_duration_ = this->get_parameter("error_throttle_duration").as_int();
    service_wait_timeout_ = this->get_parameter("service_wait_timeout").as_int();
    max_service_wait_attempts_ = this->get_parameter("max_service_wait_attempts").as_int();

    std::string motor_ind_r;
    std::string motor_ind_l;

    if (side == PlayerSide::RIGHT_PLAYER)
    {
        // Declare and load right player parameters
        this->declare_parameter("right_player.motor_index_right", 0);
        this->declare_parameter("right_player.motor_index_left", 1);
        this->declare_parameter("right_player_edge", std::vector<double>{0.23, 0.42, 0.32, 0.0});

        motor_ind_r = std::to_string(this->get_parameter("right_player.motor_index_right").as_int());
        motor_ind_l = std::to_string(this->get_parameter("right_player.motor_index_left").as_int());

        auto edge_vector = this->get_parameter("right_player_edge").as_double_array();
        EDGE = {static_cast<float>(edge_vector[0]),
                static_cast<float>(edge_vector[1]),
                static_cast<float>(edge_vector[2]),
                static_cast<float>(edge_vector[3])};
        sign = 1.0f;
    }
    else
    { // LEFT_PLAYER
        // Declare and load left player parameters
        this->declare_parameter("left_player.motor_index_right", 2);
        this->declare_parameter("left_player.motor_index_left", 3);
        this->declare_parameter("left_player_edge", std::vector<double>{0.0, 0.19, 0.32, 0.0});

        motor_ind_r = std::to_string(this->get_parameter("left_player.motor_index_right").as_int());
        motor_ind_l = std::to_string(this->get_parameter("left_player.motor_index_left").as_int());

        auto edge_vector = this->get_parameter("left_player_edge").as_double_array();
        EDGE = {static_cast<float>(edge_vector[0]),
                static_cast<float>(edge_vector[1]),
                static_cast<float>(edge_vector[2]),
                static_cast<float>(edge_vector[3])};
        sign = -1.0f;
    }

    RCLCPP_INFO(this->get_logger(),
                "Loaded parameters: max_vel=%.3f, decel_dist=%.3f, peg_radius=%.4f",
                max_velocity_,
                deceleration_distance_,
                peg_radius_);
    RCLCPP_INFO(this->get_logger(), "Edge boundaries: [%.3f, %.3f, %.3f, %.3f]", EDGE[0], EDGE[1], EDGE[2], EDGE[3]);

    RCLCPP_INFO(this->get_logger(),
                "Loaded parameters: max_vel=%.3f, decel_dist=%.3f, peg_radius=%.4f",
                max_velocity_,
                deceleration_distance_,
                peg_radius_);
    RCLCPP_INFO(this->get_logger(), "Edge boundaries: [%.3f, %.3f, %.3f, %.3f]", EDGE[0], EDGE[1], EDGE[2], EDGE[3]);

    group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::PublisherOptions pub_options;
    pub_options.callback_group = group_;

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = group_;

    // Use reliable QoS with KeepAll to match odrive_can_node expectations
    auto qos_control = rclcpp::QoS(rclcpp::KeepAll()).reliable();
    // Use reliable QoS for status updates (ensure delivery)
    auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    // Build topic names using motor indices
    std::string control_topic_r = "/odrive_axis" + motor_ind_r + "/control_message";
    std::string control_topic_l = "/odrive_axis" + motor_ind_l + "/control_message";
    std::string status_topic_r = "/odrive_axis" + motor_ind_r + "/controller_status";
    std::string status_topic_l = "/odrive_axis" + motor_ind_l + "/controller_status";
    std::string service_r = "/odrive_axis" + motor_ind_r + "/request_axis_state";
    std::string service_l = "/odrive_axis" + motor_ind_l + "/request_axis_state";

    std::string topic_prefix = (side == PlayerSide::RIGHT_PLAYER) ? "right_player" : "left_player";
    std::string magnet_topic = "/" + topic_prefix + "_magnet_position";

    motor_left_pub_ =
        this->create_publisher<odrive_can::msg::ControlMessage>(control_topic_l, qos_control, pub_options);
    motor_right_pub_ =
        this->create_publisher<odrive_can::msg::ControlMessage>(control_topic_r, qos_control, pub_options);

    position_magnet_publisher_ =
        this->create_publisher<std_msgs::msg::Float32MultiArray>(magnet_topic, qos_reliable, pub_options);

    RCLCPP_INFO(
        this->get_logger(), "Created publishers for motors %s and %s", motor_ind_r.c_str(), motor_ind_l.c_str());

    request_axis_r_state_client_ =
        this->create_client<odrive_can::srv::AxisState>(service_r, rmw_qos_profile_services_default, group_);
    request_axis_l_state_client_ =
        this->create_client<odrive_can::srv::AxisState>(service_l, rmw_qos_profile_services_default, group_);

    controller_r_subscriber = this->create_subscription<odrive_can::msg::ControllerStatus>(
        status_topic_r,
        qos_reliable,
        std::bind(&Player::controller_status_callback_right, this, std::placeholders::_1),
        sub_options);
    controller_l_subscriber = this->create_subscription<odrive_can::msg::ControllerStatus>(
        status_topic_l,
        qos_reliable,
        std::bind(&Player::controller_status_callback_left, this, std::placeholders::_1),
        sub_options);

    // Initialize motors to CLOSED_LOOP_CONTROL state
    // Wait for services to be available before setting state
    RCLCPP_INFO(this->get_logger(), "Waiting for motor state services...");
    if (request_axis_r_state_client_->wait_for_service(std::chrono::seconds(service_wait_timeout_)) &&
        request_axis_l_state_client_->wait_for_service(std::chrono::seconds(service_wait_timeout_)))
    {
        RCLCPP_INFO(this->get_logger(), "Setting motors to CLOSED_LOOP_CONTROL state...");
        set_motor_state(static_cast<int>(ODriveAxisState::CLOSED_LOOP_CONTROL));
        RCLCPP_INFO(this->get_logger(), "Motors initialized to CLOSED_LOOP_CONTROL");
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Motor state services not available, motors may need manual initialization");
    }

    RCLCPP_INFO(this->get_logger(), "Player node %s initialization complete", side_name);
}

void Player::send_commands(float v_x, float v_y)
{
    // Clamp input velocities to safe range
    v_x = std::clamp(v_x, -max_velocity_, max_velocity_);
    v_y = std::clamp(v_y, -max_velocity_, max_velocity_);

    // Apply deceleration profile near boundaries if calibrated and enabled
    if (deceleration_enabled_.load() && !synchronized_state.empty())
    {
        deacceleration_profile(v_x, v_y);
    }

    // Log frequency periodically
    if (++callback_count >= frequency_log_interval_)
    {
        const rclcpp::Time current_time = this->now();
        const double dt = (current_time - last_time).seconds();
        if (dt > 0.0)
        {
            const double freq = frequency_log_interval_ / dt;
            RCLCPP_INFO(this->get_logger(), "Motor commands freq: %.2f Hz", freq);
        }
        last_time = current_time;
        callback_count = 0;
    }

    // Convert velocities to motor commands (differential drive)
    const float meters_to_rotations = 2.0f / distance_per_revolution_;

    odrive_can::msg::ControlMessage control_r_msg;
    control_r_msg.input_vel = sign * (v_x + v_y) * meters_to_rotations;
    control_r_msg.control_mode = static_cast<int>(ODriveControlMode::VELOCITY_CONTROL);
    control_r_msg.input_mode = static_cast<int>(ODriveInputMode::VEL_RAMP);

    odrive_can::msg::ControlMessage control_l_msg;
    control_l_msg.input_vel = sign * (v_y - v_x) * meters_to_rotations;
    control_l_msg.control_mode = static_cast<int>(ODriveControlMode::VELOCITY_CONTROL);
    control_l_msg.input_mode = static_cast<int>(ODriveInputMode::VEL_RAMP);

    motor_right_pub_->publish(control_r_msg);
    motor_left_pub_->publish(control_l_msg);
}

void Player::calibrate(const geometry_msgs::msg::Point& peg_pos)
{
    synchronized_state = {0.0f, 0.0f, 0.0f, 0.0f};
    synchronized_state[0] = encoder_right;
    synchronized_state[1] = encoder_left;
    synchronized_state[2] = static_cast<float>(peg_pos.x);
    synchronized_state[3] = static_cast<float>(peg_pos.y);

    RCLCPP_INFO(this->get_logger(),
                "Calibrated: encoders=[%.3f, %.3f], position=[%.3f, %.3f]",
                encoder_right,
                encoder_left,
                peg_pos.x,
                peg_pos.y);
}

void Player::change_motor_state(const int& des_state)
{
    set_motor_state(des_state);
}

void Player::deacceleration_profile(float& v_x, float& v_y)
{
    // Helper lambda to compute linear deceleration
    auto linear_decel = [this](float distance) -> float { return (max_velocity_ / deceleration_distance_) * distance; };

    const float safe_zone = deceleration_distance_ + peg_radius_;
    const float min_clearance = peg_radius_ * min_clearance_factor_;

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

    // // Handle unreachable front corners (geometry constraints)
    // const bool near_top = magnet_position[1] >= EDGE[3] - corner_y_margin_;
    // const bool near_bottom = magnet_position[1] <= EDGE[2] + corner_y_margin_;
    // const bool in_corner_y = near_top || near_bottom;

    // if (side_ == PlayerSide::LEFT_PLAYER && magnet_position[0] >= EDGE[1] - corner_x_margin_ && in_corner_y)
    // {
    //     v_x = std::min(0.0f, v_x); // Prevent moving further right
    //     v_y = near_top ? std::min(v_y, 0.0f) : std::max(v_y, 0.0f);
    // }
    // else if (side_ == PlayerSide::RIGHT_PLAYER && magnet_position[0] <= EDGE[0] + corner_x_margin_ && in_corner_y)
    // {
    //     v_x = std::max(0.0f, v_x); // Prevent moving further left
    //     v_y = near_top ? std::min(v_y, 0.0f) : std::max(v_y, 0.0f);
    // }
}

void Player::controller_status_callback_right(const odrive_can::msg::ControllerStatus::SharedPtr msg)
{
    encoder_right = msg->pos_estimate;
    update_magnet_pos();

    if (msg->active_errors != 0)
    {
        RCLCPP_FATAL(this->get_logger(), "Right motor error detected: 0x%X. Shutting down.", msg->active_errors);
        if (fatal_error_callback_)
        {
            fatal_error_callback_();
        }
    }
}

void Player::controller_status_callback_left(const odrive_can::msg::ControllerStatus::SharedPtr msg)
{
    encoder_left = msg->pos_estimate;
    update_magnet_pos();

    if (msg->active_errors != 0)
    {
        RCLCPP_FATAL(this->get_logger(), "Left motor error detected: 0x%X. Shutting down.", msg->active_errors);
        if (fatal_error_callback_)
        {
            fatal_error_callback_();
        }
    }
}

void Player::update_magnet_pos()
{
    if (synchronized_state.empty())
    {
        return; // Early exit if not calibrated
    }

    // Calculate encoder deltas
    const float delta_encoder_r = (encoder_right - synchronized_state[0]) * distance_per_revolution_;
    const float delta_encoder_l = (encoder_left - synchronized_state[1]) * distance_per_revolution_;

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
    int wait_attempts = 0;

    while (!request_axis_r_state_client_->wait_for_service(std::chrono::seconds(service_wait_timeout_)))
    {
        if (++wait_attempts >= max_service_wait_attempts_)
        {
            RCLCPP_ERROR(this->get_logger(), "Right motor service not available after timeout");
            return;
        }
        RCLCPP_WARN(this->get_logger(), "Waiting for right motor service...");
    }

    wait_attempts = 0;
    while (!request_axis_l_state_client_->wait_for_service(std::chrono::seconds(service_wait_timeout_)))
    {
        if (++wait_attempts >= max_service_wait_attempts_)
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

} // namespace klask_motor_commander
