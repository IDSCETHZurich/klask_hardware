#include "klask_motor_commander_pkg/player.hpp"

Player::Player(PlayerSide side)
    : Node(side == PlayerSide::RIGHT_PLAYER ? "right_player" : "left_player"),
      finding_peg(false),
      peg_mag_synchronized(false),
      synchronized_once(false),
      position({0.0f, 0.0f}),
      velocity({0.0f, 0.0f}),
      synchronized_state({0.0f, 0.0f, 0.0f, 0.0f}),
      encoder_values({0.0f, 0.0f, 0.0f, 0.0f}),
      side_(side),
      synch_buffer(100, false),
      sync_elements_to_check(0.0f),
      motor_at_wall(false),
      motor_at_wall_calib(false),
      motor_at_wall_buffer(4, std::vector<bool>(7, false)),
      wall_prot_activated(4, false),
      torque_left_motor(0.0f),
      torque_right_motor(0.0f),
      home(4, 0.0f),
      EDGE(4, 0.0f),
      at_home(false),
      magnet_position({0.0f, 0.0f}),
      magnet_velocity({0.0f, 0.0f})
{
    const char *side_name = (side == PlayerSide::RIGHT_PLAYER) ? "right_player" : "left_player";
    RCLCPP_INFO(this->get_logger(), "Initializing Player node: %s", side_name);

    std::string motor_ind_r;
    std::string motor_ind_l;

    if (side == PlayerSide::RIGHT_PLAYER)
    {
        motor_ind_r = "0";
        motor_ind_l = "1";
        home = {410.0f, 175.0f};
        EDGE = {283.0f, 528.0f, 5.0f, 367.0f};
        sign = 1.0f;
    }
    else
    { // LEFT_PLAYER
        motor_ind_r = "2";
        motor_ind_l = "3";
        home = {110.0f, 175.0f};
        EDGE = {5.0f, 250.0f, 5.0f, 367.0f};
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

void Player::velocity_targets(float v_x, float v_y)
{
    if (peg_mag_synchronized && !finding_peg)
    {
        publish_velocity(v_x, v_y);
    }
    else if (!finding_peg)
    {
        find_peg();
    }
}

void Player::is_synchronized()
{
    float delta = 7.0f;

    std::copy_backward(synch_buffer.begin(), synch_buffer.end() - 1, synch_buffer.end());
    if (velocity[0] < delta && velocity[1] < delta)
    {
        if (magnet_velocity[0] < delta && magnet_velocity[1] < delta && peg_mag_synchronized)
        {
            synch_buffer[0] = true;
        }
        else
        {
            synch_buffer[0] = false;
        }
    }
    else
    {
        synch_buffer[0] = true;
    }

    float mag_vel = std::sqrt(magnet_velocity[0] * magnet_velocity[0] +
                              magnet_velocity[1] * magnet_velocity[1]);
    sync_elements_to_check = 0.4 * mag_vel + 0.6 * sync_elements_to_check;
    unsigned int elements_to_check = std::min(
        static_cast<unsigned int>(synch_buffer.size()) - 1,
        interpolate_elements(sync_elements_to_check));

    if (std::all_of(synch_buffer.begin(), synch_buffer.begin() + elements_to_check,
                    [](bool val)
                    { return !val; }))
    {
        if (peg_mag_synchronized)
        {
            RCLCPP_INFO(this->get_logger(), "Peg and magnet desynchronized");
        }
        peg_mag_synchronized = false;
    }
    else if (std::all_of(synch_buffer.begin(), synch_buffer.begin() + 3,
                         [](bool val)
                         { return val; }))
    {
        if (!peg_mag_synchronized)
        {
            RCLCPP_INFO(this->get_logger(), "Peg and magnet synchronized");
        }
        peg_mag_synchronized = true;
    }
}

void Player::find_peg()
{
    finding_peg = true;
    RCLCPP_INFO(this->get_logger(), "In Function");
    while (!peg_mag_synchronized)
    {
        move_to_corner();
        move_pattern();
    }
    synchronized_once = true;
    find_home();
    finding_peg = false;
}

void Player::move_to_corner()
{
    bool in_corner = false;
    bool at_edge = false;

    rclcpp::Rate loop_rate(50);

    while (rclcpp::ok() && (!in_corner && !peg_mag_synchronized))
    {
        if (motor_at_wall_calib && !at_edge)
        {
            at_edge = true;
            send_commands(-sign * 0.008f, 0.0f);
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }
        else if (!motor_at_wall_calib && at_edge)
        {
            send_commands(0.0f, sign * 0.008f);
        }
        else if (motor_at_wall_calib && at_edge)
        {
            send_commands(0.0f, -sign * 0.008f);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            send_commands(0.0f, 0.0f);

            // Set corner position: right_player at x=530, left_player at x=0
            magnet_position[0] = (side_ == PlayerSide::RIGHT_PLAYER) ? 530.0f : 0.0f;
            // Set corner position: left_player at y=370, right_player at y=0
            magnet_position[1] = (side_ == PlayerSide::LEFT_PLAYER) ? 370.0f : 0.0f;
            synchronized_state = {encoder_values[0], encoder_values[1],
                                  magnet_position[0], magnet_position[1]};
            RCLCPP_INFO(this->get_logger(), "Peg in corner");

            in_corner = true;
        }
        else
        {
            send_commands(sign * 0.008f, 0.0f);
        }

        is_synchronized();
        loop_rate.sleep();
    }
}

void Player::move_pattern()
{
    bool moving_in_x;
    bool moving_in_y;
    bool moving_in_pos_x;
    bool moving_in_pos_y;
    float x_target;
    float y_target;

    if (side_ == PlayerSide::RIGHT_PLAYER)
    {
        moving_in_x = true;
        moving_in_y = false;
        moving_in_pos_x = false;
        moving_in_pos_y = false;
        x_target = magnet_position[0] - 200.0f;
        y_target = magnet_position[1] - 30.0f;
    }
    else
    { // LEFT_PLAYER
        moving_in_x = true;
        moving_in_y = false;
        moving_in_pos_x = true;
        moving_in_pos_y = true;
        x_target = magnet_position[0] + 200.0f;
        y_target = magnet_position[1] + 30.0f;
    }

    rclcpp::Rate loop_rate(50);
    while (rclcpp::ok() && !peg_mag_synchronized)
    {
        float v_x = moving_in_x ? (moving_in_pos_x ? 0.02f : -0.02f) : 0.0f;
        float v_y = moving_in_y ? (moving_in_pos_y ? 0.02f : -0.02f) : 0.0f;

        publish_velocity(v_x, v_y);

        if (moving_in_x && std::fabs(magnet_position[0] - x_target) < 10.0f)
        {
            moving_in_pos_x = !moving_in_pos_x;
            moving_in_y = true;
            moving_in_x = false;
            x_target = magnet_position[0] + 190.0f * (moving_in_pos_x ? 1.0f : -1.0f);
        }
        else if (moving_in_y && magnet_position[1] - y_target < 5.0f)
        {
            moving_in_y = false;
            moving_in_x = true;
            y_target = magnet_position[1] - 30.0f;
            if (y_target < 0.0f)
            {
                break;
            }
        }
        is_synchronized();
        loop_rate.sleep();
    }
}

void Player::find_home()
{
    const float epsilon = 4;
    float v_x = 0.0;
    float v_y = 0.0;
    rclcpp::Rate loop_rate(20);
    while (!at_home && peg_mag_synchronized)
    {
        if (std::abs(position[0] - home[0]) > epsilon)
        {
            v_x = std::copysign(0.005f, home[0] - position[0]);
        }
        else
        {
            v_x = 0.0;
        }
        if (std::abs(position[1] - home[1]) > epsilon)
        {
            v_y = std::copysign(0.005f, home[1] - position[1]);
        }
        else
        {
            v_y = 0.0;
        }
        publish_velocity(v_x, v_y);
        loop_rate.sleep();
        if (v_x == 0.0 && v_y == 0.0)
        {
            publish_velocity(0.0f, 0.0f);
            RCLCPP_INFO(this->get_logger(), "Peg is at home");
            at_home = true;
            synchronized_state = {0.0f, 0.0f, 0.0f, 0.0f};
            synchronized_state[0] = encoder_values[0];
            synchronized_state[1] = encoder_values[1];
            synchronized_state[2] = position[0];
            synchronized_state[3] = position[1];
        }
    }
}

unsigned int Player::interpolate_elements(float x) const
{
    static float x1 = 5.875f;
    static float x2 = 235.0f;
    static float y1 = 50.0f;
    static float y2 = 7.0f;
    unsigned int y = std::ceil(((y2 - y1) / (x2 - x1)) * x + y1 - x1 * ((y2 - y1) / (x2 - x1)));
    return y;
}

void Player::publish_velocity(float v_x, float v_y)
{
    float mag_pos = 0.0f;
    int counter = 0;
    for (auto &row : motor_at_wall_buffer)
    {
        std::copy_backward(row.begin(), row.end() - 1, row.end());
        row[0] = false;
        if (counter > 1)
        {
            mag_pos = magnet_position[1];
        }
        else
        {
            mag_pos = magnet_position[0];
        }
        if (motor_at_wall && std::fabs(mag_pos - EDGE[counter]) < 15.0f)
        {
            row[0] = true;
        }
        else if (std::fabs(mag_pos - EDGE[counter]) > 35.0f)
        {
            if (wall_prot_activated[counter])
            {
                RCLCPP_INFO(this->get_logger(), "Stopped hitting against %u st wall", counter);
            }
            wall_prot_activated[counter] = false;
        }
        counter++;
    }
    deaceleration_profile(v_x, v_y);
    move_away_from_wall(v_x, v_y);

    send_commands(v_x, v_y);
}

void Player::deaceleration_profile(float &v_x, float &v_y)
{
    float distance = 35.0f;
    float kf_safety_width = 5.0f;
    float speed_limit_weight = 20.0f;

    if (position[0] <= EDGE[0] + distance)
    {
        v_x = std::max(v_x, std::abs(v_x) * std::tanh((EDGE[0] - position[0] - kf_safety_width) / speed_limit_weight));
    }
    else if (position[0] + distance >= EDGE[1])
    {
        v_x = std::min(std::abs(v_x) * std::tanh((EDGE[1] - position[0] + kf_safety_width) / speed_limit_weight), v_x);
    }
    if (position[1] <= EDGE[2] + distance)
    {
        v_y = std::max(v_y, std::abs(v_y) * std::tanh((EDGE[2] - position[1] - kf_safety_width) / speed_limit_weight));
    }
    else if (position[1] + distance >= EDGE[3])
    {
        v_y = std::min(v_y, std::abs(v_y) * std::tanh((EDGE[3] - position[1] + kf_safety_width) / speed_limit_weight));
    }
}

void Player::move_away_from_wall(float &v_x, float &v_y)
{
    if (wall_prot_activated[0] || std::any_of(motor_at_wall_buffer[0].begin(), motor_at_wall_buffer[0].end(), [](bool b)
                                              { return b; }))
    {
        v_x = std::max(0.0f, v_x);
        if (!wall_prot_activated[0])
        {
            RCLCPP_INFO(this->get_logger(), "Hitting against 1st wall");
        }
        wall_prot_activated[0] = true;
    }
    else if (wall_prot_activated[1] || std::any_of(motor_at_wall_buffer[1].begin(), motor_at_wall_buffer[1].end(), [](bool b)
                                                   { return b; }))
    {
        v_x = std::min(0.0f, v_x);
        if (!wall_prot_activated[1])
        {
            RCLCPP_INFO(this->get_logger(), "Hitting against 2st wall");
        }
        wall_prot_activated[1] = true;
    }
    if (wall_prot_activated[2] || std::any_of(motor_at_wall_buffer[2].begin(), motor_at_wall_buffer[2].end(), [](bool b)
                                              { return b; }))
    {
        v_y = std::max(0.0f, v_y);
        if (!wall_prot_activated[2])
        {
            RCLCPP_INFO(this->get_logger(), "Hitting against 3st wall");
        }
        wall_prot_activated[2] = true;
    }
    else if (wall_prot_activated[3] || std::any_of(motor_at_wall_buffer[3].begin(), motor_at_wall_buffer[3].end(), [](bool b)
                                                   { return b; }))
    {
        v_y = std::min(0.0f, v_y);
        if (!wall_prot_activated[3])
        {
            RCLCPP_INFO(this->get_logger(), "Hitting against 4st wall");
        }
        wall_prot_activated[3] = true;
    }
}

void Player::send_commands(float v_x, float v_y)
{
    odrive_can::msg::ControlMessage control_r_msg;
    odrive_can::msg::ControlMessage control_l_msg;

    control_r_msg.input_vel = sign * (v_x + v_y) * (2 / DISTANCE_PER_REVOLUTION);
    control_r_msg.control_mode = 2;
    control_r_msg.input_mode = 1;
    control_l_msg.input_vel = sign * (v_y - v_x) * (2 / DISTANCE_PER_REVOLUTION);
    control_l_msg.control_mode = 2;
    control_l_msg.input_mode = 1;
    motor_right_pub_->publish(control_r_msg);
    motor_left_pub_->publish(control_l_msg);
}

void Player::controller_status_callback_right(const odrive_can::msg::ControllerStatus::SharedPtr msg)
{
    // Update encoder state
    encoder_values[0] = msg->pos_estimate;
    encoder_values[2] = msg->vel_estimate;
    torque_right_motor = msg->torque_estimate;

    // Check torque limits and handle collision detection
    float total_torque = calculate_total_torque();
    handle_torque_limits(total_torque);

    // Update magnet position estimate
    update_magnet_position(encoder_values);

    // Check for motor errors
    if (msg->active_errors != 0)
    {
        send_commands(0.0f, 0.0f);
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000, // Log at most once every 5 seconds
            "Right motor error detected: 0x%X. Stopping motors.",
            msg->active_errors);
        // Note: Removed long sleep that blocks the callback thread
        set_motor_state(1);
    }
}

void Player::controller_status_callback_left(const odrive_can::msg::ControllerStatus::SharedPtr msg)
{
    // Update encoder state
    encoder_values[1] = msg->pos_estimate;
    encoder_values[3] = msg->vel_estimate;
    torque_left_motor = msg->torque_estimate;

    // Check torque limits and handle collision detection
    float total_torque = calculate_total_torque();
    handle_torque_limits(total_torque);

    // Update magnet position estimate
    update_magnet_position(encoder_values);

    // Check for motor errors
    if (msg->active_errors != 0)
    {
        send_commands(0.0f, 0.0f);
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000, // Log at most once every 5 seconds
            "Left motor error detected: 0x%X. Stopping motors.",
            msg->active_errors);
        set_motor_state(1);
    }
}

void Player::update_magnet_position(const std::vector<float> &encoder_values)
{
    if (!synchronized_state.empty())
    {
        magnet_position[0] = synchronized_state[2] +
                             (1.0f / 2.0f) * ((encoder_values[0] - synchronized_state[0]) * DISTANCE_PER_REVOLUTION - (encoder_values[1] - synchronized_state[1]) * DISTANCE_PER_REVOLUTION) * REAL_TO_CAM_FACTOR_X;
        magnet_position[1] = synchronized_state[3] +
                             (1.0f / 2.0f) * ((encoder_values[0] - synchronized_state[0]) * DISTANCE_PER_REVOLUTION + (encoder_values[1] - synchronized_state[1]) * DISTANCE_PER_REVOLUTION) * REAL_TO_CAM_FACTOR_Y;
    }
    magnet_velocity[0] = (1.0f / 2.0f) * (encoder_values[2] * DISTANCE_PER_REVOLUTION - encoder_values[3] * DISTANCE_PER_REVOLUTION) * REAL_TO_CAM_FACTOR_X;
    magnet_velocity[1] = (1.0f / 2.0f) * (encoder_values[2] * DISTANCE_PER_REVOLUTION + encoder_values[3] * DISTANCE_PER_REVOLUTION) * REAL_TO_CAM_FACTOR_Y;

    std_msgs::msg::Float32MultiArray msg;
    msg.data.resize(4);
    msg.data[0] = magnet_position[0];
    msg.data[1] = magnet_position[1];
    msg.data[2] = magnet_velocity[0];
    msg.data[3] = magnet_velocity[1];
    position_magnet_publisher_->publish(msg);
}

void Player::set_motor_state(int state)
{
    auto request_r = std::make_shared<odrive_can::srv::AxisState::Request>();
    request_r->axis_requested_state = state;
    auto request_l = std::make_shared<odrive_can::srv::AxisState::Request>();
    request_l->axis_requested_state = state;

    while (rclcpp::ok() && !request_axis_r_state_client_->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(this->get_logger(), "Waiting for service /odrive_axis0/request_axis_state...");
    }
    while (rclcpp::ok() && !request_axis_l_state_client_->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(this->get_logger(), "Waiting for service /odrive_axis1/request_axis_state...");
    }

    // If node is shutting down, don't send requests
    if (!rclcpp::ok())
    {
        return;
    }

    auto future_r = request_axis_r_state_client_->async_send_request(request_r);
    auto future_l = request_axis_l_state_client_->async_send_request(request_l);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_r);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_l);
}

float Player::calculate_total_torque() const
{
    return std::sqrt(std::pow(torque_right_motor, 2) + std::pow(torque_left_motor, 2));
}

void Player::handle_torque_limits(float total_torque)
{
    if (total_torque > SHUTDOWN_TORQUE)
    {
        send_commands(0.0f, 0.0f);
        RCLCPP_ERROR(this->get_logger(), "Shutdown torque (%.3f N·m) surpassed! Emergency stop.", total_torque);
        set_motor_state(1); // Set to idle
        return;
    }

    if (total_torque > MAX_TORQUE)
    {
        motor_at_wall = true;
        if (total_torque > CALIB_TORQUE)
        {
            motor_at_wall_calib = true;
        }
    }
    else
    {
        motor_at_wall = false;
        motor_at_wall_calib = false;
    }
}
