#include "klask_motor_commander_pkg/open_loop_controller.hpp"
#include "klask_motor_commander_pkg/player.hpp"
#include <cmath>
#include <chrono>
#include <thread>

using namespace std::placeholders;

namespace klask_motor_commander
{

OpenLoopController::OpenLoopController(std::shared_ptr<Player> player)
    : Node("open_loop_controller_" +
           std::string(player->get_side() == PlayerSide::RIGHT_PLAYER ? "right_player" : "left_player"))
    , player_(player)
    , player_name_(player->get_side() == PlayerSide::RIGHT_PLAYER ? "right_player" : "left_player")
    , waiting_for_initial_state_(false)
{
    RCLCPP_INFO(this->get_logger(), "Initializing OpenLoopController for %s...", player_name_.c_str());

    // === Declare and load ROS parameters ===

    // Movement validation parameters
    this->declare_parameter("movement_validation_duration", 2.0);
    this->declare_parameter("movement_threshold", 0.01);

    // Control loop parameters
    this->declare_parameter("control_frequency", 10.0);

    // Timeout settings
    this->declare_parameter("initial_state_timeout", 5.0);

    // Load parameters
    validation_duration_ = static_cast<float>(this->get_parameter("movement_validation_duration").as_double());
    movement_threshold_ = static_cast<float>(this->get_parameter("movement_threshold").as_double());
    control_frequency_ = this->get_parameter("control_frequency").as_double();
    initial_state_timeout_ = this->get_parameter("initial_state_timeout").as_double();
    validation_active_ = false;

    RCLCPP_INFO(this->get_logger(), "Loaded parameters: control_freq=%.1f Hz", control_frequency_);
    RCLCPP_INFO(this->get_logger(),
                "Movement validation: duration=%.1fs, threshold=%.4fm",
                validation_duration_,
                movement_threshold_);

    // Subscribe to board state
    this->declare_parameter("board_state_topic", "/board_state");
    this->declare_parameter("board_state_qos_depth", 10);

    std::string board_state_topic = this->get_parameter("board_state_topic").as_string();
    int qos_depth = this->get_parameter("board_state_qos_depth").as_int();

    board_state_sub_ = this->create_subscription<klask_interfaces::msg::State>(
        board_state_topic,
        rclcpp::QoS(qos_depth).reliable(),
        std::bind(&OpenLoopController::board_state_callback, this, _1));

    // Create control loop timer
    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency_));
    control_timer_ = this->create_wall_timer(timer_period, std::bind(&OpenLoopController::control_loop_callback, this));

    // Create action server
    action_server_ = rclcpp_action::create_server<HomePeg>(this,
                                                           "home_peg_" + player_name_,
                                                           std::bind(&OpenLoopController::handle_goal, this, _1, _2),
                                                           std::bind(&OpenLoopController::handle_cancel, this, _1),
                                                           std::bind(&OpenLoopController::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "OpenLoopController for %s initialized", player_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Action server 'home_peg_%s' ready", player_name_.c_str());
}

rclcpp_action::GoalResponse OpenLoopController::handle_goal(const rclcpp_action::GoalUUID& uuid,
                                                            std::shared_ptr<const HomePeg::Goal> goal)
{
    (void)uuid;
    RCLCPP_INFO(this->get_logger(),
                "%s: Received homing goal request to position [%.3f, %.3f]",
                player_name_.c_str(),
                goal->home_position.x,
                goal->home_position.y);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse OpenLoopController::handle_cancel(const std::shared_ptr<GoalHandleHomePeg> goal_handle)
{
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "%s: Received cancel request", player_name_.c_str());
    return rclcpp_action::CancelResponse::ACCEPT;
}

void OpenLoopController::handle_accepted(const std::shared_ptr<GoalHandleHomePeg> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "%s: Starting peg homing sequence...", player_name_.c_str());

    const auto goal = goal_handle->get_goal();

    // Use goal home position provided by caller
    geometry_msgs::msg::Point target_home = goal->home_position;

    // Get homing parameters from goal
    homing_velocity_ = goal->homing_velocity;
    position_tolerance_ = goal->position_tolerance;

    RCLCPP_INFO(this->get_logger(),
                "%s: Homing to position [%.3f, %.3f] with velocity %.3f m/s, tolerance %.3f m",
                player_name_.c_str(),
                target_home.x,
                target_home.y,
                homing_velocity_,
                position_tolerance_);

    // Set goal and activate homing
    target_goal_ = target_home;
    active_goal_handle_ = goal_handle;
    goal_accepted_time_ = this->now();

    // Set flag to wait for a NEW board state message
    waiting_for_initial_state_ = true;
    RCLCPP_INFO(this->get_logger(), "%s: Waiting for fresh board_state message...", player_name_.c_str());
}

void OpenLoopController::board_state_callback(const klask_interfaces::msg::State::SharedPtr msg)
{
    // State is already in engineering units (meters, m/s) from state estimator
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_state_ = msg;
    state_received_ = true;

    // If we were waiting for initial state, trigger movement validation start
    if (waiting_for_initial_state_.load())
    {
        waiting_for_initial_state_ = false;
    }
}

float OpenLoopController::calculate_distance(const geometry_msgs::msg::Point& p1,
                                             const geometry_msgs::msg::Point& p2) const
{
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}

geometry_msgs::msg::Point OpenLoopController::get_current_peg_position() const
{
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(state_mutex_));
    if (!latest_state_)
    {
        throw std::runtime_error(player_name_ + ": No state available");
    }

    if (player_name_ == "right_player")
    {
        return latest_state_->right_peg.position;
    }
    else
    {
        return latest_state_->left_peg.position;
    }
}

void OpenLoopController::control_loop_callback()
{
    // Check if we have an active goal
    if (!active_goal_handle_)
    {
        return;
    }

    // If waiting for initial state, check timeout and log
    if (waiting_for_initial_state_.load())
    {
        double elapsed = (this->now() - goal_accepted_time_).seconds();

        if (elapsed >= initial_state_timeout_)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "%s: Timeout waiting for board_state message (%.1f seconds). Aborting homing.",
                         player_name_.c_str(),
                         initial_state_timeout_);

            waiting_for_initial_state_ = false;

            // Stop motors
            player_->send_commands(0.0f, 0.0f);

            // Abort action
            auto result = std::make_shared<HomePeg::Result>();
            result->success = false;
            result->message = "Timeout waiting for board_state";
            result->final_position = geometry_msgs::msg::Point();
            active_goal_handle_->abort(result);
            active_goal_handle_.reset();
            return;
        }

        // Log progress
        RCLCPP_INFO_THROTTLE(this->get_logger(),
                             *this->get_clock(),
                             1000,
                             "%s: Waiting for board_state message (%.1f s elapsed)...",
                             player_name_.c_str(),
                             elapsed);
        return;
    }

    // Get current position
    geometry_msgs::msg::Point current_pos;
    try
    {
        current_pos = get_current_peg_position();
    }
    catch (const std::exception& e)
    {
        // State not available
        return;
    }

    // Check if this is the first control loop after state received (start validation)
    if (validation_active_ == false && validation_timer_ == nullptr)
    {
        RCLCPP_INFO(this->get_logger(),
                    "%s: Board state received! Initial position: [%.3f, %.3f], Target: [%.3f, %.3f]",
                    player_name_.c_str(),
                    current_pos.x,
                    current_pos.y,
                    target_goal_.x,
                    target_goal_.y);
        start_movement_validation(current_pos);
    }

    // Check for cancellation
    if (active_goal_handle_->is_canceling())
    {
        player_->send_commands(0.0f, 0.0f);
        auto result = std::make_shared<HomePeg::Result>();
        result->success = false;
        result->message = "Homing canceled";
        result->final_position = current_pos;
        active_goal_handle_->canceled(result);
        active_goal_handle_.reset();
        RCLCPP_INFO(this->get_logger(), "%s: Homing canceled", player_name_.c_str());
        return;
    }

    // Calculate distance to goal
    float distance = calculate_distance(current_pos, target_goal_);

    // Check if we reached the goal
    if (distance <= position_tolerance_)
    {
        player_->send_commands(0.0f, 0.0f);
        auto result = std::make_shared<HomePeg::Result>();
        result->success = true;
        result->message = "Homing completed successfully";
        result->final_position = current_pos;
        active_goal_handle_->succeed(result);
        active_goal_handle_.reset();
        RCLCPP_INFO(this->get_logger(), "%s: Reached home position!", player_name_.c_str());
        return;
    }

    // Calculate direction vector to goal
    float dx = target_goal_.x - current_pos.x;
    float dy = target_goal_.y - current_pos.y;

    // Normalize direction
    float dir_x = dx / distance;
    float dir_y = dy / distance;

    // Calculate velocity commands
    float v_x = dir_x * homing_velocity_;
    float v_y = dir_y * homing_velocity_;

    // Send velocity command
    player_->send_commands(v_x, v_y);

    // Publish feedback
    auto feedback = std::make_shared<HomePeg::Feedback>();
    feedback->current_position = current_pos;
    feedback->distance_remaining = distance;
    active_goal_handle_->publish_feedback(feedback);
}

void OpenLoopController::start_movement_validation(const geometry_msgs::msg::Point& start_pos)
{
    // Store starting position
    validation_start_position_ = start_pos;

    RCLCPP_INFO(this->get_logger(),
                "%s: Movement validation started from [%.3f, %.3f]. Will check if peg moved after %.1f seconds.",
                player_name_.c_str(),
                start_pos.x,
                start_pos.y,
                validation_duration_);

    // Cancel existing timer if any
    if (validation_timer_)
    {
        validation_timer_->cancel();
    }

    // Create one-shot timer for validation
    validation_active_ = true;
    validation_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(validation_duration_ * 1000)),
                                std::bind(&OpenLoopController::movement_validation_callback, this));
}

void OpenLoopController::movement_validation_callback()
{
    // Cancel timer (one-shot)
    if (validation_timer_)
    {
        validation_timer_->cancel();
        validation_timer_.reset();
    }

    if (!validation_active_ || !active_goal_handle_)
    {
        return;
    }

    validation_active_ = false;

    // Get current position
    if (!latest_state_)
    {
        RCLCPP_WARN(this->get_logger(), "%s: No state available for movement validation", player_name_.c_str());
        return;
    }

    geometry_msgs::msg::Point current_pos = get_current_peg_position();

    // Calculate distance moved from start
    float dx = current_pos.x - validation_start_position_.x;
    float dy = current_pos.y - validation_start_position_.y;
    float distance_moved = std::sqrt(dx * dx + dy * dy);

    RCLCPP_INFO(this->get_logger(),
                "%s: Movement validation: peg moved %.4f m from start position (threshold: %.4f m)",
                player_name_.c_str(),
                distance_moved,
                movement_threshold_);

    if (distance_moved < movement_threshold_)
    {
        // Peg didn't move enough
        RCLCPP_ERROR(this->get_logger(),
                     "%s: Peg not moving! Moved only %.4f m < threshold %.4f m after %.1f seconds. "
                     "Possible desynchronization between magnetic and physical peg.",
                     player_name_.c_str(),
                     distance_moved,
                     movement_threshold_,
                     validation_duration_);

        // Stop motors
        player_->send_commands(0.0f, 0.0f);

        // Abort action
        auto result = std::make_shared<HomePeg::Result>();
        result->success = false;
        result->message = "Peg not moving - possible desynchronization";
        result->final_position = current_pos;
        active_goal_handle_->abort(result);
        active_goal_handle_.reset();
    }
    else
    {
        RCLCPP_INFO(
            this->get_logger(), "%s: Movement validation passed! Peg is moving correctly.", player_name_.c_str());
    }
}
} // namespace klask_motor_commander
