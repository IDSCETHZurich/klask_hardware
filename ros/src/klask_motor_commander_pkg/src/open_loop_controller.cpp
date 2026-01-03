#include "klask_motor_commander_pkg/open_loop_controller.hpp"
#include "klask_motor_commander_pkg/player.hpp"
#include <cmath>
#include <chrono>
#include <thread>

using namespace std::placeholders;

namespace klask_motor_commander {

OpenLoopController::OpenLoopController(std::shared_ptr<Player> player)
    : Node("open_loop_controller_" + std::string(player->get_side() == PlayerSide::RIGHT_PLAYER ? "right_player" : "left_player")),
      player_(player),
      player_name_(player->get_side() == PlayerSide::RIGHT_PLAYER ? "right_player" : "left_player")
{
    RCLCPP_INFO(this->get_logger(), "Initializing OpenLoopController for %s...", player_name_.c_str());

    // === Declare and load ROS parameters ===

    // Homing motion parameters
    this->declare_parameter("homing_velocity", 0.01);
    this->declare_parameter("position_tolerance", 0.03);

    // Movement validation parameters
    this->declare_parameter("validation_duration", 2.0);
    this->declare_parameter("movement_threshold", 0.01);
    this->declare_parameter("expected_distance_factor", 0.5);

    // Fast feedback loop parameters
    this->declare_parameter("feedback_velocity_multiplier", 2.0);
    this->declare_parameter("feedback_step_duration", 0.1);

    // Timeout settings
    this->declare_parameter("initial_state_timeout", 5.0);
    this->declare_parameter("state_update_timeout", 2.0);
    this->declare_parameter("feedback_state_timeout", 0.5);

    // Load parameters
    homing_velocity_ = static_cast<float>(this->get_parameter("homing_velocity").as_double());
    position_tolerance_ = static_cast<float>(this->get_parameter("position_tolerance").as_double());
    validation_duration_ = static_cast<float>(this->get_parameter("validation_duration").as_double());
    movement_threshold_ = static_cast<float>(this->get_parameter("movement_threshold").as_double());
    expected_distance_factor_ = static_cast<float>(this->get_parameter("expected_distance_factor").as_double());
    feedback_velocity_multiplier_ = static_cast<float>(this->get_parameter("feedback_velocity_multiplier").as_double());
    feedback_step_duration_ = static_cast<float>(this->get_parameter("feedback_step_duration").as_double());
    initial_state_timeout_ = this->get_parameter("initial_state_timeout").as_double();
    state_update_timeout_ = this->get_parameter("state_update_timeout").as_double();
    feedback_state_timeout_ = this->get_parameter("feedback_state_timeout").as_double();

    // Set default home position based on player
    if (player_->get_side() == PlayerSide::RIGHT_PLAYER)
    {
        this->declare_parameter("right_player_home_x", 0.26);
        this->declare_parameter("right_player_home_y", 0.16);
        default_home_.x = this->get_parameter("right_player_home_x").as_double();
        default_home_.y = this->get_parameter("right_player_home_y").as_double();
    }
    else
    {
        this->declare_parameter("left_player_home_x", 0.16);
        this->declare_parameter("left_player_home_y", 0.16);
        default_home_.x = this->get_parameter("left_player_home_x").as_double();
        default_home_.y = this->get_parameter("left_player_home_y").as_double();
    }
    default_home_.z = 0.0;

    RCLCPP_INFO(this->get_logger(), "Loaded parameters: homing_vel=%.3f, pos_tol=%.3f, validation_dur=%.1f",
                homing_velocity_, position_tolerance_, validation_duration_);

    // Subscribe to board state
    this->declare_parameter("board_state_topic", "/board_state");
    this->declare_parameter("board_state_qos_depth", 10);
    
    std::string board_state_topic = this->get_parameter("board_state_topic").as_string();
    int qos_depth = this->get_parameter("board_state_qos_depth").as_int();
    
    board_state_sub_ = this->create_subscription<klask_interfaces::msg::State>(
        board_state_topic,
        rclcpp::QoS(qos_depth).reliable(),
        std::bind(&OpenLoopController::board_state_callback, this, _1));

    // Create action server
    action_server_ = rclcpp_action::create_server<HomePeg>(
        this,
        "home_peg_" + player_name_,
        std::bind(&OpenLoopController::handle_goal, this, _1, _2),
        std::bind(&OpenLoopController::handle_cancel, this, _1),
        std::bind(&OpenLoopController::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "OpenLoopController for %s initialized. Default home position: [%.3f, %.3f]",
                player_name_.c_str(), default_home_.x, default_home_.y);
    RCLCPP_INFO(this->get_logger(), "Action server 'home_peg_%s' ready", player_name_.c_str());
}

rclcpp_action::GoalResponse OpenLoopController::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const HomePeg::Goal> goal)
{
    (void)uuid;
    RCLCPP_INFO(this->get_logger(),
                "%s: Received homing goal request to position [%.3f, %.3f]",
                player_name_.c_str(), goal->home_position.x, goal->home_position.y);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse OpenLoopController::handle_cancel(
    const std::shared_ptr<GoalHandleHomePeg> goal_handle)
{
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "%s: Received cancel request", player_name_.c_str());
    return rclcpp_action::CancelResponse::ACCEPT;
}

void OpenLoopController::handle_accepted(const std::shared_ptr<GoalHandleHomePeg> goal_handle)
{
    // Execute in a new thread to avoid blocking
    std::thread{std::bind(&OpenLoopController::execute, this, _1), goal_handle}.detach();
}

void OpenLoopController::execute(const std::shared_ptr<GoalHandleHomePeg> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "%s: Starting peg homing sequence...", player_name_.c_str());

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<HomePeg::Feedback>();
    auto result = std::make_shared<HomePeg::Result>();

    // Use goal home position or default if not provided (0, 0, 0)
    geometry_msgs::msg::Point target_home = goal->home_position;
    if (target_home.x == 0.0 && target_home.y == 0.0 && target_home.z == 0.0)
    {
        target_home = default_home_;
        RCLCPP_INFO(this->get_logger(), "%s: Using default home position", player_name_.c_str());
    }

    try
    {
        // Wait for initial state
        RCLCPP_INFO(this->get_logger(), "%s: Waiting for initial board state...", player_name_.c_str());
        if (!wait_for_state(initial_state_timeout_))
        {
            result->success = false;
            result->message = "Failed to receive initial board state";
            goal_handle->abort(result);
            return;
        }

        geometry_msgs::msg::Point current_pos = get_current_peg_position();

        RCLCPP_INFO(this->get_logger(),
                    "%s: Initial position: [%.3f, %.3f]",
                    player_name_.c_str(), current_pos.x, current_pos.y);

        // First movement: validate synchronization
        RCLCPP_INFO(this->get_logger(), "%s: Performing synchronization check...", player_name_.c_str());
        current_pos = move_step_with_validation(current_pos, target_home);
        RCLCPP_INFO(this->get_logger(), "%s: Synchronization confirmed. Switching to fast feedback loop.", player_name_.c_str());

        // Continue homing with fast feedback loop
        float distance = calculate_distance(current_pos, target_home);

        while (distance > position_tolerance_ && rclcpp::ok())
        {
            // Check for cancellation
            if (goal_handle->is_canceling())
            {
                player_->send_commands(0.0f, 0.0f);
                result->success = false;
                result->message = "Homing canceled";
                result->final_position = current_pos;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "%s: Homing canceled", player_name_.c_str());
                return;
            }

            current_pos = move_step_feedback_loop(current_pos, target_home);
            distance = calculate_distance(current_pos, target_home);

            // Publish feedback
            feedback->current_position = current_pos;
            feedback->distance_remaining = distance;
            goal_handle->publish_feedback(feedback);

            RCLCPP_INFO(this->get_logger(), "%s: %.3f m from goal", player_name_.c_str(), distance);
        }

        player_->send_commands(0.0f, 0.0f);
        RCLCPP_INFO(this->get_logger(), "%s: Reached home position!", player_name_.c_str());

        result->success = true;
        result->message = "Homing completed successfully";
        result->final_position = current_pos;
        goal_handle->succeed(result);
    }
    catch (const std::exception &e)
    {
        player_->send_commands(0.0f, 0.0f);
        result->success = false;
        result->message = std::string("Homing failed: ") + e.what();
        result->final_position = get_current_peg_position();
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "%s: %s", player_name_.c_str(), result->message.c_str());
    }
}

void OpenLoopController::board_state_callback(const klask_interfaces::msg::State::SharedPtr msg)
{
    // State is already in engineering units (meters, m/s) from state estimator
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_state_ = msg;
    state_received_ = true;
}

bool OpenLoopController::wait_for_state(double timeout_sec)
{
    state_received_ = false;

    auto start_time = std::chrono::steady_clock::now();
    auto timeout_duration = std::chrono::duration<double>(timeout_sec);

    while (rclcpp::ok())
    {
        // Don't call spin_some - the node is already in an executor spinning in another thread
        // Just wait for the callback to set state_received_

        if (state_received_)
        {
            return true;
        }

        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed > timeout_duration)
        {
            RCLCPP_WARN(this->get_logger(), "Timeout waiting for board state");
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return false;
}

float OpenLoopController::calculate_distance(const geometry_msgs::msg::Point &p1,
                                             const geometry_msgs::msg::Point &p2) const
{
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}

geometry_msgs::msg::Point OpenLoopController::get_current_peg_position() const
{
    std::lock_guard<std::mutex> lock(const_cast<std::mutex &>(state_mutex_));
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

geometry_msgs::msg::Point OpenLoopController::move_step_with_validation(
    const geometry_msgs::msg::Point &current_pos,
    const geometry_msgs::msg::Point &goal_pos)
{
    // Calculate direction vector to goal
    float dx = goal_pos.x - current_pos.x;
    float dy = goal_pos.y - current_pos.y;
    float distance_to_goal = std::sqrt(dx * dx + dy * dy);

    if (distance_to_goal < position_tolerance_)
    {
        RCLCPP_INFO(this->get_logger(), "%s: Peg already at goal position", player_name_.c_str());
        return current_pos;
    }

    // Normalize direction
    float dir_x = dx / distance_to_goal;
    float dir_y = dy / distance_to_goal;

    // Calculate velocity commands
    float v_x = dir_x * homing_velocity_;
    float v_y = dir_y * homing_velocity_;

    RCLCPP_INFO(this->get_logger(),
                "%s: current=[%.3f, %.3f], goal=[%.3f, %.3f], distance=%.3f m",
                player_name_.c_str(), current_pos.x, current_pos.y,
                goal_pos.x, goal_pos.y, distance_to_goal);

    // Store initial position
    geometry_msgs::msg::Point initial_pos = current_pos;

    // Send movement command
    RCLCPP_INFO(this->get_logger(),
                "%s: Moving with velocity [%.3f, %.3f] m/s for %.1f seconds",
                player_name_.c_str(), v_x, v_y, validation_duration_);

    player_->send_commands(v_x, v_y);

    // Wait for validation duration
    std::this_thread::sleep_for(std::chrono::milliseconds(
        static_cast<int>(validation_duration_ * 1000)));

    // Stop movement
    player_->send_commands(0.0f, 0.0f);

    // Wait for updated state
    if (!wait_for_state(state_update_timeout_))
    {
        throw std::runtime_error(player_name_ + ": Failed to receive state after movement");
    }

    // Get updated position
    geometry_msgs::msg::Point new_pos = get_current_peg_position();

    // Calculate actual movement
    float actual_movement = calculate_distance(initial_pos, new_pos);
    float expected_movement = homing_velocity_ * validation_duration_;

    RCLCPP_INFO(this->get_logger(),
                "%s: Moved from [%.3f, %.3f] to [%.3f, %.3f], distance=%.3f m (expected ~%.3f m)",
                player_name_.c_str(), initial_pos.x, initial_pos.y,
                new_pos.x, new_pos.y, actual_movement, expected_movement);

    // Validate movement
    if (actual_movement < movement_threshold_)
    {
        throw std::runtime_error(
            player_name_ + ": Movement validation failed - peg did not move! " +
            "Expected ~" + std::to_string(expected_movement) + " m, got " +
            std::to_string(actual_movement) + " m. Peg may be desynchronized.");
    }

    if (actual_movement < expected_movement * expected_distance_factor_)
    {
        RCLCPP_WARN(this->get_logger(),
                    "%s: Movement less than expected (%.3f m vs %.3f m). Possible slippage.",
                    player_name_.c_str(), actual_movement, expected_movement);
    }

    return new_pos;
}

geometry_msgs::msg::Point OpenLoopController::move_step_feedback_loop(
    const geometry_msgs::msg::Point &current_pos,
    const geometry_msgs::msg::Point &goal_pos)
{
    // Calculate direction vector to goal
    float dx = goal_pos.x - current_pos.x;
    float dy = goal_pos.y - current_pos.y;
    float distance_to_goal = std::sqrt(dx * dx + dy * dy);

    if (distance_to_goal < position_tolerance_)
    {
        RCLCPP_INFO(this->get_logger(), "%s: Peg already at goal position", player_name_.c_str());
        return current_pos;
    }

    // Normalize direction
    float dir_x = dx / distance_to_goal;
    float dir_y = dy / distance_to_goal;

    // Use faster velocity for feedback loop
    const float feedback_velocity = homing_velocity_ * feedback_velocity_multiplier_;

    // Calculate velocity commands
    float v_x = dir_x * feedback_velocity;
    float v_y = dir_y * feedback_velocity;

    // Send movement command
    player_->send_commands(v_x, v_y);

    // Brief wait for movement
    std::this_thread::sleep_for(std::chrono::milliseconds(
        static_cast<int>(feedback_step_duration_ * 1000)));

    // Stop movement
    player_->send_commands(0.0f, 0.0f);

    // Wait for updated state
    if (!wait_for_state(feedback_state_timeout_))
    {
        throw std::runtime_error(player_name_ + ": Failed to receive state after movement");
    }

    // Get updated position
    geometry_msgs::msg::Point new_pos = get_current_peg_position();

    return new_pos;
}

}  // namespace klask_motor_commander
