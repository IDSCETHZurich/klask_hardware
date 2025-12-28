#include "klask_motor_commander_pkg/open_loop_controller.hpp"
#include "klask_motor_commander_pkg/player.hpp"
#include <cmath>
#include <chrono>
#include <thread>

using namespace std::placeholders;

OpenLoopController::OpenLoopController(std::shared_ptr<Player> player)
    : Node("open_loop_controller_" + std::string(player->get_side() == PlayerSide::RIGHT_PLAYER ? "right_player" : "left_player")),
      player_(player),
      player_name_(player->get_side() == PlayerSide::RIGHT_PLAYER ? "right_player" : "left_player")
{
    RCLCPP_INFO(this->get_logger(), "Initializing OpenLoopController for %s...", player_name_.c_str());

    // Set default home position based on player
    if (player_->get_side() == PlayerSide::RIGHT_PLAYER)
    {
        default_home_.x = 0.26; // Right player field center
        default_home_.y = 0.16;
    }
    else
    {
        default_home_.x = 0.16; // Left player field center
        default_home_.y = 0.16;
    }
    default_home_.z = 0.0;

    // Subscribe to board state
    board_state_sub_ = this->create_subscription<klask_interfaces::msg::State>(
        "/board_state",
        rclcpp::QoS(10).reliable(),
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
        if (!wait_for_state(5.0))
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

        while (distance > POSITION_TOLERANCE && rclcpp::ok())
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
    // TODO: Remove this onece estimator is correct
    constexpr float PIXEL_TO_EU_X = 0.42f / 656.0f; // Meters per pixel in x
    constexpr float PIXEL_TO_EU_Y = 0.32f / 497.0f; // Meters per pixel in y

    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_state_ = msg;
    // Convert peg positions from pixels to meters
    latest_state_->left_peg.position.x *= PIXEL_TO_EU_X;
    latest_state_->left_peg.position.y *= PIXEL_TO_EU_Y;
    latest_state_->right_peg.position.x *= PIXEL_TO_EU_X;
    latest_state_->right_peg.position.y *= PIXEL_TO_EU_Y;
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

    if (distance_to_goal < POSITION_TOLERANCE)
    {
        RCLCPP_INFO(this->get_logger(), "%s: Peg already at goal position", player_name_.c_str());
        return current_pos;
    }

    // Normalize direction
    float dir_x = dx / distance_to_goal;
    float dir_y = dy / distance_to_goal;

    // Calculate velocity commands
    float v_x = dir_x * HOMING_VELOCITY;
    float v_y = dir_y * HOMING_VELOCITY;

    RCLCPP_INFO(this->get_logger(),
                "%s: current=[%.3f, %.3f], goal=[%.3f, %.3f], distance=%.3f m",
                player_name_.c_str(), current_pos.x, current_pos.y,
                goal_pos.x, goal_pos.y, distance_to_goal);

    // Store initial position
    geometry_msgs::msg::Point initial_pos = current_pos;

    // Send movement command
    RCLCPP_INFO(this->get_logger(),
                "%s: Moving with velocity [%.3f, %.3f] m/s for %.1f seconds",
                player_name_.c_str(), v_x, v_y, VALIDATION_DURATION);

    player_->send_commands(v_x, v_y);

    // Wait for validation duration
    std::this_thread::sleep_for(std::chrono::milliseconds(
        static_cast<int>(VALIDATION_DURATION * 1000)));

    // Stop movement
    player_->send_commands(0.0f, 0.0f);

    // Wait for updated state
    if (!wait_for_state(2.0))
    {
        throw std::runtime_error(player_name_ + ": Failed to receive state after movement");
    }

    // Get updated position
    geometry_msgs::msg::Point new_pos = get_current_peg_position();

    // Calculate actual movement
    float actual_movement = calculate_distance(initial_pos, new_pos);
    float expected_movement = HOMING_VELOCITY * VALIDATION_DURATION;

    RCLCPP_INFO(this->get_logger(),
                "%s: Moved from [%.3f, %.3f] to [%.3f, %.3f], distance=%.3f m (expected ~%.3f m)",
                player_name_.c_str(), initial_pos.x, initial_pos.y,
                new_pos.x, new_pos.y, actual_movement, expected_movement);

    // Validate movement
    if (actual_movement < MOVEMENT_THRESHOLD)
    {
        throw std::runtime_error(
            player_name_ + ": Movement validation failed - peg did not move! " +
            "Expected ~" + std::to_string(expected_movement) + " m, got " +
            std::to_string(actual_movement) + " m. Peg may be desynchronized.");
    }

    if (actual_movement < expected_movement * EXPECTED_DISTANCE_FACTOR)
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

    if (distance_to_goal < POSITION_TOLERANCE)
    {
        RCLCPP_INFO(this->get_logger(), "%s: Peg already at goal position", player_name_.c_str());
        return current_pos;
    }

    // Normalize direction
    float dir_x = dx / distance_to_goal;
    float dir_y = dy / distance_to_goal;

    // Use faster velocity for feedback loop (2x homing velocity)
    constexpr float FEEDBACK_VELOCITY = HOMING_VELOCITY * 2.0f;
    constexpr float FEEDBACK_STEP_DURATION = 0.1f; // 100ms steps

    // Calculate velocity commands
    float v_x = dir_x * FEEDBACK_VELOCITY;
    float v_y = dir_y * FEEDBACK_VELOCITY;

    // Send movement command
    player_->send_commands(v_x, v_y);

    // Brief wait for movement
    std::this_thread::sleep_for(std::chrono::milliseconds(
        static_cast<int>(FEEDBACK_STEP_DURATION * 1000)));

    // Stop movement
    player_->send_commands(0.0f, 0.0f);

    // Wait for updated state
    if (!wait_for_state(0.5))
    {
        throw std::runtime_error(player_name_ + ": Failed to receive state after movement");
    }

    // Get updated position
    geometry_msgs::msg::Point new_pos = get_current_peg_position();

    return new_pos;
}
