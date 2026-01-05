#include "klask_motor_commander_pkg/open_loop_controller.hpp"
#include "klask_motor_commander_pkg/player.hpp"
#include <cmath>
#include <chrono>
#include <thread>

using namespace std::placeholders;

namespace klask_motor_commander
{

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

        // Control loop parameters
        this->declare_parameter("control_frequency", 10.0);

        // Timeout settings
        this->declare_parameter("initial_state_timeout", 5.0);

        // Load parameters
        homing_velocity_ = static_cast<float>(this->get_parameter("homing_velocity").as_double());
        position_tolerance_ = static_cast<float>(this->get_parameter("position_tolerance").as_double());
        control_frequency_ = this->get_parameter("control_frequency").as_double();
        initial_state_timeout_ = this->get_parameter("initial_state_timeout").as_double();

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

        RCLCPP_INFO(this->get_logger(), "Loaded parameters: homing_vel=%.3f, pos_tol=%.3f, control_freq=%.1f Hz",
                    homing_velocity_, position_tolerance_, control_frequency_);

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
        control_timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&OpenLoopController::control_loop_callback, this));

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
        RCLCPP_INFO(this->get_logger(), "%s: Starting peg homing sequence...", player_name_.c_str());

        const auto goal = goal_handle->get_goal();

        // Use goal home position or default if not provided (0, 0, 0)
        geometry_msgs::msg::Point target_home = goal->home_position;
        if (target_home.x == 0.0 && target_home.y == 0.0 && target_home.z == 0.0)
        {
            target_home = default_home_;
            RCLCPP_INFO(this->get_logger(), "%s: Using default home position", player_name_.c_str());
        }

        // Check if state is available
        if (!latest_state_)
        {
            RCLCPP_WARN(this->get_logger(), "%s: No board state available yet, will start when state arrives", player_name_.c_str());
        }
        else
        {
            geometry_msgs::msg::Point current_pos = get_current_peg_position();
            RCLCPP_INFO(this->get_logger(),
                        "%s: Initial position: [%.3f, %.3f], Target: [%.3f, %.3f]",
                        player_name_.c_str(), current_pos.x, current_pos.y,
                        target_home.x, target_home.y);
        }

        // Set goal and activate homing
        target_goal_ = target_home;
        active_goal_handle_ = goal_handle;

        RCLCPP_INFO(this->get_logger(), "%s: Homing activated, control loop will handle movement", player_name_.c_str());
    }

    void OpenLoopController::board_state_callback(const klask_interfaces::msg::State::SharedPtr msg)
    {
        // State is already in engineering units (meters, m/s) from state estimator
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_state_ = msg;
        state_received_ = true;
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

    void OpenLoopController::control_loop_callback()
    {
        // Check if we have an active goal
        if (!active_goal_handle_)
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "%s: No active goal in control loop", player_name_.c_str());
            return;
        }

        // Get current position
        geometry_msgs::msg::Point current_pos;
        try
        {
            current_pos = get_current_peg_position();
        }
        catch (const std::exception &e)
        {
            // No state available yet, skip this iteration
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "%s: No state available in control loop", player_name_.c_str());
            return;
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

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                             "%s: Moving toward goal, distance: %.3f m, velocity: [%.3f, %.3f] m/s",
                             player_name_.c_str(), distance, v_x, v_y);

        // Publish feedback
        auto feedback = std::make_shared<HomePeg::Feedback>();
        feedback->current_position = current_pos;
        feedback->distance_remaining = distance;
        active_goal_handle_->publish_feedback(feedback);
    }
} // namespace klask_motor_commander
