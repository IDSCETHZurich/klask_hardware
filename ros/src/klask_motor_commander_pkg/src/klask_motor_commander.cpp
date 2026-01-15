/**
 * @file klask_motor_commander.cpp
 * @brief Main entry point for the motor commander node.
 *
 * This file initializes the ODriveController node and executes it with a
 * multi-threaded executor to handle concurrent callbacks for player and
 * opponent motor control.
 */

#include "klask_motor_commander_pkg/odrive_controller.hpp"
#include "klask_motor_commander_pkg/open_loop_controller.hpp"
#include "klask_motor_commander_pkg/player_side.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <klask_interfaces/action/home_and_calibrate.hpp>
#include <klask_interfaces/action/home_peg.hpp>
#include <klask_interfaces/msg/state.hpp>
#include <klask_interfaces/srv/calibrate_encoders.hpp>
#include <memory>
#include <atomic>

using namespace klask_motor_commander;

/**
 * @brief Main function for klask_motor_commander_pkg.
 *
 * Initializes ROS2, creates the ODriveController node along with its
 * associated Player nodes, and spins them in a multi-threaded executor
 * for concurrent processing.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Exit status (0 for success).
 */
int main(int argc, char* argv[])
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create a temporary node to load the player and homing parameters
    auto temp_node = std::make_shared<rclcpp::Node>("temp_param_loader");
    temp_node->declare_parameter("player", "both");
    temp_node->declare_parameter("enable_homing", true);
    std::string player_str = temp_node->get_parameter("player").as_string();
    bool enable_homing = temp_node->get_parameter("enable_homing").as_bool();
    temp_node.reset();

    // Parse player configuration string to enum
    PlayerSide player_config;
    try
    {
        player_config = player_side_from_string(player_str);
    }
    catch (const std::invalid_argument& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("klask_motor_commander"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    // Create the main controller node with player configuration
    auto controller_node = std::make_shared<ODriveController>(player_config);

    // === Load parameters from controller node ===
    // Note: player parameter was already loaded and used to create controller_node
    controller_node->declare_parameter("right_player_action_name", "home_peg_right_player");
    controller_node->declare_parameter("left_player_action_name", "home_peg_left_player");
    controller_node->declare_parameter("action_server_wait_timeout", 5);
    controller_node->declare_parameter("homing_action_timeout", 30);
    controller_node->declare_parameter("calibration_service_timeout", 5);
    controller_node->declare_parameter("startup_delay", 1000);
    controller_node->declare_parameter("inter_homing_delay", 500);

    std::string right_action_name = controller_node->get_parameter("right_player_action_name").as_string();
    std::string left_action_name = controller_node->get_parameter("left_player_action_name").as_string();
    int action_wait_timeout = controller_node->get_parameter("action_server_wait_timeout").as_int();
    int homing_timeout = controller_node->get_parameter("homing_action_timeout").as_int();
    int calibration_timeout = controller_node->get_parameter("calibration_service_timeout").as_int();
    int startup_delay = controller_node->get_parameter("startup_delay").as_int();
    std::string calibrate_service = controller_node->get_parameter("calibrate_encoders_service").as_string();

    // Build list of active players from controller node
    const auto& all_players = controller_node->get_all_players();

    RCLCPP_INFO(controller_node->get_logger(),
                "Player configuration: %s (%zu player(s) active), Homing: %s",
                player_side_to_string(player_config).c_str(),
                all_players.size(),
                enable_homing ? "ENABLED" : "DISABLED");

    // Create multi-threaded executor for concurrent callback processing
    rclcpp::executors::MultiThreadedExecutor executor;

    // Add all nodes to the executor
    executor.add_node(controller_node);
    for (const auto& [player_name, player_node] : all_players)
    {
        executor.add_node(player_node);
    }

    RCLCPP_INFO(controller_node->get_logger(), "Motor commander system initialized.");

    // Spin executor in a separate thread to process callbacks
    std::thread spin_thread([&executor]() { executor.spin(); });

    // Delay to ensure action servers and services are ready
    std::this_thread::sleep_for(std::chrono::milliseconds(startup_delay));

    RCLCPP_INFO(controller_node->get_logger(), "System ready. Motors in CLOSED_LOOP_CONTROL.");

    // Execute homing sequence if enabled
    if (enable_homing)
    {
        // Call home_and_calibrate action for configured players
        RCLCPP_INFO(controller_node->get_logger(), "Calling home_and_calibrate action...");

        using HomeAndCalibrate = klask_interfaces::action::HomeAndCalibrate;

        // Determine which action(s) to call based on player configuration
        std::vector<std::string> action_names;
        if (has_player(player_config, PlayerSide::RIGHT_PLAYER))
        {
            action_names.push_back("home_and_calibrate_right_player");
        }
        if (has_player(player_config, PlayerSide::LEFT_PLAYER))
        {
            action_names.push_back("home_and_calibrate_left_player");
        }

        bool all_succeeded = true;
        for (const auto& action_name : action_names)
        {
            auto home_calibrate_client = rclcpp_action::create_client<HomeAndCalibrate>(controller_node, action_name);

            // Wait for action server to be available
            if (!home_calibrate_client->wait_for_action_server(std::chrono::seconds(5)))
            {
                RCLCPP_ERROR(controller_node->get_logger(), "%s action server not available", action_name.c_str());
                all_succeeded = false;
                continue;
            }

            // Send action goal
            auto goal = HomeAndCalibrate::Goal();

            auto send_goal_options = rclcpp_action::Client<HomeAndCalibrate>::SendGoalOptions();
            send_goal_options.feedback_callback =
                [&controller_node, action_name](rclcpp_action::ClientGoalHandle<HomeAndCalibrate>::SharedPtr,
                                                const std::shared_ptr<const HomeAndCalibrate::Feedback> feedback)
            {
                RCLCPP_INFO(controller_node->get_logger(),
                            "[%s] Homing progress: %s - %s",
                            action_name.c_str(),
                            feedback->current_phase.c_str(),
                            feedback->status_message.c_str());
            };

            auto goal_handle_future = home_calibrate_client->async_send_goal(goal, send_goal_options);

            // Wait for goal to be accepted
            if (goal_handle_future.wait_for(std::chrono::seconds(action_wait_timeout)) == std::future_status::ready)
            {
                auto goal_handle = goal_handle_future.get();
                if (!goal_handle)
                {
                    RCLCPP_ERROR(controller_node->get_logger(), "[%s] Action goal was rejected", action_name.c_str());
                    all_succeeded = false;
                }
                else
                {
                    // Wait for result with extended timeout (homing can take a while)
                    auto result_future = home_calibrate_client->async_get_result(goal_handle);
                    int total_timeout = homing_timeout * 2 + action_wait_timeout * 4 + calibration_timeout;

                    if (result_future.wait_for(std::chrono::seconds(total_timeout)) == std::future_status::ready)
                    {
                        auto result = result_future.get();
                        if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success)
                        {
                            RCLCPP_INFO(controller_node->get_logger(),
                                        "[%s] Successful: %s",
                                        action_name.c_str(),
                                        result.result->message.c_str());
                        }
                        else
                        {
                            RCLCPP_ERROR(controller_node->get_logger(),
                                         "[%s] Failed: %s",
                                         action_name.c_str(),
                                         result.result->message.c_str());
                            all_succeeded = false;
                        }
                    }
                    else
                    {
                        RCLCPP_ERROR(controller_node->get_logger(), "[%s] Action timed out", action_name.c_str());
                        all_succeeded = false;
                    }
                }
            }
            else
            {
                RCLCPP_ERROR(controller_node->get_logger(), "[%s] Action goal send timed out", action_name.c_str());
                all_succeeded = false;
            }
        }

        if (!all_succeeded)
        {
            RCLCPP_WARN(controller_node->get_logger(), "One or more home_and_calibrate actions failed");
        }
        else
        {
            RCLCPP_INFO(controller_node->get_logger(), "All home_and_calibrate actions completed successfully");
        }
    }
    else
    {
        RCLCPP_INFO(controller_node->get_logger(), "Homing disabled. Nodes ready for manual control (e.g., teleop).");
        RCLCPP_INFO(controller_node->get_logger(), "System will continue running. Press Ctrl+C to exit.");
    }

    // Wait for spin thread to complete
    spin_thread.join();

    RCLCPP_INFO(controller_node->get_logger(), "Shutting down motor commander system...");

    for (const auto& [player_name, player_node] : all_players)
    {
        executor.remove_node(player_node);
    }
    controller_node->remove_pointers();

    executor.remove_node(controller_node);

    // Clean shutdown
    rclcpp::shutdown();
    return 0;
}