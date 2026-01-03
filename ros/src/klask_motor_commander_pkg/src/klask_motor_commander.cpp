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
int main(int argc, char *argv[])
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create a temporary node to load the player parameter
    auto temp_node = std::make_shared<rclcpp::Node>("temp_param_loader");
    temp_node->declare_parameter("player", "both");
    std::string player_str = temp_node->get_parameter("player").as_string();
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
    controller_node->declare_parameter("calibrate_encoders_service", "calibrate_encoders");

    std::string right_action_name = controller_node->get_parameter("right_player_action_name").as_string();
    std::string left_action_name = controller_node->get_parameter("left_player_action_name").as_string();
    int action_wait_timeout = controller_node->get_parameter("action_server_wait_timeout").as_int();
    int homing_timeout = controller_node->get_parameter("homing_action_timeout").as_int();
    int calibration_timeout = controller_node->get_parameter("calibration_service_timeout").as_int();
    int startup_delay = controller_node->get_parameter("startup_delay").as_int();
    int inter_homing_delay = controller_node->get_parameter("inter_homing_delay").as_int();
    std::string calibrate_service = controller_node->get_parameter("calibrate_encoders_service").as_string();

    // Type alias for HomePeg action
    using HomePeg = klask_interfaces::action::HomePeg;

    // Structure to hold player-specific data
    struct PlayerData
    {
        std::string name;
        std::string action_name;
        Player::SharedPtr player_node;
        std::shared_ptr<OpenLoopController> homing_controller;
        std::shared_ptr<rclcpp_action::Client<HomePeg>> homing_client;
        geometry_msgs::msg::Point final_position;
    };

    // Build list of active players based on configuration
    std::vector<PlayerData> active_players;
    
    // Build list of active players from controller node
    const auto& all_players = controller_node->get_all_players();
    for (const auto& [player_name, player_node] : all_players)
    {
        PlayerData player_data;
        player_data.name = player_name;
        player_data.action_name = (player_name == "right_player") ? right_action_name : left_action_name;
        player_data.player_node = player_node;
        player_data.homing_controller = nullptr;
        player_data.homing_client = nullptr;
        player_data.final_position = geometry_msgs::msg::Point();
        active_players.push_back(player_data);
    }

    RCLCPP_INFO(controller_node->get_logger(), 
                "Player configuration: %s (%zu player(s) active)",
                player_side_to_string(player_config).c_str(), active_players.size());

    // Create multi-threaded executor for concurrent callback processing
    rclcpp::executors::MultiThreadedExecutor executor;

    // Add all nodes to the executor
    executor.add_node(controller_node);
    for (const auto& player_data : active_players)
    {
        executor.add_node(player_data.player_node);
    }

    RCLCPP_INFO(controller_node->get_logger(), "Motor commander system initialized.");

    // Create open-loop controllers for homing (action servers) for active players
    RCLCPP_INFO(controller_node->get_logger(), "Creating open-loop controllers for peg homing...");
    for (auto& player_data : active_players)
    {
        player_data.homing_controller = std::make_shared<OpenLoopController>(
            std::dynamic_pointer_cast<Player>(player_data.player_node));
        executor.add_node(player_data.homing_controller);
        RCLCPP_INFO(controller_node->get_logger(), "%s homing controller created", player_data.name.c_str());
    }

    // Spin executor in a separate thread to process callbacks
    std::thread spin_thread([&executor]()
                            { executor.spin(); });

    // Delay to ensure action servers are ready
    std::this_thread::sleep_for(std::chrono::milliseconds(startup_delay));

    // Execute homing sequence using action clients
    RCLCPP_INFO(controller_node->get_logger(), "=== Starting Peg Homing Sequence ===");

    // Create action clients for active players and wait for servers
    for (auto& player_data : active_players)
    {
        player_data.homing_client = rclcpp_action::create_client<HomePeg>(
            controller_node, player_data.action_name);
        
        if (!player_data.homing_client->wait_for_action_server(std::chrono::seconds(action_wait_timeout)))
        {
            RCLCPP_ERROR(controller_node->get_logger(), "%s homing action server not available", 
                        player_data.name.c_str());
            rclcpp::shutdown();
            spin_thread.join();
            return 1;
        }
    }

    bool homing_success = true;

    try
    {
        // Home all active players
        for (size_t i = 0; i < active_players.size(); i++)
        {
            auto& player_data = active_players[i];
            
            RCLCPP_INFO(controller_node->get_logger(), "Sending homing goal for %s...", player_data.name.c_str());
            
            auto goal = HomePeg::Goal();
            goal.home_position.x = 0.0; // Use default
            goal.home_position.y = 0.0;
            goal.home_position.z = 0.0;

            auto goal_handle_future = player_data.homing_client->async_send_goal(goal);

            // Wait for goal to be accepted (executor is already spinning in separate thread)
            if (goal_handle_future.wait_for(std::chrono::seconds(action_wait_timeout)) != std::future_status::ready)
            {
                RCLCPP_ERROR(controller_node->get_logger(), "Failed to send %s homing goal - timeout", 
                            player_data.name.c_str());
                homing_success = false;
            }
            else
            {
                auto goal_handle = goal_handle_future.get();
                if (!goal_handle)
                {
                    RCLCPP_ERROR(controller_node->get_logger(), "%s homing goal was rejected", 
                                player_data.name.c_str());
                    homing_success = false;
                }
                else
                {
                    // Wait for result
                    auto result_future = player_data.homing_client->async_get_result(goal_handle);
                    if (result_future.wait_for(std::chrono::seconds(homing_timeout)) != std::future_status::ready)
                    {
                        RCLCPP_ERROR(controller_node->get_logger(), "%s homing action timed out", 
                                    player_data.name.c_str());
                        homing_success = false;
                    }
                    else
                    {
                        auto result = result_future.get();
                        if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success)
                        {
                            player_data.final_position = result.result->final_position;
                            RCLCPP_INFO(controller_node->get_logger(),
                                        "%s peg homed at [%.3f, %.3f]",
                                        player_data.name.c_str(),
                                        player_data.final_position.x, 
                                        player_data.final_position.y);
                        }
                        else
                        {
                            RCLCPP_ERROR(controller_node->get_logger(),
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

            std::this_thread::sleep_for(std::chrono::milliseconds(inter_homing_delay));
        }

        RCLCPP_INFO(controller_node->get_logger(), "=== Peg Homing Complete ===");
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(controller_node->get_logger(),
                     "Homing failed: %s. Shutting down...", e.what());

        // Cleanup and shutdown
        for (auto& player_data : active_players)
        {
            if (player_data.homing_controller)
            {
                executor.remove_node(player_data.homing_controller);
            }
        }
        for (const auto& player_data : active_players)
        {
            executor.remove_node(player_data.player_node);
        }
        executor.remove_node(controller_node);

        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }

    // Remove homing controllers from executor (no longer needed)
    for (auto& player_data : active_players)
    {
        if (player_data.homing_controller)
        {
            executor.remove_node(player_data.homing_controller);
            player_data.homing_controller = nullptr;
        }
    }

    // Construct State message from homed positions (only for active players)
    klask_interfaces::msg::State calibration_state;
    for (const auto& player_data : active_players)
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
    RCLCPP_INFO(controller_node->get_logger(), "Calling calibration service with homed state...");

    auto calibrate_client = controller_node->create_client<klask_interfaces::srv::CalibrateEncoders>(
        calibrate_service);

    // Wait for service to be available
    if (!calibrate_client->wait_for_service(std::chrono::seconds(calibration_timeout)))
    {
        RCLCPP_ERROR(controller_node->get_logger(), "Calibration service not available");
    }
    else
    {
        // Call calibration service with homed state
        auto request = std::make_shared<klask_interfaces::srv::CalibrateEncoders::Request>();
        request->state = calibration_state;

        auto result = calibrate_client->async_send_request(request);

        // Wait for result with timeout (executor is already spinning)
        if (result.wait_for(std::chrono::seconds(calibration_timeout)) == std::future_status::ready)
        {
            auto response = result.get();
            if (response->success)
            {
                RCLCPP_INFO(controller_node->get_logger(), "Calibration successful: %s",
                            response->message.c_str());
            }
            else
            {
                RCLCPP_ERROR(controller_node->get_logger(), "Calibration failed: %s",
                             response->message.c_str());
            }
        }
        else
        {
            RCLCPP_ERROR(controller_node->get_logger(), "Calibration service call timed out");
        }
    }

    RCLCPP_INFO(controller_node->get_logger(), "System ready. Spinning main executor...");

    // Wait for spin thread to complete
    spin_thread.join();

    RCLCPP_INFO(controller_node->get_logger(), "Shutting down motor commander system...");

    for (const auto& player_data : active_players)
    {
        executor.remove_node(player_data.player_node);
    }
    controller_node->remove_pointers();

    executor.remove_node(controller_node);

    // Clean shutdown
    rclcpp::shutdown();
    return 0;
}