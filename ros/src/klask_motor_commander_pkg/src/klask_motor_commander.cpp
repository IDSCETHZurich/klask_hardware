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
#include <rclcpp_action/rclcpp_action.hpp>
#include <klask_interfaces/action/home_peg.hpp>
#include <klask_interfaces/msg/state.hpp>
#include <klask_interfaces/srv/calibrate_encoders.hpp>
#include <memory>
#include <atomic>

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

    // Create the main controller node
    auto controller_node = std::make_shared<ODriveController>();

    // Create multi-threaded executor for concurrent callback processing
    rclcpp::executors::MultiThreadedExecutor executor;

    // Add all nodes to the executor
    executor.add_node(controller_node);
    executor.add_node(controller_node->get_right_player_node());
    executor.add_node(controller_node->get_left_player_node());

    RCLCPP_INFO(controller_node->get_logger(), "Motor commander system initialized.");

    // Create open-loop controllers for homing (action servers)
    RCLCPP_INFO(controller_node->get_logger(), "Creating open-loop controllers for peg homing...");
    auto right_homing_controller = std::make_shared<OpenLoopController>(
        std::dynamic_pointer_cast<Player>(controller_node->get_right_player_node()));
    auto left_homing_controller = std::make_shared<OpenLoopController>(
        std::dynamic_pointer_cast<Player>(controller_node->get_left_player_node()));

    // Add homing controllers to executor
    executor.add_node(right_homing_controller);
    executor.add_node(left_homing_controller);

    // Spin executor in a separate thread to process callbacks
    std::thread spin_thread([&executor]()
                            { executor.spin(); });

    // Small delay to ensure action servers are ready
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Execute homing sequence using action clients
    RCLCPP_INFO(controller_node->get_logger(), "=== Starting Peg Homing Sequence ===");

    using HomePeg = klask_interfaces::action::HomePeg;

    // Create action clients
    auto right_homing_client = rclcpp_action::create_client<HomePeg>(
        controller_node, "home_peg_right_player");
    auto left_homing_client = rclcpp_action::create_client<HomePeg>(
        controller_node, "home_peg_left_player");

    // Wait for action servers
    if (!right_homing_client->wait_for_action_server(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(controller_node->get_logger(), "Right homing action server not available");
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }
    if (!left_homing_client->wait_for_action_server(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(controller_node->get_logger(), "Left homing action server not available");
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }

    geometry_msgs::msg::Point right_final_pos;
    geometry_msgs::msg::Point left_final_pos;
    bool homing_success = true;

    try
    {
        // // Home right player peg
        // RCLCPP_INFO(controller_node->get_logger(), "Sending homing goal for right player...");
        // auto right_goal = HomePeg::Goal();
        // right_goal.home_position.x = 0.0; // Use default
        // right_goal.home_position.y = 0.0;
        // right_goal.home_position.z = 0.0;

        // auto right_goal_handle_future = right_homing_client->async_send_goal(right_goal);

        // // Wait for goal to be accepted (executor is already spinning in separate thread)
        // if (right_goal_handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
        // {
        //     RCLCPP_ERROR(controller_node->get_logger(), "Failed to send right homing goal - timeout");
        //     homing_success = false;
        // }
        // else
        // {
        //     auto right_goal_handle = right_goal_handle_future.get();
        //     if (!right_goal_handle)
        //     {
        //         RCLCPP_ERROR(controller_node->get_logger(), "Right homing goal was rejected");
        //         homing_success = false;
        //     }
        //     else
        //     {
        //         // Wait for result
        //         auto right_result_future = right_homing_client->async_get_result(right_goal_handle);
        //         if (right_result_future.wait_for(std::chrono::seconds(30)) != std::future_status::ready)
        //         {
        //             RCLCPP_ERROR(controller_node->get_logger(), "Right homing action timed out");
        //             homing_success = false;
        //         }
        //         else
        //         {
        //             auto right_result = right_result_future.get();
        //             if (right_result.code == rclcpp_action::ResultCode::SUCCEEDED && right_result.result->success)
        //             {
        //                 right_final_pos = right_result.result->final_position;
        //                 RCLCPP_INFO(controller_node->get_logger(),
        //                             "Right player peg homed at [%.3f, %.3f]",
        //                             right_final_pos.x, right_final_pos.y);
        //             }
        //             else
        //             {
        //                 RCLCPP_ERROR(controller_node->get_logger(),
        //                              "Right homing failed: %s", right_result.result->message.c_str());
        //                 homing_success = false;
        //             }
        //         }
        //     }
        // }

        // if (!homing_success)
        // {
        //     throw std::runtime_error("Right player homing failed");
        // }

        // Small delay between homing operations
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Home left player peg
        RCLCPP_INFO(controller_node->get_logger(), "Sending homing goal for left player...");
        auto left_goal = HomePeg::Goal();
        left_goal.home_position.x = 0.0; // Use default
        left_goal.home_position.y = 0.0;
        left_goal.home_position.z = 0.0;

        auto left_goal_handle_future = left_homing_client->async_send_goal(left_goal);

        // Wait for goal to be accepted (executor is already spinning in separate thread)
        if (left_goal_handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
        {
            RCLCPP_ERROR(controller_node->get_logger(), "Failed to send left homing goal - timeout");
            homing_success = false;
        }
        else
        {
            auto left_goal_handle = left_goal_handle_future.get();
            if (!left_goal_handle)
            {
                RCLCPP_ERROR(controller_node->get_logger(), "Left homing goal was rejected");
                homing_success = false;
            }
            else
            {
                // Wait for result
                auto left_result_future = left_homing_client->async_get_result(left_goal_handle);
                if (left_result_future.wait_for(std::chrono::seconds(30)) != std::future_status::ready)
                {
                    RCLCPP_ERROR(controller_node->get_logger(), "Left homing action timed out");
                    homing_success = false;
                }
                else
                {
                    auto left_result = left_result_future.get();
                    if (left_result.code == rclcpp_action::ResultCode::SUCCEEDED && left_result.result->success)
                    {
                        left_final_pos = left_result.result->final_position;
                        RCLCPP_INFO(controller_node->get_logger(),
                                    "Left player peg homed at [%.3f, %.3f]",
                                    left_final_pos.x, left_final_pos.y);
                    }
                    else
                    {
                        RCLCPP_ERROR(controller_node->get_logger(),
                                     "Left homing failed: %s", left_result.result->message.c_str());
                        homing_success = false;
                    }
                }
            }
        }

        if (!homing_success)
        {
            throw std::runtime_error("Left player homing failed");
        }

        RCLCPP_INFO(controller_node->get_logger(), "=== Peg Homing Complete ===");
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(controller_node->get_logger(),
                     "Homing failed: %s. Shutting down...", e.what());

        // Cleanup and shutdown
        executor.remove_node(left_homing_controller);
        executor.remove_node(right_homing_controller);
        executor.remove_node(controller_node->get_right_player_node());
        executor.remove_node(controller_node->get_left_player_node());
        executor.remove_node(controller_node);

        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }

    // Remove homing controllers from executor (no longer needed)
    executor.remove_node(left_homing_controller);
    executor.remove_node(right_homing_controller);

    // Construct State message from homed positions
    klask_interfaces::msg::State calibration_state;
    calibration_state.right_peg.position = right_final_pos;
    calibration_state.left_peg.position = left_final_pos;

    // Call calibration with final homed state
    RCLCPP_INFO(controller_node->get_logger(), "Calling calibration service with homed state...");

    auto calibrate_client = controller_node->create_client<klask_interfaces::srv::CalibrateEncoders>(
        "calibrate_encoders");

    // Wait for service to be available
    if (!calibrate_client->wait_for_service(std::chrono::seconds(5)))
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
        if (result.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
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

    executor.remove_node(controller_node->get_right_player_node());
    executor.remove_node(controller_node->get_left_player_node());
    controller_node->remove_pointers();

    executor.remove_node(controller_node);

    // Clean shutdown
    rclcpp::shutdown();
    return 0;
}