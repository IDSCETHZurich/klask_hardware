/**
 * @file klask_motor_commander.cpp
 * @brief Main entry point for the motor commander node.
 * 
 * This file initializes the ODriveController node and executes it with a
 * multi-threaded executor to handle concurrent callbacks for player and
 * opponent motor control.
 */

#include "klask_motor_commander_pkg/odrive_controller.hpp"
#include <memory>

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
int main(int argc, char *argv[]) {
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
    
    RCLCPP_INFO(controller_node->get_logger(), "Motor commander system initialized. Spinning...");
    
    // Spin the executor (blocks until shutdown)
    executor.spin();
    
    RCLCPP_INFO(controller_node->get_logger(), "Shutting down motor commander system...");
    
    // Clean shutdown
    rclcpp::shutdown();
    return 0;
}