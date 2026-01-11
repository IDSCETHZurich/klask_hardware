#ifndef KLASK_MOTOR_COMMANDER_PKG__OPEN_LOOP_CONTROLLER_HPP_
#define KLASK_MOTOR_COMMANDER_PKG__OPEN_LOOP_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <klask_interfaces/action/home_peg.hpp>
#include <klask_interfaces/msg/state.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <atomic>
#include <mutex>

namespace klask_motor_commander
{

    // Forward declaration
    class Player;

    /**
     * @brief Open-loop controller for homing a single peg to target position.
     *
     * This controller drives a peg to home position using dead-reckoning and
     * validates movement by checking actual position changes from the state estimator.
     * If the peg doesn't move as expected, an error is thrown indicating potential
     * desynchronization between the magnetic peg and physical peg.
     *
     * Implements a ROS2 action server for the HomePeg action.
     */
    class OpenLoopController : public rclcpp::Node
    {
    public:
        using HomePeg = klask_interfaces::action::HomePeg;
        using GoalHandleHomePeg = rclcpp_action::ServerGoalHandle<HomePeg>;

        /**
         * @brief Construct a new Open Loop Controller object.
         *
         * @param player Shared pointer to player instance
         */
        explicit OpenLoopController(std::shared_ptr<Player> player);

    private:
        /// Shared pointer to player instance
        std::shared_ptr<Player> player_;

        /// Player name ("right_player" or "left_player")
        std::string player_name_;

        /// Action server for homing
        rclcpp_action::Server<HomePeg>::SharedPtr action_server_;

        /// Subscription to board state topic
        rclcpp::Subscription<klask_interfaces::msg::State>::SharedPtr board_state_sub_;

        /// Latest received board state
        klask_interfaces::msg::State::SharedPtr latest_state_;

        /// Mutex for thread-safe state access
        std::mutex state_mutex_;

        /// Flag indicating new state received
        std::atomic<bool> state_received_{false};

        /// Default home position
        geometry_msgs::msg::Point default_home_;

        // === Parameters (loaded from ROS parameters) ===

        /// Homing motion parameters
        float homing_velocity_;
        float position_tolerance_;

        /// Movement validation parameters
        float validation_duration_;
        float movement_threshold_;
        float expected_distance_factor_;

        /// Control loop timer
        rclcpp::TimerBase::SharedPtr control_timer_;
        double control_frequency_;

        /// Timeout values
        double initial_state_timeout_;

        /// Movement validation
        rclcpp::TimerBase::SharedPtr validation_timer_;
        geometry_msgs::msg::Point validation_start_position_;
        geometry_msgs::msg::Point validation_expected_position_;
        bool validation_active_;

        /// Goal state
        geometry_msgs::msg::Point target_goal_;
        std::shared_ptr<GoalHandleHomePeg> active_goal_handle_;

        /// State waiting tracking
        rclcpp::Time goal_accepted_time_;
        std::atomic<bool> waiting_for_initial_state_;

        /**
         * @brief Handle new goal request.
         */
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const HomePeg::Goal> goal);

        /**
         * @brief Handle cancel request.
         */
        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleHomePeg> goal_handle);

        /**
         * @brief Handle accepted goal - sets goal and activates homing.
         */
        void handle_accepted(const std::shared_ptr<GoalHandleHomePeg> goal_handle);

        /**
         * @brief Callback for board state updates.
         *
         * @param msg Shared pointer to received state message
         */
        void board_state_callback(const klask_interfaces::msg::State::SharedPtr msg);

        /**
         * @brief Wait for a new board state message.
         *
         * @param timeout_sec Maximum time to wait in seconds
         * @return true if new state received, false if timeout
         */
        bool wait_for_state(double timeout_sec = 2.0);

        /**
         * @brief Calculate Euclidean distance between two points.
         *
         * @param p1 First point
         * @param p2 Second point
         * @return float Distance in meters
         */
        float calculate_distance(const geometry_msgs::msg::Point &p1,
                                 const geometry_msgs::msg::Point &p2) const;

        /**
         * @brief Get current peg position from latest state.
         *
         * @return geometry_msgs::msg::Point Current peg position
         */
        geometry_msgs::msg::Point get_current_peg_position() const;

        /**
         * @brief Control loop callback - sends velocity commands toward goal.
         */
        void control_loop_callback();

        /**
         * @brief Validation callback - checks if peg moved as expected.
         */
        void movement_validation_callback();

        /**
         * @brief Start movement validation timer.
         *
         * @param start_pos Current position when validation starts
         */
        void start_movement_validation(const geometry_msgs::msg::Point& start_pos);
    };

} // namespace klask_motor_commander

#endif // KLASK_MOTOR_COMMANDER_PKG__OPEN_LOOP_CONTROLLER_HPP_
