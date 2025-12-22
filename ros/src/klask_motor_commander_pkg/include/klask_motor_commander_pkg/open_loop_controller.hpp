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

    // Movement validation parameters
    static constexpr float HOMING_VELOCITY = 0.01f;         ///< Slow velocity for homing (m/s)
    static constexpr float VALIDATION_DURATION = 2.0f;      ///< Duration for movement validation (s)
    static constexpr float POSITION_TOLERANCE = 0.005f;     ///< Tolerance for goal reaching (m)
    static constexpr float MOVEMENT_THRESHOLD = 0.01f;      ///< Minimum expected movement (m)
    static constexpr float EXPECTED_DISTANCE_FACTOR = 0.5f; ///< Factor of expected vs actual movement

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
     * @brief Handle accepted goal and execute homing.
     */
    void handle_accepted(const std::shared_ptr<GoalHandleHomePeg> goal_handle);

    /**
     * @brief Execute the homing action.
     */
    void execute(const std::shared_ptr<GoalHandleHomePeg> goal_handle);

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
     * @brief Move peg one step toward goal with movement validation.
     *
     * @param current_pos Current peg position
     * @param goal_pos Target goal position
     * @return geometry_msgs::msg::Point Updated position after movement
     * @throws std::runtime_error If movement validation fails
     */
    geometry_msgs::msg::Point move_step_with_validation(
        const geometry_msgs::msg::Point &current_pos,
        const geometry_msgs::msg::Point &goal_pos);

    /**
     * @brief Move peg one step toward goal using fast feedback loop.
     *
     * @param current_pos Current peg position
     * @param goal_pos Target goal position
     * @return geometry_msgs::msg::Point Updated position after movement
     * @throws std::runtime_error If state update fails
     */
    geometry_msgs::msg::Point move_step_feedback_loop(
        const geometry_msgs::msg::Point &current_pos,
        const geometry_msgs::msg::Point &goal_pos);
};

#endif // KLASK_MOTOR_COMMANDER_PKG__OPEN_LOOP_CONTROLLER_HPP_
