#ifndef KLASK_MOTOR_COMMANDER_PKG__PLAYER_SIDE_HPP_
#define KLASK_MOTOR_COMMANDER_PKG__PLAYER_SIDE_HPP_

#include <string>
#include <stdexcept>

namespace klask_motor_commander {

/**
 * @brief Enum to identify which side(s) of the table are active.
 * 
 * Uses bitwise flags to allow combining multiple players.
 */
enum class PlayerSide : int
{
    NONE = 0,                                    ///< No players active
    RIGHT_PLAYER = 1 << 0,                       ///< Right side player (motors 0 and 1)
    LEFT_PLAYER = 1 << 1,                        ///< Left side player (motors 2 and 3)
    BOTH_PLAYERS = RIGHT_PLAYER | LEFT_PLAYER    ///< Both players active
};

/**
 * @brief Bitwise OR operator for PlayerSide enum.
 */
inline PlayerSide operator|(PlayerSide lhs, PlayerSide rhs)
{
    return static_cast<PlayerSide>(static_cast<int>(lhs) | static_cast<int>(rhs));
}

/**
 * @brief Bitwise AND operator for PlayerSide enum.
 */
inline PlayerSide operator&(PlayerSide lhs, PlayerSide rhs)
{
    return static_cast<PlayerSide>(static_cast<int>(lhs) & static_cast<int>(rhs));
}

/**
 * @brief Bitwise OR assignment operator for PlayerSide enum.
 */
inline PlayerSide& operator|=(PlayerSide& lhs, PlayerSide rhs)
{
    lhs = lhs | rhs;
    return lhs;
}

/**
 * @brief Check if a specific player side is active in a PlayerSide configuration.
 * 
 * @param config The PlayerSide configuration (possibly BOTH_PLAYERS)
 * @param side The specific side to check for
 * @return true if the side is active in the configuration
 */
inline bool has_player(PlayerSide config, PlayerSide side)
{
    return (config & side) == side;
}

/**
 * @brief Convert PlayerSide enum to string name for logging.
 * 
 * @param side The PlayerSide value
 * @return String representation ("right", "left", "both", "none")
 */
inline std::string player_side_to_string(PlayerSide side)
{
    switch (side)
    {
        case PlayerSide::RIGHT_PLAYER:
            return "right";
        case PlayerSide::LEFT_PLAYER:
            return "left";
        case PlayerSide::BOTH_PLAYERS:
            return "both";
        case PlayerSide::NONE:
            return "none";
        default:
            return "unknown";
    }
}

/**
 * @brief Parse string to PlayerSide enum.
 * 
 * @param str String representation ("right", "left", "both")
 * @return PlayerSide enum value
 * @throws std::invalid_argument if string is not recognized
 */
inline PlayerSide player_side_from_string(const std::string& str)
{
    if (str == "right")
        return PlayerSide::RIGHT_PLAYER;
    else if (str == "left")
        return PlayerSide::LEFT_PLAYER;
    else if (str == "both")
        return PlayerSide::BOTH_PLAYERS;
    else
        throw std::invalid_argument("Invalid player configuration: '" + str + 
                                   "'. Must be 'right', 'left', or 'both'");
}

/**
 * @brief Get the string name for a specific player side (not configuration).
 * 
 * @param side The specific player side (RIGHT_PLAYER or LEFT_PLAYER only)
 * @return String name ("right_player" or "left_player")
 */
inline std::string get_player_name(PlayerSide side)
{
    switch (side)
    {
        case PlayerSide::RIGHT_PLAYER:
            return "right_player";
        case PlayerSide::LEFT_PLAYER:
            return "left_player";
        default:
            return "unknown_player";
    }
}

}  // namespace klask_motor_commander

#endif // KLASK_MOTOR_COMMANDER_PKG__PLAYER_SIDE_HPP_
