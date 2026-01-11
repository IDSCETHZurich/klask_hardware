std_msgs/Header header

klask_interfaces/ObjectState ball
klask_interfaces/ObjectState left_peg
klask_interfaces/ObjectState right_peg

geometry_msgs/Point left_goal_pos
geometry_msgs/Point right_goal_pos

# Status field encoded as bit flags:
# UNKNOWN = 0
# READY = 1 << 0 (bit 0)
# BALL_IN_LEFT_GOAL = 1 << 1 (bit 1)
# BALL_IN_RIGHT_GOAL = 1 << 2 (bit 2)
# PEG_IN_LEFT_GOAL = 1 << 3 (bit 3)
# PEG_IN_RIGHT_GOAL = 1 << 4 (bit 4)
std_msgs/UInt64 status