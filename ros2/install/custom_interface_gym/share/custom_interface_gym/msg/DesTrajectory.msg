# Trajectory.msg

std_msgs/Header header

# Trajectory ID, starts from "1".
uint32 trajectory_id

# Action command for the trajectory server.
uint32 ACTION_ADD                  = 1  # Add a new trajectory
uint32 ACTION_ABORT                = 2  # Abort the current trajectory
uint32 ACTION_WARN_START           = 3  # Warning: trajectory start issue
uint32 ACTION_WARN_FINAL           = 4  # Warning: trajectory final issue
uint32 ACTION_WARN_IMPOSSIBLE      = 5  # Warning: impossible trajectory
uint32 action                       # Action type

# Trajectory order and segments.
uint32 num_order                    # Polynomial order of the trajectory (D)
uint32 num_segment                  # Number of trajectory segments

# Array of flattened matrices (as a single sequence of float64)
float64[] matrices_flat             # Flattened sequence of matrix values

# Duration for each segment
float64[] duration_vector                      # Duration for each segment

# Debugging information
string debug_info                   # Optional debug info (e.g., for warnings or issues)
