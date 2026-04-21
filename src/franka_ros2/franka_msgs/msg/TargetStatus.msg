## The status of a motion action
# IDLE: The robot is idle, no motion is being executed
uint8 IDLE = 0

# EXECUTING: The robot is currently executing the motion
uint8 EXECUTING = 1

# TARGET_REACHED: The robot has reached the target position given two conditions:
# - the absolute difference between the current and goal joint positions is within the goal_tolerance
# - the joint velocities are below a small threshold
uint8 TARGET_REACHED = 2

# ABORTED: The motion was aborted due to an error or external intervention
uint8 ABORTED = 3

## The current status of the motion (one of the above states)
uint8 status
