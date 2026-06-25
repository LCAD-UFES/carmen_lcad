# Mirrors carmen_localize_ackerman_globalpos_message
# Published by the bridge when slam_toolbox produces a new pose estimate.

std_msgs/Header header

# 2D pose (x, y, theta) — same coordinate frame as CARMEN globalpos
float64 x
float64 y
float64 theta

# Full 6-DOF pose for 3D consumers
geometry_msgs/PoseWithCovariance pose

# Velocity information
float64 v          # linear  [m/s]
float64 phi        # steering angle [rad]

# Convergence flag (mirrors filter->converged)
bool converged

float64 timestamp
