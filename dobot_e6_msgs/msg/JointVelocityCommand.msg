# ============================================================
# JointVelocityCommand
# ============================================================
#
# Joint-space velocity command for the Dobot Magician E6.
#
# All joint velocities follow the ROS convention and are
# expressed in radians per second [rad/s].
#
# Positive and negative values define the direction of motion.
#
# Order:
#   velocity[0] -> Joint 1
#   velocity[1] -> Joint 2
#   velocity[2] -> Joint 3
#   velocity[3] -> Joint 4
#   velocity[4] -> Joint 5
#   velocity[5] -> Joint 6
#
# A value of 0.0 means that the corresponding joint should
# remain stationary.
#
# ============================================================

# Desired joint velocities [rad/s]
float64[6] velocity
