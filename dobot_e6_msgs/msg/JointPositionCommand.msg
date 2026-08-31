# ============================================================
# JointPositionCommand
# ============================================================
#
# Joint-space position command for the Dobot Magician E6.
#
# The desired joint positions follow the ROS convention and
# are expressed in radians.
#
# velocity_ratio and acceleration_ratio correspond to the
# percentage parameters used internally by the Dobot MovJ
# command.
#
# ============================================================


# Desired joint positions [rad]
#
# Order:
#   position[0] -> Joint 1
#   position[1] -> Joint 2
#   position[2] -> Joint 3
#   position[3] -> Joint 4
#   position[4] -> Joint 5
#   position[5] -> Joint 6
float64[6] position


# Joint motion velocity ratio [%]
#
# Expected range:
#   1.0 - 100.0
#
# This is NOT joint velocity in rad/s.
float64 velocity_ratio


# Joint motion acceleration ratio [%]
#
# Expected range:
#   1.0 - 100.0
#
# This is NOT joint acceleration in rad/s^2.
float64 acceleration_ratio
