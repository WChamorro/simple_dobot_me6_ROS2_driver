# Realtime joint telemetry for the Dobot Magician E6.
#
# Values are reported for joints 1-6 in the same order as `name`.
# Exact physical units should be verified against the DOBOT SDK/manual
# before using these fields for quantitative analysis or control.

std_msgs/Header header

string[6] name

float64[6] current
float64[6] torque
float64[6] temperature
float64[6] voltage
