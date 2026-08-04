# Absolute arm-joint targets. Zero degrees is each joint's calibrated center.
float64 joint1_deg
float64 joint2_deg
float64 joint3_deg

# Absolute finger opening: 0 cm is closed and max_opening_cm is fully open.
float64 gripper_opening_cm

# Time allowed for every motor to reach its target. Must be positive.
float64 duration_sec
