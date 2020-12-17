# Specifies the state of the robot joints 

# The time stamp at which the measurement was taken (only qualitative but monotonically increasing)
time stamp

# The joint positions [rad]
float64[] positions

# The commanded joint positions of the controller [rad]
float64[] commanded_positions

# The measured torques applied to the joints [Nm]
float64[] measured_torques

# The estimated external torques applied to the joints [Nm]
float64[] external_torques