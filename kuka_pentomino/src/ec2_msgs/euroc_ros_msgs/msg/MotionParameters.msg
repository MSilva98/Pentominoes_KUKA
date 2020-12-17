# The motion parameters message contains specifies general parameters for motions

# Indicates if the service call should block until the motion is finished (synchornous)
# or immediatly return after commanding the motion (asynchronous).
bool blocking

# Specifies the relative maximum velocity during the motion
# Range: 0.0 to 1.0, where 1.0 indicates the maximum possible velocity
float64 velocity

# With active blending the controller will not exactly converge 
# to via points on a path but optimize the trajectories smootheness / speed.
# The value indicates the distance to the via point in [m] at which the
# internal controller applies blending
float64 blending

# Default value for blending
float64 DEFAULT_BLENDING=0.0

# Default value for velocity
float64 DEFAULT_VELOCITY=0.1

# Default value for blocking
bool DEFAULT_BLOCKING=False

