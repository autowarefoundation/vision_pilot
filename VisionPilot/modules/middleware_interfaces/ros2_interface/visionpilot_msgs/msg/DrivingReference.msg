# One VisionPilot path and speed/stop intent from a single camera cycle.
#
# SI_CONTROL: the Safety Island selects this reference (or the Autoware
# trajectory) and its follower proposes the command. The path and the
# speed horizon belong to the same cycle; consumers must reject them if a
# matched pair from one cycle is not available.

# When VP produced this reference (VP host clock).
builtin_interfaces/Time stamp

# Capture time of the camera frame this cycle came from; see DrivingCommand.
builtin_interfaces/Time source_stamp
bool has_source_stamp

# VP process session and strictly increasing cycle counter.
uint32 session
uint64 cycle

# False when the upstream pipeline produced no usable reference this cycle.
bool valid

# Fused lane-center polynomial in base_link:
#   y = a x^2 + b x + c, x forward [m], y left [m].
# Consumers sample geometry from the polynomial instead of relying on a
# fixed point count. path_valid=false means the fit is unavailable and the
# reference must be rejected.
bool path_valid
float64 path_a
float64 path_b
float64 path_c
float64 path_x_max_m

# Planned speed schedule: v[i] m/s at t = i * horizon_dt_s.
float64 horizon_dt_s
float64[] speed_horizon_mps