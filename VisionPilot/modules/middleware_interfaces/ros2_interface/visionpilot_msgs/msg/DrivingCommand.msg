# One VisionPilot driving command produced from a single camera cycle.
#
# VP_CONTROL: the Safety Island checks this request and may replace it with
# its own stop/hold response, but must not recompute VP's normal decision.

# When VP produced this command (VP host clock).
builtin_interfaces/Time stamp

# Capture time of the camera frame this cycle's decision came from.
# Carried through from the image header so consumers can match the same
# camera cycle instead of using arrival/publication time.
builtin_interfaces/Time source_stamp
bool has_source_stamp

# VP process session (changes on restart) and strictly increasing cycle
# counter for duplicate/order detection.
uint32 session
uint64 cycle

# False when the upstream pipeline produced no usable plan this cycle.
bool valid

# Steering tyre angle (rad, positive left) and longitudinal intent:
# signed acceleration plus the speed VP intends after the planning horizon.
float64 steering_tire_angle_rad
float64 target_speed_mps
float64 acceleration_mps2