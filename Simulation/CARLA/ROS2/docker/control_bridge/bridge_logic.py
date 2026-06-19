"""Pure ackermann -> vehicle-control mapping (no ROS2 / CARLA deps).

Kept separate from control_bridge.py (the ROS2 node) so it is unit-testable anywhere
with plain pytest — see test_control_bridge.py.
"""
from dataclasses import dataclass


@dataclass
class BridgeParams:
    """Vehicle/gain parameters for the ackermann->control mapping."""

    max_steer: float = 1.22       # front-wheel steer limit [rad]; lincoln.mkz ~70deg
    kp: float = 3.0               # throttle/brake P-gain; high enough to break static friction from rest
    throttle_max: float = 1.0
    brake_deadband: float = 0.5   # [m/s] speed-error band with no throttle/brake


def ackermann_to_control(steer_rad, target_v, cur_v, p: BridgeParams):
    """(steer angle, target speed, current speed) -> (throttle, steer, brake).

      steer  = clamp(steer_rad / max_steer, [-1, 1])
      err    = target_v - cur_v
      err > 0            -> throttle = clamp(kp*err, [0, throttle_max]), brake = 0
      err < -deadband    -> brake = clamp(-kp*err, [0, 1]), throttle = 0
      otherwise          -> coast (throttle = brake = 0)
    """
    steer = max(-1.0, min(1.0, steer_rad / p.max_steer))
    err = target_v - cur_v
    if err > 0.0:
        throttle = max(0.0, min(p.throttle_max, p.kp * err))
        brake = 0.0
    elif err < -p.brake_deadband:
        throttle = 0.0
        brake = max(0.0, min(1.0, -p.kp * err))
    else:
        throttle = 0.0
        brake = 0.0
    return throttle, steer, brake
