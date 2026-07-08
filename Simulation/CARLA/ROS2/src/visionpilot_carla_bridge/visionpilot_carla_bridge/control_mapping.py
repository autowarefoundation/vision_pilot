"""Pure VisionPilot-actuation -> CARLA-vehicle-control mapping (no ROS2 / CARLA deps).

Kept separate from the ROS2 node so it is unit-testable anywhere with plain pytest.

The bridge maps VisionPilot's steer prediction (tyre angle, rad) to a normalized CARLA
steer via `steer_to_carla`, and translates VisionPilot's longitudinal accel command to
throttle/brake with a FEED-FORWARD `accel_to_throttle_brake` map. The bridge runs NO speed
controller of its own: VisionPilot already closes the speed loop (its longitudinal planner
gets the ego speed on /vehicle/speed + the configured speed_limit), so a bridge-side PID
would only override that autodrive behaviour. All outputs are normalized: steer in [-1, 1],
throttle/brake in [0, 1].

NOTE: an EMA/slew steering filter used to live here. It was removed on purpose — its phase
lag was measured to CAUSE the low-speed lateral limit cycle (rail-fraction 0.73 -> 0.12 when
cut). Do not reintroduce output smoothing without re-measuring the loop delay.
"""

from dataclasses import dataclass


@dataclass
class ControlParams:
    """Steering parameters for the actuation->control mapping."""

    max_steer: float = 1.2217  # front-wheel steer limit [rad]; fallback if CARLA doesn't report
    steer_gain: float = 1.0  # proportional gain on the VisionPilot steer prediction


def steer_to_carla(steer_rad: float, max_steer: float, steer_gain: float = 1.0) -> float:
    """Tyre angle [rad] -> normalized steer [-1, 1], direct proportional with steer_gain."""
    return max(-1.0, min(1.0, steer_gain * steer_rad / max_steer))


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


@dataclass
class AccelMap:
    """Feed-forward parameters: desired longitudinal accel [m/s^2] -> throttle/brake pedal.

    The bridge does NOT close the speed loop — VisionPilot does. VP's longitudinal planner
    already takes the ego speed + the configured speed_limit and emits the acceleration it
    wants; the bridge only translates that accel to a pedal position via a simple linear
    vehicle model. Any model error (drag, grade, the nonlinear pedal map) is trimmed by VP's
    own loop on the next frame. This is why there is no PID, target_speed, or integral here.

    throttle_ref_accel : accel that maps to throttle = 1.0 (smaller -> more eager throttle)
    brake_ref_accel    : decel that maps to brake = 1.0 (smaller -> stronger braking)
    accel_deadband     : |accel| below this -> coast (no throttle, no brake)
    """

    throttle_ref_accel: float = 2.0
    brake_ref_accel: float = 3.0
    throttle_max: float = 1.0
    brake_max: float = 1.0
    accel_deadband: float = 0.1


def accel_to_throttle_brake(
    accel_mps2: float,
    throttle_ref_accel: float = AccelMap.throttle_ref_accel,
    brake_ref_accel: float = AccelMap.brake_ref_accel,
    throttle_max: float = AccelMap.throttle_max,
    brake_max: float = AccelMap.brake_max,
    accel_deadband: float = AccelMap.accel_deadband,
):
    """Desired accel [m/s^2] -> (throttle, brake), each in [0, 1]. Pure feed-forward, no state."""
    if accel_mps2 > accel_deadband:
        return _clamp(accel_mps2 / throttle_ref_accel, 0.0, throttle_max), 0.0
    if accel_mps2 < -accel_deadband:
        return 0.0, _clamp(-accel_mps2 / brake_ref_accel, 0.0, brake_max)
    return 0.0, 0.0


def decide_control(
    steer_rad: float,
    accel_mps2: float,
    now_s: float,
    last_vp_t,
    last_speed_t,
    params: ControlParams,
    accel_map: AccelMap,
    aeb_accel_threshold: float = -2.0,
    vp_cmd_timeout: float = 1.0,
    speed_timeout: float = 1.0,
):
    """Full per-tick control decision -> (steer, throttle, brake), all normalized.

    Watchdog: if the latest VisionPilot command OR the latest ego speed is missing/stale,
    hold the vehicle stopped (full brake) — covers VP start-up, a stalled pipeline, and a
    dead speed feed (which would otherwise let VP run speed-blind into runaway accel).
    AEB: an accel command below aeb_accel_threshold forces full brake regardless of the map.
    `last_vp_t` / `last_speed_t` are seconds on the same clock as `now_s`, or None if never seen.
    """
    vp_ok = last_vp_t is not None and (now_s - last_vp_t) < vp_cmd_timeout
    speed_ok = last_speed_t is not None and (now_s - last_speed_t) < speed_timeout
    if not (vp_ok and speed_ok):
        return 0.0, 0.0, 1.0
    steer = steer_to_carla(steer_rad, params.max_steer, params.steer_gain)
    throttle, brake = accel_to_throttle_brake(
        accel_mps2,
        throttle_ref_accel=accel_map.throttle_ref_accel,
        brake_ref_accel=accel_map.brake_ref_accel,
        throttle_max=accel_map.throttle_max,
        brake_max=accel_map.brake_max,
        accel_deadband=accel_map.accel_deadband,
    )
    if accel_mps2 < aeb_accel_threshold:  # planner AEB override
        throttle, brake = 0.0, 1.0
    return steer, throttle, brake
