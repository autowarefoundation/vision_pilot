"""Unit tests for the VisionPilot-actuation -> CARLA-control mapping.

Pure functions only (no ROS2 / CARLA), so this runs anywhere with pytest:
    pytest test/test_control_mapping.py
"""

import pytest
from visionpilot_carla_bridge.control_mapping import (
    AccelMap,
    ControlParams,
    accel_to_throttle_brake,
    decide_control,
    steer_to_carla,
)

P = ControlParams()  # max_steer=1.2217, steer_gain=1.0


def test_steer_gain_scales_proportionally():
    # default gain 1.0 is a faithful pass-through; gain doubles the response.
    assert steer_to_carla(P.max_steer / 2.0, P.max_steer, 1.0) == pytest.approx(0.5)
    assert steer_to_carla(P.max_steer / 2.0, P.max_steer, 2.0) == pytest.approx(1.0)
    assert steer_to_carla(P.max_steer, P.max_steer, 3.0) == pytest.approx(1.0)  # clamps


def test_steer_respects_vehicle_specific_max_steer():
    # The same tyre angle normalizes differently once the ego's real limit is known
    # (latched /carla_bridge/max_steer_angle overrides the 1.2217 fallback).
    assert steer_to_carla(0.35, 0.70) == pytest.approx(0.5)
    assert steer_to_carla(0.35, 1.2217) == pytest.approx(0.2865, abs=1e-4)


def test_accel_positive_maps_to_throttle_only():
    # Feed-forward: VP's positive accel -> throttle, no brake. No PID state.
    thr, brk = accel_to_throttle_brake(2.0, throttle_ref_accel=2.0)
    assert thr == pytest.approx(1.0)
    assert brk == 0.0


def test_accel_negative_maps_to_brake_only():
    thr, brk = accel_to_throttle_brake(-3.0, brake_ref_accel=3.0)
    assert thr == 0.0
    assert brk == pytest.approx(1.0)


def test_accel_deadband_coasts():
    # Small |accel| inside the deadband -> neither throttle nor brake.
    assert accel_to_throttle_brake(0.05, accel_deadband=0.1) == (0.0, 0.0)


def test_accel_throttle_and_brake_clamped():
    assert accel_to_throttle_brake(10.0, throttle_ref_accel=2.0)[0] == pytest.approx(1.0)
    assert accel_to_throttle_brake(-10.0, brake_ref_accel=3.0)[1] == pytest.approx(1.0)


def test_steer_normalized_by_max_steer():
    assert steer_to_carla(P.max_steer / 2.0, P.max_steer) == pytest.approx(0.5)


def test_steer_clamped_to_unit():
    assert steer_to_carla(2.0 * P.max_steer, P.max_steer) == pytest.approx(1.0)
    assert steer_to_carla(-2.0 * P.max_steer, P.max_steer) == pytest.approx(-1.0)


# ── decide_control: the full per-tick decision (watchdog + maps + AEB) ─────────

A = AccelMap()


def _decide(steer=0.0, accel=0.0, now=10.0, vp_t=9.9, speed_t=9.9, **kw):
    return decide_control(steer, accel, now, vp_t, speed_t, P, A, **kw)


def test_decide_fresh_inputs_map_through():
    steer, thr, brk = _decide(steer=P.max_steer / 2.0, accel=1.0)
    assert steer == pytest.approx(0.5)
    assert thr == pytest.approx(0.5)  # 1.0 / throttle_ref_accel 2.0
    assert brk == 0.0


def test_decide_never_seen_vp_holds_stopped():
    # VP not started yet: no steering, no throttle, full brake.
    assert _decide(steer=0.5, accel=2.0, vp_t=None) == (0.0, 0.0, 1.0)


def test_decide_stale_vp_cmd_holds_stopped():
    assert _decide(steer=0.5, accel=2.0, vp_t=8.0) == (0.0, 0.0, 1.0)  # 2 s > 1 s timeout


def test_decide_stale_speed_holds_stopped():
    # A dead speed feed must hold the car even with fresh VP commands (guards the
    # speed-blind-runaway failure mode).
    assert _decide(steer=0.5, accel=2.0, speed_t=None) == (0.0, 0.0, 1.0)
    assert _decide(steer=0.5, accel=2.0, speed_t=8.0) == (0.0, 0.0, 1.0)


def test_decide_aeb_overrides_to_full_brake():
    steer, thr, brk = _decide(steer=0.2, accel=-5.0)
    assert thr == 0.0
    assert brk == pytest.approx(1.0)
    assert steer != 0.0  # AEB brakes but keeps steering authority


def test_decide_throttle_brake_mutually_exclusive():
    for accel in (-6.0, -2.0, -0.5, 0.0, 0.5, 2.0, 6.0):
        _, thr, brk = _decide(accel=accel)
        assert thr == 0.0 or brk == 0.0, (accel, thr, brk)


def test_decide_custom_timeouts_respected():
    out = _decide(
        steer=0.5, accel=1.0, vp_t=7.0, speed_t=7.0, vp_cmd_timeout=5.0, speed_timeout=5.0
    )
    assert out[2] == 0.0  # 3 s old but within the 5 s window -> drives


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
