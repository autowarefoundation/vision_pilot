"""Unit tests for the ackermann -> CarlaEgoVehicleControl mapping.

Tests the pure function only (no ROS2 / CARLA), so it runs anywhere with pytest.
    pytest test_control_bridge.py
"""
import pytest

from bridge_logic import BridgeParams, ackermann_to_control

P = BridgeParams()  # defaults: max_steer=1.22, kp=3.0, throttle_max=1.0, brake_deadband=0.5


def test_steer_normalized_by_max_steer():
    # Half the max steer angle -> half of full lock.
    _, steer, _ = ackermann_to_control(P.max_steer / 2.0, 0.0, 0.0, P)
    assert steer == pytest.approx(0.5)


def test_steer_clamped_to_unit():
    # Beyond max steer (and the opposite sign) saturates at +/-1.
    assert ackermann_to_control(2.0 * P.max_steer, 0.0, 0.0, P)[1] == pytest.approx(1.0)
    assert ackermann_to_control(-2.0 * P.max_steer, 0.0, 0.0, P)[1] == pytest.approx(-1.0)


def test_throttle_on_positive_speed_error():
    # target 10, current 0 -> kp*err far exceeds the cap, clamped to throttle_max.
    throttle, _, brake = ackermann_to_control(0.0, 10.0, 0.0, P)
    assert throttle == pytest.approx(P.throttle_max)
    assert brake == 0.0
    # Small positive error stays below the cap and unclamped.
    throttle, _, brake = ackermann_to_control(0.0, 0.2, 0.0, P)
    assert throttle == pytest.approx(P.kp * 0.2)
    assert brake == 0.0


def test_brake_past_deadband():
    # current well above target -> brake, no throttle.
    throttle, _, brake = ackermann_to_control(0.0, 0.0, 5.0, P)
    assert throttle == 0.0
    assert brake == pytest.approx(min(1.0, P.kp * 5.0))


def test_coast_inside_deadband():
    # |err| within the deadband -> neither throttle nor brake.
    throttle, _, brake = ackermann_to_control(0.0, 10.0, 10.2, P)
    assert throttle == 0.0
    assert brake == 0.0


def test_brake_clamped_to_unit():
    throttle, _, brake = ackermann_to_control(0.0, 0.0, 100.0, P)
    assert throttle == 0.0
    assert brake == pytest.approx(1.0)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
