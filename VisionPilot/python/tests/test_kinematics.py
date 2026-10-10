"""rollout_command(): a held command on a kinematic bicycle model."""

from __future__ import annotations

import math

import numpy as np
import pytest
from vision_pilot.kinematics import rollout_command


def test_straight_constant_acceleration() -> None:
    r = rollout_command(
        steering_tyre_rad=0.0, acceleration_mps2=1.0, speed_mps=10.0, wheelbase_m=2.8
    )
    assert len(r.t_s) == 40 and r.t_s[0] == pytest.approx(0.1) and r.t_s[-1] == pytest.approx(4.0)
    np.testing.assert_allclose(r.x_m, 10.0 * r.t_s + 0.5 * r.t_s**2)
    np.testing.assert_allclose(r.y_m, 0.0)
    np.testing.assert_allclose(r.yaw_rad, 0.0)
    np.testing.assert_allclose(r.speed_mps, 10.0 + r.t_s)


@pytest.mark.parametrize("delta", [0.05, -0.12])
def test_constant_steering_traces_the_commanded_circle(delta: float) -> None:
    L = 2.86
    r = rollout_command(
        steering_tyre_rad=delta, acceleration_mps2=0.0, speed_mps=8.0, wheelbase_m=L
    )
    radius = L / math.tan(delta)  # signed: positive turns left
    # Every point lies on the circle centred at (0, R), and the heading is s / R.
    np.testing.assert_allclose(np.hypot(r.x_m, r.y_m - radius), abs(radius), rtol=1e-9)
    np.testing.assert_allclose(r.yaw_rad, 8.0 * r.t_s / radius, rtol=1e-9)
    assert np.sign(r.y_m[-1]) == np.sign(delta)


def test_braking_stops_instead_of_reversing() -> None:
    r = rollout_command(
        steering_tyre_rad=0.0, acceleration_mps2=-5.0, speed_mps=10.0, wheelbase_m=2.8
    )
    assert r.speed_mps.min() == pytest.approx(0.0)
    assert np.all(np.diff(r.x_m) >= -1e-12)
    np.testing.assert_allclose(r.x_m[-1], 10.0**2 / (2 * 5.0))  # stopping distance


def test_standstill_with_braking_stays_put() -> None:
    r = rollout_command(
        steering_tyre_rad=0.1, acceleration_mps2=-1.0, speed_mps=0.0, wheelbase_m=2.8
    )
    np.testing.assert_allclose(r.x_m, 0.0)
    np.testing.assert_allclose(r.speed_mps, 0.0)


def test_rejects_invalid_arguments() -> None:
    with pytest.raises(ValueError):
        rollout_command(
            steering_tyre_rad=0.0, acceleration_mps2=0.0, speed_mps=1.0, wheelbase_m=0.0
        )
    with pytest.raises(ValueError):
        rollout_command(
            steering_tyre_rad=2.0, acceleration_mps2=0.0, speed_mps=1.0, wheelbase_m=2.8
        )
