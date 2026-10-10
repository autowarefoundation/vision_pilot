"""Turning a VisionPilot command into a trajectory.

VisionPilot commands a front tyre angle and an acceleration; an alpasim-style
driver returns a timed trajectory. :func:`rollout_command` bridges the two by
holding the command over the horizon on a kinematic bicycle model, which is
exactly what VisionPilot would do to the vehicle until its next cycle: a
constant-curvature arc with a constant-acceleration speed profile. A trajectory
follower (pure pursuit, as in ``carla_driver_interface``) tracking that arc
reproduces the commanded curvature on whatever vehicle it drives.

Frame: x forward, y left, origin at the vehicle reference point at t = 0.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import numpy.typing as npt

__all__ = ["Rollout", "rollout_command"]


@dataclass(frozen=True)
class Rollout:
    """A sampled trajectory: arrays of equal length, one entry per step."""

    t_s: npt.NDArray[np.float64]  # time since the command, (0, horizon]
    x_m: npt.NDArray[np.float64]
    y_m: npt.NDArray[np.float64]
    yaw_rad: npt.NDArray[np.float64]
    speed_mps: npt.NDArray[np.float64]


def rollout_command(
    *,
    steering_tyre_rad: float,
    acceleration_mps2: float,
    speed_mps: float,
    wheelbase_m: float,
    horizon_s: float = 4.0,
    step_s: float = 0.1,
) -> Rollout:
    """Hold a (tyre angle, acceleration) command for ``horizon_s``.

    ``wheelbase_m`` converts the tyre angle to path curvature, ``tan(delta) / L``;
    pass the value the planner used (VisionPilot's ``front_axle_to_cog_m``) so
    the arc has the curvature the MPC intended. Speed never goes below zero:
    braking ends at a standstill rather than reversing.
    """
    if wheelbase_m <= 0.0 or horizon_s <= 0.0 or step_s <= 0.0:
        raise ValueError("wheelbase_m, horizon_s and step_s must be positive")
    if abs(steering_tyre_rad) >= math.pi / 2:
        raise ValueError("steering_tyre_rad must be within (-pi/2, pi/2)")

    n = max(1, int(round(horizon_s / step_s)))
    t = step_s * np.arange(1, n + 1, dtype=np.float64)
    v0 = max(0.0, float(speed_mps))
    a = float(acceleration_mps2)

    # Distance travelled under constant acceleration, clamped at a standstill.
    if a < 0.0 and v0 > 0.0:
        t_stop = v0 / -a
        tc = np.minimum(t, t_stop)
    elif a < 0.0:
        tc = np.zeros_like(t)
    else:
        tc = t
    s = v0 * tc + 0.5 * a * tc**2
    speed = v0 + a * tc

    # Exact constant-curvature arc; the series form avoids 0/0 when going straight.
    kappa = math.tan(steering_tyre_rad) / wheelbase_m
    yaw = kappa * s
    if abs(kappa) > 1e-9:
        x = np.sin(yaw) / kappa
        y = (1.0 - np.cos(yaw)) / kappa
    else:
        x = s.copy()
        y = 0.5 * kappa * s**2
    return Rollout(t_s=t, x_m=x, y_m=y, yaw_rad=yaw, speed_mps=speed)
