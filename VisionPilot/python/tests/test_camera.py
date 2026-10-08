"""ground_homography() against an independent forward projection."""

from __future__ import annotations

import math

import numpy as np
import pytest

import vision_pilot as vp


def project(
    point_xyz: np.ndarray,
    *,
    width: int,
    height: int,
    hfov: float,
    cam: np.ndarray,
    pitch: float,
    yaw: float,
):
    """Pinhole projection written from first principles, without matrices."""
    d = point_xyz - cam
    # Camera axes in the ego frame: forward, left, up after yaw then pitch-down.
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    fwd = np.array([cp * cy, cp * sy, -sp])
    left = np.array([-sy, cy, 0.0])
    up = np.cross(fwd, left)
    z, x, y = d @ fwd, -(d @ left), -(d @ up)
    f = width / (2 * math.tan(math.radians(hfov) / 2))
    return np.array([f * x / z + width / 2, f * y / z + height / 2])


@pytest.mark.parametrize(
    "pitch_deg, yaw_deg, x_m, y_m",
    [(0.0, 0.0, 0.0, 0.0), (3.0, 0.0, 1.2, 0.0), (5.0, 4.0, 2.0, -0.3)],
)
def test_ground_points_round_trip(pitch_deg: float, yaw_deg: float, x_m: float, y_m: float) -> None:
    H = vp.ground_homography(
        width=1920,
        height=1080,
        horizontal_fov_deg=60.0,
        camera_height_m=1.4,
        pitch_down_deg=pitch_deg,
        yaw_left_deg=yaw_deg,
        x_m=x_m,
        y_m=y_m,
    )
    cam = np.array([x_m, y_m, 1.4])
    for gx, gy in [(10.0, 0.0), (25.0, 3.5), (60.0, -2.0)]:
        uv = project(
            np.array([gx, gy, 0.0]),
            width=1920,
            height=1080,
            hfov=60.0,
            cam=cam,
            pitch=math.radians(pitch_deg),
            yaw=math.radians(yaw_deg),
        )
        g = H @ np.array([uv[0], uv[1], 1.0])
        np.testing.assert_allclose(g[:2] / g[2], [gx, gy], atol=1e-6)


def test_conventions() -> None:
    H = vp.ground_homography(
        width=1000, height=500, horizontal_fov_deg=90.0, camera_height_m=1.0, pitch_down_deg=10.0
    )
    assert H[2, 2] == pytest.approx(1.0)

    def ground(u: float, v: float) -> np.ndarray:
        g = H @ np.array([u, v, 1.0])
        return g[:2] / g[2]

    # The principal point of a camera pitched down 10 deg at 1 m hits the road
    # straight ahead at 1 / tan(10 deg).
    np.testing.assert_allclose(ground(500, 250), [1 / math.tan(math.radians(10)), 0.0], atol=1e-9)
    # Lower in the image is closer; left in the image is +y (left).
    assert ground(500, 400)[0] < ground(500, 300)[0]
    assert ground(100, 400)[1] > 0 > ground(900, 400)[1]


@pytest.mark.parametrize(
    "width, fov, height_m", [(100, 60.0, 0.0), (100, 180.0, 1.0), (0, 60.0, 1.0)]
)
def test_rejects_invalid_cameras(width: int, fov: float, height_m: float) -> None:
    with pytest.raises(ValueError):
        vp.ground_homography(
            width=width, height=100, horizontal_fov_deg=fov, camera_height_m=height_m
        )
