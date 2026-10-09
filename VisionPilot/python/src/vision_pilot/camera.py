"""Ground homographies for ideal pinhole cameras.

VisionPilot projects image points onto a flat road through ``H``, a 3x3
homography from raw image pixels to ground coordinates in the ego frame
(x forward, y left, metres). The app reads ``H`` from ``config/H.yaml``, which
is calibrated for a real camera; a simulator camera has known intrinsics and
mounting, so its ``H`` follows from geometry alone.

Conventions (ISO 8855, right-handed):

* ego frame: origin on the ground, x forward, y left, z up;
* the camera sits at ``(x_m, y_m, camera_height_m)`` in that frame;
* ``pitch_down_deg`` > 0 tilts the optical axis towards the road,
  ``yaw_left_deg`` > 0 turns it left, roll is zero.

CARLA is left-handed (y right) and its pitch is positive *up*, so a camera
attached with ``carla.Transform(carla.Location(x, y, z), carla.Rotation(pitch=p, yaw=w))``
maps to ``x_m=x, y_m=-y, camera_height_m=z + <actor origin height>,
pitch_down_deg=-p, yaw_left_deg=-w``.
"""

from __future__ import annotations

import math

import numpy as np
import numpy.typing as npt

__all__ = ["camera_intrinsics", "ground_homography"]

# Optical frame (x right, y down, z forward) from an ego-aligned camera body
# frame (x forward, y left, z up).
_OPTICAL_FROM_BODY = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])


def camera_intrinsics(
    width: int, height: int, horizontal_fov_deg: float
) -> npt.NDArray[np.float64]:
    """Intrinsic matrix of an ideal pinhole camera with square pixels.

    This is how CARLA's ``sensor.camera.rgb`` forms an image from its
    ``image_size_x``, ``image_size_y`` and ``fov`` attributes.
    """
    if width <= 0 or height <= 0:
        raise ValueError("width and height must be positive")
    if not 0.0 < horizontal_fov_deg < 180.0:
        raise ValueError("horizontal_fov_deg must be in (0, 180)")
    f = width / (2.0 * math.tan(math.radians(horizontal_fov_deg) / 2.0))
    return np.array([[f, 0.0, width / 2.0], [0.0, f, height / 2.0], [0.0, 0.0, 1.0]])


def ground_homography(
    *,
    width: int,
    height: int,
    horizontal_fov_deg: float,
    camera_height_m: float,
    pitch_down_deg: float = 0.0,
    yaw_left_deg: float = 0.0,
    x_m: float = 0.0,
    y_m: float = 0.0,
) -> npt.NDArray[np.float64]:
    """Homography from image pixels to the ground plane, ``(u, v, 1) -> (x, y, 1)``.

    The result is what :class:`vision_pilot.Pilot` takes as ``ground_homography``
    (the C++ app's ``H``), normalised so that ``H[2, 2] == 1``.
    """
    if camera_height_m <= 0.0:
        raise ValueError("camera_height_m must be positive: the camera must be above the road")

    pitch = math.radians(pitch_down_deg)
    yaw = math.radians(yaw_left_deg)
    # Ego <- camera body: yaw about z, then pitch about the (left-pointing) y axis,
    # where a positive angle about +y turns +x towards -z (down).
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    ego_from_body = rz @ ry

    k = camera_intrinsics(width, height, horizontal_fov_deg)
    optical_from_ego = _OPTICAL_FROM_BODY @ ego_from_body.T
    position = np.array([x_m, y_m, camera_height_m])

    # A ground point (x, y, 0) lands on pixel  K R ([x, y, 0] - c)
    #   = K R [e_x  e_y  -c] [x, y, 1]^T,  so pixel <- ground is a homography.
    pixel_from_ground = (
        k @ optical_from_ego @ np.column_stack([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], -position])
    )
    ground_from_pixel = np.linalg.inv(pixel_from_ground)
    return ground_from_pixel / ground_from_pixel[2, 2]
