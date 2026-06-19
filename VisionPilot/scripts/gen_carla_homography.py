"""Generate the CARLA ground homography H_carla.yaml (image pixel -> world ground BEV).

CARLA UE5 0.10 publishes a `camera_info`, but its intrinsic matrix K is unreliable
(observed fx ~ 1e5 for a 1280px / 120deg-fov camera, which should be ~370). So we build
the intrinsics analytically from the spawn-config camera geometry instead:

  fx = fy = (width / 2) / tan(hfov / 2)   cx = width/2   cy = height/2

and a flat-ground inverse-perspective map from the camera extrinsics (height + pitch,
no roll/yaw). H maps a full-frame image pixel (u, v, 1) to world ground (X forward,
Y left, 1) in metres — the SAME convention as VisionPilot/config/H.yaml and the fixed
model-view V in find_homography_C_matrix.py, so the existing C-matrix codegen consumes
it unchanged. Defaults match Simulation/CARLA/ROS2/config/VisionPilot_carla10.json
(main_cam: 1280x720, fov 60, mount x=1.25 z=1.58, pitch 0).

Usage:
  python gen_carla_homography.py --output ../../Simulation/CARLA/ROS2/config/H_carla.yaml
"""

import argparse
import math
from pathlib import Path

import cv2
import numpy as np


def rotation_world_to_optical(pitch_deg: float) -> np.ndarray:
    """World (X fwd, Y left, Z up) -> camera optical (x right, y down, z fwd).

    Base (pitch=0): x=-Y, y=-Z, z=X. A positive pitch tilts the camera downward
    (rotation about the camera x-axis), so the ground fills more of the frame.
    """
    base = np.array([[0.0, -1.0, 0.0],
                     [0.0, 0.0, -1.0],
                     [1.0, 0.0, 0.0]], dtype=np.float64)
    p = math.radians(pitch_deg)
    # Rotate about the optical x-axis (points right) by +pitch -> look down.
    rx = np.array([[1.0, 0.0, 0.0],
                   [0.0, math.cos(p), -math.sin(p)],
                   [0.0, math.sin(p), math.cos(p)]], dtype=np.float64)
    return rx @ base


def build_homography(width, height, fov_deg, cam_x, cam_z, pitch_deg) -> np.ndarray:
    fx = (width / 2.0) / math.tan(math.radians(fov_deg) / 2.0)
    fy = fx
    cx, cy = width / 2.0, height / 2.0
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]], dtype=np.float64)

    R = rotation_world_to_optical(pitch_deg)
    C = np.array([cam_x, 0.0, cam_z], dtype=np.float64)  # camera centre in world

    # Sample a grid of world ground points over the road region and project to pixels;
    # then fit H (pixel -> world). A homography needs >= 4 correspondences; the grid is
    # over-determined for robustness.
    world_pts, pix_pts = [], []
    for X in np.linspace(5.0, 60.0, 12):
        for Y in np.linspace(-6.0, 6.0, 9):
            Pw = np.array([X, Y, 0.0]) - C
            Pc = R @ Pw
            if Pc[2] <= 0.1:           # behind / too close to the camera plane
                continue
            u = K[0, 0] * Pc[0] / Pc[2] + K[0, 2]
            v = K[1, 1] * Pc[1] / Pc[2] + K[1, 2]
            if not (0 <= u < width and 0 <= v < height):
                continue
            pix_pts.append([u, v])
            world_pts.append([X, Y])

    pix = np.asarray(pix_pts, dtype=np.float64)
    wld = np.asarray(world_pts, dtype=np.float64)
    H, _ = cv2.findHomography(pix, wld, method=0)
    return H


def main() -> None:
    ap = argparse.ArgumentParser(description="Generate CARLA ground homography H_carla.yaml")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fov", type=float, default=60.0, help="horizontal fov [deg]")
    ap.add_argument("--cam-x", type=float, default=1.25, help="camera forward offset [m]")
    ap.add_argument("--cam-z", type=float, default=1.58, help="camera height [m]")
    ap.add_argument("--pitch", type=float, default=0.0, help="camera pitch down [deg]")
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()

    H = build_homography(args.width, args.height, args.fov, args.cam_x, args.cam_z, args.pitch)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fs = cv2.FileStorage(str(args.output.resolve()), cv2.FILE_STORAGE_WRITE)
    fs.write("H", H.astype(np.float64))
    fs.release()
    # FileStorage writes 'opencv-matrix'; the loader (cv2.FileStorage / generate_config) reads it.
    print(f"H_carla written to {args.output.resolve()}\n{H}")


if __name__ == "__main__":
    main()
