from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import numpy.typing as npt
import pytest

import vision_pilot as vp

WEIGHTS = Path(
    os.environ.get("VISIONPILOT_MODEL_DIR", Path(__file__).parents[2] / "modules/models/weights")
)

# A camera like the one VisionPilot is specified for: 52 degree HFOV, ~1 MP.
WIDTH, HEIGHT = 1280, 720


@pytest.fixture(scope="session")
def ground_h() -> npt.NDArray[np.float64]:
    return vp.ground_homography(
        width=WIDTH, height=HEIGHT, horizontal_fov_deg=52.0, camera_height_m=1.5, pitch_down_deg=2.0
    )


@pytest.fixture(scope="session")
def road_image() -> npt.NDArray[np.uint8]:
    """A flat grey road under a sky, with two lane markings converging ahead (RGB)."""
    h, w = HEIGHT, WIDTH
    img = np.full((h, w, 3), 90, np.uint8)
    img[: h * 2 // 5] = (200, 170, 120)
    rows = np.arange(h * 2 // 5, h)
    t = (rows - rows[0]) / (h - rows[0])
    for start, end in ((w * 0.47, w * 0.30), (w * 0.53, w * 0.70)):
        cols = (start + (end - start) * t).astype(int)
        for dc in range(-3, 4):
            img[rows, np.clip(cols + dc, 0, w - 1)] = 255
    return img


def make_pilot(
    ground_h: npt.NDArray[np.float64], seed: int | None = 1, **inference: object
) -> vp.Pilot:
    if not (WEIGHTS / "autodrive_fp32.onnx").exists():
        pytest.skip(f"model weights not found in {WEIGHTS}")
    config = vp.PilotConfig(
        inference=vp.InferenceConfig(model_dir=str(WEIGHTS), seed=seed, **inference),
        # Let every MPC solve converge, so results do not depend on the machine.
        mpc_max_cpu_time_s=10.0,
    )
    return vp.Pilot(config, ground_h)
