"""Bindings that need no model weights."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

import vision_pilot as vp

SCRIPT = Path(__file__).parents[2] / "scripts/find_homography_C_matrix.py"


def test_preprocess_homography_matches_build_script() -> None:
    """The C++ port gives the C the build-time script writes, for the shipped H.yaml."""
    cv2 = pytest.importorskip("cv2")
    import importlib.util

    spec = importlib.util.spec_from_file_location("find_c", SCRIPT)
    assert spec and spec.loader
    script = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(script)

    fs = cv2.FileStorage(str(SCRIPT.parents[1] / "config/H.yaml"), cv2.FILE_STORAGE_READ)
    H = fs.getNode("H").mat()
    fs.release()

    expected = script.find_homography_C_matrix(H).astype(np.float32)
    actual = vp.compute_preprocess_homography(H)
    np.testing.assert_allclose(actual, expected, rtol=1e-4, atol=1e-6)


def test_config_defaults_and_nested_mutation() -> None:
    cfg = vp.PilotConfig()
    assert cfg.engine.provider == "cpu"
    assert cfg.inference.precision == "fp32"
    assert cfg.inference.seed is None
    assert cfg.speed_limit_mps == pytest.approx(33.3)
    assert cfg.mpc_max_cpu_time_s == pytest.approx(0.015)  # the vehicle's real-time budget

    # Nested configs are references, so they can be edited in place.
    cfg.inference.seed = 42
    cfg.inference.long_fusion.radar_enabled = True
    assert cfg.inference.seed == 42
    assert cfg.inference.long_fusion.radar_enabled

    T = np.eye(4)
    T[0, 3] = 1.5
    cfg.inference.long_fusion.cam_T = T
    np.testing.assert_array_equal(cfg.inference.long_fusion.cam_T, T)


def test_keyword_only_configs() -> None:
    with pytest.raises(TypeError):
        vp.EngineConfig("cuda")  # type: ignore[misc]
    assert vp.EngineConfig(provider="cuda", device_id=1).device_id == 1


def test_planner_lateral_response_is_mirror_symmetric() -> None:
    def plan(cte: float, epsi: float) -> vp.Plan:
        # A budget the solver never hits: the result must not depend on CPU speed.
        planner = vp.Planner(speed_limit_mps=20.0, front_axle_to_cog_m=1.4, mpc_max_cpu_time_s=10.0)
        return planner.compute_plan(
            cte=cte,
            epsi=epsi,
            kappa=0.0,
            ego_v=15.0,
            has_cipo=False,
            cipo_v=20.0,
            cipo_distance=9999.0,
        )

    left, right = plan(0.4, 0.02), plan(-0.4, -0.02)
    # [delta_0, delta_0, delta_1, ...]: element 1 is the command for this cycle.
    assert left.steering.ndim == 1 and len(left.steering) >= 2
    assert left.steering.dtype == np.float64
    assert abs(left.steering[1]) > 1e-4
    np.testing.assert_allclose(left.steering, -right.steering, rtol=1e-6, atol=1e-9)
    # Below the speed limit on a free road: accelerate.
    assert left.acceleration > 0
    assert left.warnings == []


def test_planner_brakes_for_a_close_lead_vehicle() -> None:
    planner = vp.Planner(speed_limit_mps=30.0, front_axle_to_cog_m=1.4)
    plan = planner.compute_plan(
        cte=0.0, epsi=0.0, kappa=0.0, ego_v=25.0, has_cipo=True, cipo_v=0.0, cipo_distance=12.0
    )
    assert plan.acceleration < -5.0
    assert vp.Warning.AEB in plan.warnings


def test_constants() -> None:
    assert (vp.NET_WIDTH, vp.NET_HEIGHT) == (1024, 512)
    assert math.isclose(vp.PLANNER_DT_S, 0.05)
