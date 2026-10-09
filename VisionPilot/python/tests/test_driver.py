"""VisionPilotDriver over real gRPC, driven by autoware_carla_egodriver's FakeLoop.

FakeLoop (a straight road, rendered through the declared pinhole camera) stands in
for CARLA, so this exercises the whole closed loop: start_session -> camera
homography, image -> Pilot.step -> command -> rolled-out plan -> ego motion.
"""

from __future__ import annotations

import logging

import grpc
import numpy as np
import pytest

pytest.importorskip("autoware_carla_egodriver")

from autoware_carla_egodriver.driver import DriveContext, DriveResult  # noqa: E402
from autoware_carla_egodriver.server import serving  # noqa: E402
from autoware_carla_egodriver.testing import FakeCamera, FakeLoop, LoopResult  # noqa: E402
from vision_pilot.driver import VisionPilotDriver, camera_ground_homography  # noqa: E402

import vision_pilot as vp  # noqa: E402

from .conftest import WEIGHTS  # noqa: E402

# The camera of autoware_carla_scenario's `driver=vision_pilot` preset (VisionPilot's
# CARLA rig): 1920x1280, 50 degree HFOV, looking very slightly down.
FRONT = FakeCamera(
    logical_id="camera_front_narrow_50fov",
    width=1920,
    height=1280,
    fov_deg=50.0,
    z=1.6,
    pitch_down_deg=0.11,
)


def test_camera_homography_matches_the_pinhole_model() -> None:
    camera = FakeCamera(
        width=640, height=360, fov_deg=52.0, x=2.9, y=0.4, z=1.6, pitch_down_deg=3.0
    )
    expected = vp.ground_homography(
        width=640, height=360, horizontal_fov_deg=52.0, camera_height_m=1.6, pitch_down_deg=3.0
    )
    # Origin below the camera: the mount's x and y drop out. The pose crosses the
    # wire as float32, hence the tolerance.
    np.testing.assert_allclose(
        camera_ground_homography(camera.available_camera()), expected, rtol=1e-6, atol=1e-9
    )


def test_warns_about_a_wide_camera(caplog: pytest.LogCaptureFixture) -> None:
    wide = FakeCamera(logical_id="camera_front_wide_120fov", width=960, height=604, fov_deg=120.0)
    with caplog.at_level(logging.WARNING, logger="vision_pilot.driver"):
        camera_ground_homography(wide.available_camera())
    assert "120 degree" in caplog.text and "driver=vision_pilot" in caplog.text


def test_rejects_a_distorted_camera() -> None:
    camera = FakeCamera(logical_id="fisheye").available_camera()
    camera.intrinsics.opencv_pinhole_param.radial_coeffs.append(0.1)
    with pytest.raises(ValueError, match="distorted"):
        camera_ground_homography(camera)


class RecordingDriver(VisionPilotDriver):
    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)  # type: ignore[arg-type]
        self.results: list[DriveResult] = []

    def drive(self, ctx: DriveContext) -> DriveResult:
        result = super().drive(ctx)
        self.results.append(result)
        return result


def _config(**overrides: object) -> vp.PilotConfig:
    if not (WEIGHTS / "autodrive_fp32.onnx").exists():
        pytest.skip(f"model weights not found in {WEIGHTS}")
    return vp.PilotConfig(
        inference=vp.InferenceConfig(model_dir=str(WEIGHTS), seed=0),
        speed_limit_mps=10.0,
        mpc_max_cpu_time_s=10.0,
        **overrides,  # type: ignore[arg-type]
    )


@pytest.fixture(scope="module")
def closed_loop() -> tuple[RecordingDriver, LoopResult]:
    driver = RecordingDriver(_config())
    with serving(driver, port=0, host="127.0.0.1") as port:
        with FakeLoop(f"127.0.0.1:{port}", [FRONT]) as loop:
            result = loop.run(15)
    return driver, result


def test_closed_loop_runs_to_completion(closed_loop) -> None:
    driver, result = closed_loop
    assert result.steps == 15 and not result.terminated_by_policy
    assert len(driver.results) == 15
    # The first frame primes VisionPilot's two-frame buffer; every later one drives.
    ready = [r.debug_scalars["ready"] for r in driver.results]
    assert ready[0] == 0.0 and all(ready[1:])
    assert all(len(r.trajectory_in_rig) == 40 for r in driver.results)
    # The debug scalars reach the runtime.
    assert all(d is not None and d.policy_name == "vision_pilot" for d in result.debug)
    assert "cte_m" in result.debug[-1].scalars


def test_plans_follow_the_command(closed_loop) -> None:
    driver, _ = closed_loop
    for plan in driver.results[1:]:
        s = plan.debug_scalars
        end = plan.trajectory_in_rig.positions[-1]
        # A free road: VisionPilot's IDM accelerates towards the limit.
        assert s["acceleration_mps2"] > 0.0
        assert end[0] > 0.0
        assert np.sign(end[1]) == np.sign(s["steering_tyre_rad"]) or abs(end[1]) < 1e-6


def test_the_ego_moves_forward(closed_loop) -> None:
    _, result = closed_loop
    assert result.ego.positions[-1, 0] > 0.5
    assert result.speeds_mps[-1] > 0.0


def test_unknown_camera_is_reported() -> None:
    driver = VisionPilotDriver(_config(), camera_id="nope")
    with serving(driver, port=0, host="127.0.0.1") as port:
        with FakeLoop(f"127.0.0.1:{port}", [FRONT]) as loop:
            with pytest.raises(grpc.RpcError) as error:
                loop.run(3)
    assert error.value.code() == grpc.StatusCode.FAILED_PRECONDITION
    assert "nope" in error.value.details()
