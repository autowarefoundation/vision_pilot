"""VisionPilotDriver against carla_driver_interface's runtime, over real gRPC.

FakeWorld (a kinematic bicycle and synthetic frames) stands in for CARLA, so
this exercises the whole closed loop: start_session -> camera homography,
image -> Pilot.step -> command -> rolled-out plan -> trajectory follower.
"""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

cdi = pytest.importorskip("carla_driver_interface")

from carla_driver_interface.driver.base import DriveContext, DriveResult  # noqa: E402
from carla_driver_interface.driver.server import serving  # noqa: E402
from carla_driver_interface.fakes import FakeWorld  # noqa: E402
from carla_driver_interface.grpc_api import ImageFormat  # noqa: E402
from carla_driver_interface.runtime import CarlaRuntime, RuntimeConfig, ScenarioSpec  # noqa: E402
from carla_driver_interface.runtime.config import CameraConfig  # noqa: E402
from carla_driver_interface.runtime.conversions import (  # noqa: E402
    available_camera,
    camera_pose_in_rig,
)
from vision_pilot.driver import VisionPilotDriver, camera_ground_homography  # noqa: E402

import vision_pilot as vp  # noqa: E402

from .conftest import WEIGHTS  # noqa: E402

# A VisionPilot-like front camera: 52 degree HFOV, tilted 3 degrees down.
FRONT = CameraConfig(
    logical_id="camera_front", width=640, height=360, fov_deg=52.0, x=1.5, z=1.6, pitch_deg=-3.0
)


def test_camera_homography_follows_the_carla_mount() -> None:
    """CARLA pitch is positive up and y right; the homography is rig-aligned."""
    rear_axle_offset = -1.4
    camera = available_camera(
        logical_id=FRONT.logical_id,
        width=FRONT.width,
        height=FRONT.height,
        horizontal_fov_deg=FRONT.fov_deg,
        pose_in_rig=camera_pose_in_rig(
            FRONT.x, 0.4, FRONT.z, FRONT.pitch_deg, 0.0, 0.0, rear_axle_offset
        ),
    )
    expected = vp.ground_homography(
        width=FRONT.width,
        height=FRONT.height,
        horizontal_fov_deg=FRONT.fov_deg,
        camera_height_m=FRONT.z,
        pitch_down_deg=3.0,
    )
    # Origin below the camera: the lateral and longitudinal mount offsets drop out.
    # The pose crosses the wire as float32, hence the tolerance.
    np.testing.assert_allclose(camera_ground_homography(camera), expected, rtol=1e-6, atol=1e-9)


def test_rejects_a_distorted_camera() -> None:
    camera = available_camera(
        logical_id="fisheye",
        width=64,
        height=48,
        horizontal_fov_deg=90.0,
        pose_in_rig=camera_pose_in_rig(1.0, 0.0, 1.5, 0.0, 0.0, 0.0, 0.0),
    )
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


@pytest.fixture(scope="module")
def closed_loop() -> tuple[RecordingDriver, object, FakeWorld]:
    if not (WEIGHTS / "autodrive_fp32.onnx").exists():
        pytest.skip(f"model weights not found in {WEIGHTS}")
    config = vp.PilotConfig(
        inference=vp.InferenceConfig(model_dir=str(WEIGHTS), seed=0),
        speed_limit_mps=10.0,
        mpc_max_cpu_time_s=10.0,
    )
    driver = RecordingDriver(config)
    scenario = ScenarioSpec(map_name="FakeTown", name="vision_pilot")
    with serving(driver, port=0, host="127.0.0.1") as port:
        runtime_config = replace(
            RuntimeConfig(driver_address=f"127.0.0.1:{port}", image_format=ImageFormat.JPEG),
            cameras=[FRONT],
            max_steps=15,
        )
        world = FakeWorld(runtime_config, scenario)
        outcome = CarlaRuntime(world, runtime_config, scenario).run_rollout()
    return driver, outcome, world


def test_closed_loop_runs_to_completion(closed_loop) -> None:
    driver, outcome, _ = closed_loop
    assert outcome.rollout_return.error == ""
    assert not outcome.terminated_by_driver
    assert len(driver.results) == 15
    # The first frame primes VisionPilot's two-frame buffer; every later one drives.
    ready = [r.debug_scalars["ready"] for r in driver.results]
    assert ready[0] == 0.0 and all(ready[1:])
    assert all(len(r.trajectory_in_rig) == 40 for r in driver.results)


def test_plans_follow_the_command(closed_loop) -> None:
    driver, _, _ = closed_loop
    for result in driver.results[1:]:
        s = result.debug_scalars
        end = result.trajectory_in_rig.positions[-1]
        # A free synthetic road: VisionPilot's IDM accelerates towards the limit.
        assert s["acceleration_mps2"] > 0.0
        assert end[0] > 0.0
        assert np.sign(end[1]) == np.sign(s["steering_tyre_rad"]) or abs(end[1]) < 1e-6


def test_the_ego_moves_forward(closed_loop) -> None:
    _, _, world = closed_loop
    start = world.history[0].pose_local_to_rig
    end = world.history[-1].pose_local_to_rig
    forward = start.rotation_matrix[:, 0]
    assert float(np.dot(end.position - start.position, forward)) > 0.5


def test_unknown_camera_is_reported() -> None:
    if not (WEIGHTS / "autodrive_fp32.onnx").exists():
        pytest.skip(f"model weights not found in {WEIGHTS}")
    config = vp.PilotConfig(inference=vp.InferenceConfig(model_dir=str(WEIGHTS)))
    driver = VisionPilotDriver(config, camera_id="nope")
    scenario = ScenarioSpec(map_name="FakeTown", name="bad_camera")
    with serving(driver, port=0, host="127.0.0.1") as port:
        runtime_config = replace(
            RuntimeConfig(driver_address=f"127.0.0.1:{port}"), cameras=[FRONT], max_steps=3
        )
        outcome = CarlaRuntime(
            FakeWorld(runtime_config, scenario), runtime_config, scenario
        ).run_rollout()
    assert "nope" in outcome.rollout_return.error
