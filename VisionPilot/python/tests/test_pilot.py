"""The full loop step against the shipped ONNX weights (CPU)."""

from __future__ import annotations

import threading
import time

import numpy as np
import pytest

import vision_pilot as vp

from .conftest import make_pilot

RGB = vp.ChannelOrder.RGB
BGR = vp.ChannelOrder.BGR


@pytest.fixture(scope="module")
def pilot(ground_h):
    return make_pilot(ground_h)


def test_first_frame_primes_the_buffer(pilot, road_image) -> None:
    pilot.reset()
    first = pilot.step(road_image, 10.0, channel_order=RGB, dt_s=0.1)
    assert not first.ready
    assert first.perception is None and first.plan is None and first.command is None
    # The preprocessed network inputs are available from the first frame on.
    assert first.warped.shape == first.resized.shape == (vp.NET_HEIGHT, vp.NET_WIDTH, 3)

    second = pilot.step(road_image, 10.0, channel_order=RGB, dt_s=0.1)
    assert second.ready
    assert second.perception is not None and second.plan is not None and second.command is not None
    assert second.perception.frame_id == 2
    assert np.isfinite(second.command.steering_tyre_rad)
    assert np.isfinite(second.command.acceleration_mps2)
    assert second.command.steering_tyre_rad == second.plan.steering[1]
    assert second.command.acceleration_mps2 == second.plan.acceleration
    assert second.perception.auto_steer.xp.shape == (64,)


def test_reset_starts_a_new_episode(pilot, road_image) -> None:
    pilot.step(road_image, 10.0, channel_order=RGB)
    assert pilot.step(road_image, 10.0, channel_order=RGB).ready
    pilot.reset()
    assert not pilot.step(road_image, 10.0, channel_order=RGB).ready


def test_output_images_are_read_only_views_that_outlive_the_result(pilot, road_image) -> None:
    pilot.reset()
    warped = pilot.step(road_image, 10.0, channel_order=RGB).warped
    assert not warped.flags.writeable
    with pytest.raises(ValueError):
        warped[0, 0, 0] = 1
    # The array keeps the underlying cv::Mat alive after the StepResult is gone.
    assert warped.sum() > 0


def test_seeded_runs_are_reproducible_and_channel_order_is_honoured(ground_h, road_image) -> None:
    bgr = np.ascontiguousarray(road_image[..., ::-1])
    rgb = road_image.copy()
    a, b = make_pilot(ground_h, seed=5), make_pilot(ground_h, seed=5)
    for _ in range(3):
        ra = a.step(rgb, 12.0, channel_order=RGB, dt_s=0.1)
        rb = b.step(bgr, 12.0, channel_order=BGR, dt_s=0.1)
        np.testing.assert_array_equal(ra.resized, rb.resized)
    # The network inputs are BGR, and the caller's arrays are left untouched.
    np.testing.assert_array_equal(ra.resized[0, 0], road_image[0, 0, ::-1])
    np.testing.assert_array_equal(rgb, road_image)
    np.testing.assert_array_equal(bgr, road_image[..., ::-1])
    assert ra.command is not None and rb.command is not None
    assert ra.command.steering_tyre_rad == rb.command.steering_tyre_rad
    assert ra.command.acceleration_mps2 == rb.command.acceleration_mps2
    assert ra.perception.lateral.cte_m == rb.perception.lateral.cte_m


def test_explicit_preprocess_homography_is_used(ground_h) -> None:
    C = vp.compute_preprocess_homography(ground_h)
    pilot = vp.Pilot(make_pilot(ground_h).config, ground_h, C * 2.0)
    np.testing.assert_allclose(pilot.preprocess_homography, C * 2.0, rtol=1e-6)
    np.testing.assert_allclose(pilot.ground_homography, ground_h)


@pytest.mark.parametrize(
    "image",
    [
        np.zeros((720, 1280), np.uint8),  # no channel axis
        np.zeros((720, 1280, 4), np.uint8),  # RGBA
        np.zeros((720, 1280, 3), np.float32),  # float, not silently truncated
        np.zeros((720, 1280, 3), np.uint8)[:, ::2],  # not C-contiguous
    ],
)
def test_rejects_malformed_images(pilot, image) -> None:
    with pytest.raises(TypeError):
        pilot.step(image, 10.0, channel_order=RGB)


def test_channel_order_is_required(pilot, road_image) -> None:
    with pytest.raises(TypeError):
        pilot.step(road_image, 10.0)  # type: ignore[call-arg]


def test_missing_weights_raise(ground_h, tmp_path) -> None:
    config = vp.PilotConfig(inference=vp.InferenceConfig(model_dir=str(tmp_path)))
    with pytest.raises(RuntimeError, match="Model file not found"):
        vp.Pilot(config, ground_h)


def test_step_releases_the_gil(pilot, road_image) -> None:
    """A Python thread keeps running while another one is inside step()."""
    pilot.reset()
    ticks = 0
    done = threading.Event()

    def spin() -> None:
        nonlocal ticks
        while not done.is_set():
            ticks += 1
            time.sleep(0)

    spinner = threading.Thread(target=spin)
    spinner.start()
    try:
        before = ticks
        pilot.step(road_image, 10.0, channel_order=RGB)
        pilot.step(road_image, 10.0, channel_order=RGB)
        during = ticks - before
    finally:
        done.set()
        spinner.join()
    assert during > 100
