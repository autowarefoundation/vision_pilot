"""Python bindings for the VisionPilot perception -> planning loop.

One :class:`Pilot` runs the same C++ step as the VisionPilot app -- image
preprocessing, AutoDrive/AutoSteer/AutoSpeed inference, lateral and
longitudinal fusion, and the IDM + MPC planner -- on frames handed in from
Python, which makes it a drop-in policy for closed-loop simulation::

    import vision_pilot as vp

    H = vp.ground_homography(width=1920, height=1080, horizontal_fov_deg=52.0,
                             camera_height_m=1.5, pitch_down_deg=2.0)
    pilot = vp.Pilot(vp.PilotConfig(inference=vp.InferenceConfig(model_dir=...)), H)
    result = pilot.step(rgb_image, ego_speed_mps, channel_order=vp.ChannelOrder.RGB, dt_s=0.1)
    if result.ready:
        steer, accel = result.command.steering_tyre_rad, result.command.acceleration_mps2
"""

from importlib.metadata import PackageNotFoundError, version

from ._core import (
    NET_HEIGHT,
    NET_WIDTH,
    PLANNER_DT_S,
    PLANNER_HORIZON,
    AutoDriveOutput,
    AutoSpeedOutput,
    AutoSteerOutput,
    ChannelOrder,
    CipoEstimate,
    Command,
    Detection,
    EngineConfig,
    InferenceConfig,
    LateralEstimate,
    Perception,
    Pilot,
    PilotConfig,
    Plan,
    Planner,
    StepResult,
    Warning,
    compute_preprocess_homography,
)
from .camera import camera_intrinsics, ground_homography

try:
    __version__ = version("vision-pilot")
except PackageNotFoundError:  # pragma: no cover - running from a source tree
    __version__ = "0.0.0"

__all__ = [
    "NET_HEIGHT",
    "NET_WIDTH",
    "PLANNER_DT_S",
    "PLANNER_HORIZON",
    "AutoDriveOutput",
    "AutoSpeedOutput",
    "AutoSteerOutput",
    "ChannelOrder",
    "CipoEstimate",
    "Command",
    "Detection",
    "EngineConfig",
    "InferenceConfig",
    "LateralEstimate",
    "Perception",
    "Pilot",
    "PilotConfig",
    "Plan",
    "Planner",
    "StepResult",
    "Warning",
    "__version__",
    "camera_intrinsics",
    "compute_preprocess_homography",
    "ground_homography",
]
