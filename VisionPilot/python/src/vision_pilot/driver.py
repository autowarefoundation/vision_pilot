"""VisionPilot as an alpasim ``egodriver`` policy, for closed-loop simulation.

:class:`VisionPilotDriver` plugs a :class:`vision_pilot.Pilot` into
``carla-driver-interface`` (the policy side of alpasim's ``EgodriverService``,
which autoware_carla_scenario speaks). Any runtime of that protocol can then
drive VisionPilot unmodified: autoware_carla_scenario's scenarios against CARLA
(``scenario driver=vision_pilot``), ``carla_driver_interface.testing.FakeLoop``
without a simulator, or upstream alpasim.

Per ``drive`` call:

1. the newest frame of the configured camera, if it is new, goes through
   ``Pilot.step`` (preprocess -> networks -> fusion -> IDM + MPC), with the
   simulation time between frames as ``dt_s``;
2. the resulting command (front tyre angle, acceleration) is held over the
   horizon on a kinematic bicycle model (:func:`vision_pilot.kinematics.rollout_command`)
   and returned as the plan in the rig frame.

The ground homography VisionPilot projects with is derived from the camera the
runtime declares in ``start_session`` (pinhole intrinsics and pose in the rig),
with its origin on the road directly below the camera.

Requires the ``closed-loop-test`` extra: ``pip install 'vision-pilot[closed-loop-test]'``.
"""

from __future__ import annotations

import logging
import math
import threading
from dataclasses import dataclass, field

import numpy as np
import numpy.typing as npt
from carla_driver_interface.driver import (
    BaseDriver,
    CameraFrame,
    DriveContext,
    DriveResult,
    SessionState,
)
from carla_driver_interface.geometry import Pose, Trajectory
from carla_driver_interface.protocol import AvailableCamera

from ._core import ChannelOrder, Command, Pilot, PilotConfig, StepResult, Warning
from .camera import ground_homography_from_extrinsics
from .kinematics import rollout_command

__all__ = ["VisionPilotDriver", "camera_ground_homography"]

logger = logging.getLogger(__name__)

_HOLD = Command(0.0, 0.0)

#: Horizontal fields of view the networks were trained around (52-55 degrees);
#: outside this band the driver still runs, but warns.
_EXPECTED_HFOV_DEG = (40.0, 70.0)


def camera_ground_homography(camera: AvailableCamera) -> npt.NDArray[np.float64]:
    """VisionPilot's ground homography for an alpasim ``AvailableCamera``.

    The camera must use the OpenCV pinhole model without distortion, which is
    how autoware_carla_scenario describes CARLA's ``sensor.camera.rgb``. The
    homography's origin is the point on the road directly below the camera, with
    the rig's axes (x forward, y left): the convention of the calibrated
    ``config/H.yaml`` the app ships with.
    """
    name, spec = camera.logical_id, camera.intrinsics
    model = spec.WhichOneof("camera_param")
    if model != "opencv_pinhole_param":
        raise ValueError(
            f"camera {name!r}: VisionPilot needs an OpenCV pinhole camera, got {model!r}"
        )
    pinhole = spec.opencv_pinhole_param
    if (
        any(pinhole.radial_coeffs)
        or any(pinhole.tangential_coeffs)
        or any(pinhole.thin_prism_coeffs)
    ):
        raise ValueError(f"camera {name!r}: distorted images are not supported")

    intrinsics = np.array(
        [
            [pinhole.focal_length_x, 0.0, pinhole.principal_point_x],
            [0.0, pinhole.focal_length_y, pinhole.principal_point_y],
            [0.0, 0.0, 1.0],
        ]
    )
    hfov = math.degrees(2.0 * math.atan(spec.resolution_w / (2.0 * pinhole.focal_length_x)))
    low, high = _EXPECTED_HFOV_DEG
    if not low <= hfov <= high:
        logger.warning(
            "camera %r has a %.0f degree horizontal FOV; VisionPilot expects about 52 "
            "(autoware_carla_scenario: driver=vision_pilot)",
            name,
            hfov,
        )
    pose_in_rig = Pose.from_proto(camera.rig_to_camera)
    height = float(pose_in_rig.position[2])
    return ground_homography_from_extrinsics(
        intrinsics=intrinsics,
        rotation_ego_from_camera=pose_in_rig.rotation_matrix,
        position_m=(0.0, 0.0, height),
    )


@dataclass
class _Episode:
    """Per-session state: one Pilot per rollout, so episodes never share filters."""

    pilot: Pilot
    camera_id: str
    last_frame_us: int | None = None
    command: Command = field(default_factory=lambda: Command(0.0, 0.0))
    last_step: StepResult | None = None
    lock: threading.Lock = field(default_factory=threading.Lock)


class VisionPilotDriver(BaseDriver):
    """Drive with VisionPilot's perception and planning stack."""

    name = "vision_pilot"
    # The Pilot keeps its own two-frame buffer.
    frame_history_length = 1

    def __init__(
        self,
        config: PilotConfig,
        *,
        camera_id: str | None = None,
        horizon_s: float = 4.0,
        step_s: float = 0.1,
    ) -> None:
        """
        config:    the Pilot configuration (models, engine, speed limit, ...).
        camera_id: logical id of the front camera to drive from; when ``None``,
                   the session must declare exactly one camera.
        horizon_s, step_s: length and sampling of the returned plan.
        """
        self.config = config
        self.camera_id = camera_id
        self.horizon_s = horizon_s
        self.step_s = step_s
        self._episodes: dict[str, _Episode] = {}
        self._lock = threading.Lock()

    # -- session lifecycle ---------------------------------------------------

    def on_session_start(self, session: SessionState) -> None:
        camera_id = self._pick_camera(session)
        H = camera_ground_homography(session.cameras[camera_id])
        episode = _Episode(pilot=Pilot(self.config, H), camera_id=camera_id)
        with self._lock:
            self._episodes[session.uuid] = episode
        logger.info("session %s: driving from %r", session.uuid, camera_id)

    def on_session_close(self, session: SessionState) -> None:
        with self._lock:
            self._episodes.pop(session.uuid, None)

    def _pick_camera(self, session: SessionState) -> str:
        if self.camera_id is not None:
            if self.camera_id not in session.cameras:
                raise ValueError(
                    f"camera {self.camera_id!r} not in this session: {sorted(session.cameras)}"
                )
            return self.camera_id
        if len(session.cameras) != 1:
            raise ValueError(
                f"the session has cameras {sorted(session.cameras)}; choose one with camera_id"
            )
        return next(iter(session.cameras))

    # -- driving -------------------------------------------------------------

    def drive(self, ctx: DriveContext) -> DriveResult:
        session = ctx.session
        with self._lock:
            episode = self._episodes[session.uuid]
        speed = session.speed_mps()

        with episode.lock:
            with session.lock:
                frame = session.latest_frame(episode.camera_id)
            # The logical id is the camera's, so a LiDAR sweep never lands here.
            if isinstance(frame, CameraFrame) and frame.frame_end_us != episode.last_frame_us:
                self._step(episode, frame, speed)
            command = episode.command
            step = episode.last_step

        return DriveResult(
            trajectory_in_rig=self._plan(ctx.time_now_us, command, speed),
            debug_scalars=_debug_scalars(command, step, speed),
        )

    def _step(self, episode: _Episode, frame: CameraFrame, speed: float) -> None:
        dt_s = 0.0
        if episode.last_frame_us is not None:
            dt_s = (frame.frame_end_us - episode.last_frame_us) * 1e-6
        result = episode.pilot.step(
            frame.as_array(), speed, channel_order=ChannelOrder.RGB, dt_s=dt_s
        )
        episode.last_frame_us = frame.frame_end_us
        episode.last_step = result
        # Until the two-frame buffer is primed there is no command: hold, as
        # the vehicle interface would without a message.
        episode.command = result.command if result.command is not None else _HOLD

    def _plan(self, time_now_us: int, command: Command, speed: float) -> Trajectory:
        rollout = rollout_command(
            steering_tyre_rad=command.steering_tyre_rad,
            acceleration_mps2=command.acceleration_mps2,
            speed_mps=speed,
            wheelbase_m=self.config.front_axle_to_cog_m,
            horizon_s=self.horizon_s,
            step_s=self.step_s,
        )
        plan = Trajectory.empty()
        for t, x, y, yaw in zip(rollout.t_s, rollout.x_m, rollout.y_m, rollout.yaw_rad):
            plan.append(time_now_us + int(round(t * 1e6)), Pose.from_xyz_yaw(x, y, 0.0, yaw))
        return plan


def _debug_scalars(command: Command, step: StepResult | None, speed: float) -> dict[str, float]:
    scalars = {
        "current_speed_mps": speed,
        "steering_tyre_rad": command.steering_tyre_rad,
        "acceleration_mps2": command.acceleration_mps2,
        "ready": float(step is not None and step.ready),
    }
    if step is None or step.perception is None or step.plan is None:
        return scalars
    lateral, cipo = step.perception.lateral, step.perception.cipo
    warnings = set(step.plan.warnings)
    scalars.update(
        {
            "cte_m": lateral.cte_m,
            "yaw_error_rad": lateral.yaw_rad,
            "curvature_1pm": lateral.curvature,
            "path_valid": float(lateral.path_valid),
            "has_cipo": float(step.has_cipo),
            "cipo_distance_m": cipo.distance_m,
            "cipo_velocity_mps": cipo.velocity_ms,
            "inference_ms": step.perception.pre_ms + step.perception.wall_ms,
            "fcw": float(Warning.FCW in warnings),
            "aeb": float(Warning.AEB in warnings),
            "ldw": float(Warning.LLDW in warnings or Warning.RLDW in warnings),
        }
    )
    return scalars


def main(argv: list[str] | None = None) -> None:
    """``vision-pilot-driver``: serve VisionPilot over alpasim's EgodriverService."""
    import argparse
    import os

    from carla_driver_interface.server import run_server

    from ._core import EngineConfig, InferenceConfig

    parser = argparse.ArgumentParser(prog="vision-pilot-driver", description=main.__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=50051)
    parser.add_argument(
        "--model-dir",
        default=os.environ.get("VISIONPILOT_MODEL_DIR", ""),
        help="directory with the <model>_<precision>.onnx weights (env: VISIONPILOT_MODEL_DIR)",
    )
    parser.add_argument("--precision", default="fp32", choices=["fp32", "int8"])
    parser.add_argument("--provider", default="cpu", choices=["cpu", "cuda", "tensorrt"])
    parser.add_argument("--device-id", type=int, default=0)
    parser.add_argument("--camera", default=None, help="logical id of the front camera")
    parser.add_argument(
        "--speed-limit",
        type=float,
        default=PilotConfig().speed_limit_mps,
        help="cruise speed on a free road [m/s]",
    )
    parser.add_argument(
        "--front-axle-to-cog",
        type=float,
        default=PilotConfig().front_axle_to_cog_m,
        help="planner length L [m]; also turns tyre angle into plan curvature",
    )
    parser.add_argument("--seed", type=int, default=None, help="particle-filter seed")
    parser.add_argument(
        "--mpc-max-cpu-time",
        type=float,
        default=1.0,
        help="lateral MPC budget per solve [s]; 0.015 reproduces the vehicle's real-time cut-off",
    )
    parser.add_argument("--horizon", type=float, default=4.0, help="plan horizon [s]")
    parser.add_argument("--step", type=float, default=0.1, help="plan sampling [s]")
    args = parser.parse_args(argv)

    if not args.model_dir:
        parser.error("--model-dir (or VISIONPILOT_MODEL_DIR) is required")
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s"
    )

    config = PilotConfig(
        engine=EngineConfig(provider=args.provider, device_id=args.device_id),
        inference=InferenceConfig(
            model_dir=args.model_dir, precision=args.precision, seed=args.seed
        ),
        speed_limit_mps=args.speed_limit,
        front_axle_to_cog_m=args.front_axle_to_cog,
        mpc_max_cpu_time_s=args.mpc_max_cpu_time,
    )
    driver = VisionPilotDriver(
        config, camera_id=args.camera, horizon_s=args.horizon, step_s=args.step
    )
    run_server(driver, port=args.port, host=args.host)


if __name__ == "__main__":
    main()
