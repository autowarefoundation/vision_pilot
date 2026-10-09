"""VisionPilot's closed-loop test scenarios, as an autoware_carla_scenario package.

Installed (a member of VisionPilot's uv workspace), it registers itself with the
``scenario`` CLI (the ``autoware_carla_scenario.scenarios`` entry point) and
brings:

* ``scenario=vision_pilot/{cut_in,intersection_straight,pedestrian_dart_out}``
  -- each written as constraints on the map, no lanelet ids, so it runs on any
  map ``scenario-expand`` finds cases on;
* ``driver=vision_pilot_in_process`` -- VisionPilot driving the ego from the
  scenario's own process (:func:`vision_pilot_policy`), on the rig of the
  framework's ``driver=vision_pilot``.

The policy reads ``VISIONPILOT_MODEL_DIR`` (the ``<model>_<precision>.onnx``
weights; defaults to the workspace's ``modules/models/weights``) and
``VISIONPILOT_PROVIDER`` (``cpu``, ``cuda`` or ``tensorrt``; default ``cpu``).
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any

__all__ = ["CONF_DIR", "register", "vision_pilot_policy"]

#: This package's Hydra config groups (scenario/, driver/).
CONF_DIR = Path(__file__).resolve().parent / "conf"

#: The weights in a VisionPilot checkout, where the workspace installs this from.
_WORKSPACE_WEIGHTS = Path(__file__).resolve().parents[4] / "modules" / "models" / "weights"


def register() -> None:
    """Add this package's configs to the ``scenario`` CLI (its entry point)."""
    from autoware_carla_scenario import register_conf_dir

    register_conf_dir(CONF_DIR)


def _model_dir() -> Path:
    value = os.environ.get("VISIONPILOT_MODEL_DIR", "").strip()
    model_dir = Path(value) if value else _WORKSPACE_WEIGHTS
    if not model_dir.is_dir():
        raise RuntimeError(
            f"driver=vision_pilot_in_process: no weights at {model_dir}; "
            "set VISIONPILOT_MODEL_DIR (see vision_pilot_test_scenarios)"
        )
    return model_dir


def vision_pilot_policy() -> Any:
    """VisionPilot as the policy ``driver.policy`` serves in-process.

    The settings of ``vision-pilot-driver --seed 0``: fp32 weights, a fixed
    particle-filter seed and a 1 s MPC budget, so a run does not depend on the
    CPU it runs on.
    """
    from vision_pilot import EngineConfig, InferenceConfig, PilotConfig
    from vision_pilot.driver import VisionPilotDriver

    provider = os.environ.get("VISIONPILOT_PROVIDER", "").strip() or "cpu"
    config = PilotConfig(
        engine=EngineConfig(provider=provider),
        inference=InferenceConfig(model_dir=str(_model_dir()), precision="fp32", seed=0),
        mpc_max_cpu_time_s=1.0,
    )
    return VisionPilotDriver(config)
