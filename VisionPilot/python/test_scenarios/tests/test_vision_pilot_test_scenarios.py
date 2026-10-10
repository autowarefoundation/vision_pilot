"""The test scenarios are logical, and the driver serves VisionPilot in-process.

Plain config checks: no CARLA, no networks.
"""

from __future__ import annotations

from pathlib import Path

import pytest
from omegaconf import OmegaConf

import vision_pilot_test_scenarios as pkg
from vision_pilot_test_scenarios import CONF_DIR

SCENARIOS = sorted((CONF_DIR / "scenario" / "vision_pilot").glob("*.yaml"))


def test_the_three_scenarios_ship() -> None:
    assert [p.stem for p in SCENARIOS] == ["cut_in", "intersection_straight", "pedestrian_dart_out"]


def _lanelet_ids(node: object, prefix: str = "") -> dict[str, object]:
    """Every ``*lanelet_id(s)`` value in a raw config outside ``sweep``, by dotted key."""
    found: dict[str, object] = {}
    if isinstance(node, dict):
        for key, value in node.items():
            path = f"{prefix}{key}"
            if key == "sweep":
                continue
            if str(key).endswith(("lanelet_id", "lanelet_ids")):
                found[path] = value
            found.update(_lanelet_ids(value, path + "."))
    return found


@pytest.mark.parametrize("path", SCENARIOS, ids=lambda p: p.stem)
def test_every_lanelet_comes_from_a_constraint(path: Path) -> None:
    """No id is written down: each is left unset (``???``) for the sweep, which
    constrains the ego's lanelet and binds the rest from it -- so the scenario
    runs on any map."""
    raw = OmegaConf.to_container(OmegaConf.load(path), resolve=False)
    assert isinstance(raw, dict)
    swept = set(raw["sweep"]["constraints"]) | set(raw["sweep"].get("bindings", {}))
    assert "ego.spawn_lanelet_id" in raw["sweep"]["constraints"]
    for key, value in _lanelet_ids(raw).items():
        assert value == "???", f"{path.name} names {key}={value} outright"
        assert key in swept, f"{path.name}: nothing fills {key}"


def test_the_driver_serves_vision_pilot_in_process() -> None:
    raw = OmegaConf.load(CONF_DIR / "driver" / "vision_pilot_in_process.yaml")
    # On the framework's VisionPilot rig (vehicle, camera, 10 Hz).
    assert raw.defaults[0] == "vision_pilot"
    assert raw.driver.policy == "vision_pilot_test_scenarios:vision_pilot_policy"
    module, _, attribute = raw.driver.policy.partition(":")
    assert module == pkg.__name__ and callable(getattr(pkg, attribute))
    assert raw.driver.warmup_s > 0


def test_the_default_weights_are_the_checkouts() -> None:
    assert pkg._WORKSPACE_WEIGHTS == Path(__file__).resolve().parents[3] / "modules/models/weights"
