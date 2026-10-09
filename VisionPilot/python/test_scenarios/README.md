# vision-pilot-test-scenarios

VisionPilot's closed-loop test scenarios, as a scenario package for
[autoware_carla_scenario](https://github.com/autowarefoundation/autoware_carla_scenario)
(`autoware-carla-scenario` 4.x). A member of VisionPilot's uv workspace: it
registers itself with the `scenario` CLI by entry point and brings

- `scenario=vision_pilot/cut_in`, `vision_pilot/intersection_straight`,
  `vision_pilot/pedestrian_dart_out`: constraints on the map, no lanelet id
  (`???`, filled per case by `scenario-expand`), so they run on any map;
- `driver=vision_pilot_in_process`: VisionPilot served from the scenario's own
  process (`driver.policy`), on the framework's `driver=vision_pilot` rig, with a
  3 s run-up.

```bash
cd VisionPilot
uv sync --package vision-pilot-test-scenarios
uv run scenario-setup                      # once: downloads CARLA
for s in cut_in intersection_straight pedestrian_dart_out; do
  case=$(uv run scenario-expand scenario=vision_pilot/$s map=town10hd_opt \
    | python -c 'import json, sys; print(" ".join(json.load(sys.stdin)["cases"][0]))')
  uv run scenario scenario=vision_pilot/$s map=town10hd_opt $case driver=vision_pilot_in_process
done
```

`VISIONPILOT_MODEL_DIR` points at other weights (default: the checkout's
`modules/models/weights`), `VISIONPILOT_PROVIDER` picks the ONNX Runtime
provider (`cpu`, `cuda` or `tensorrt`; default `cpu`).
