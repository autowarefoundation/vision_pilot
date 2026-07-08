# VisionPilot ⇄ CARLA (0.9.16 / 0.10) — ROS 2

Drive a CARLA ego **camera-only** with the single-binary VisionPilot closed loop
(perception → planning → control → actuation) over CARLA's **native `--ros2`**.
**VisionPilot itself is unmodified** — everything CARLA-specific lives here, and the
**same code drives both CARLA 0.9.16 (UE4) and 0.10 (UE5)**: their native ROS 2 layers
publish identical topic names, so only the ego blueprint, launcher and CARLA wheel differ.

**CARLA always runs on the HOST (from `$CARLA_ROOT`), never in Docker.** Only VisionPilot
and the control bridge run in the dev container (`autoware-visionpilot-dev:cuda`).

```text
HOST:  CARLA --ros2 (windowed)  +  ros_carla_config.py (spawn ego/camera — pure PythonAPI, no ROS)
CONTAINERS (one DDS domain — ROS never crosses the host/container boundary):
  /carla/hero/main_cam/image ── sensor_msgs/Image (CARLA native) ──►  VisionPilot (camera_subscriber)
  ego_telemetry (bridge ctr, PythonAPI over TCP:2000)
      ├─ /vehicle/speed (Float64 m/s) ──► VisionPilot planner + bridge watchdog
      └─ /carla_bridge/max_steer_angle (latched, rad) ──► bridge
  VisionPilot  (ENABLE_ROS2_INTERFACE=ON, source.mode=ros2)
      /vehicle/steering_cmd (Float64 rad)  ┐
      /vehicle/throttle_cmd (Float64 m/s²) ┴─►  carla_control_relay ─► /carla/hero/vehicle_control_cmd
                                                (feed-forward + watchdog + AEB)  (CarlaEgoVehicleControl)
```

**Control ownership:** VisionPilot closes the speed loop (its longitudinal planner consumes
`/vehicle/speed` + the configured `speed_limit`); the bridge is a stateless **feed-forward**
accel→pedal map — no PID, no target speed. The bridge watchdog holds the car (full brake)
whenever VisionPilot commands **or** the ego speed go stale, so a dead speed feed can never
cause runaway throttle. The lateral MPC is arc-length (distance-domain), i.e. speed-invariant —
feeding VisionPilot the real speed is safe at any `speed_limit`.

## What is shared vs version-specific

| Concern                                                  | Shared                                                                                    | CARLA 0.10.0 (UE5)                                                  | CARLA 0.9.16 (UE4)                                                         |
| -------------------------------------------------------- | ----------------------------------------------------------------------------------------- | ------------------------------------------------------------------- | -------------------------------------------------------------------------- |
| Ego/sensor rig (+ default spawn)                         | —                                                                                         | `config/VisionPilot_carla10.json` (`vehicle.lincoln.mkz`, spawn 63) | `config/VisionPilot_carla916.json` (`vehicle.lincoln.mkz_2020`, spawn 100) |
| Launcher (auto-detected by `drive.sh`)                   | —                                                                                         | `CarlaUnreal.sh -nosound --ros2`                                    | `CarlaUE4.sh -nosound --ros2`                                              |
| CARLA wheel (auto-staged by `drive.sh`)                  | —                                                                                         | 0.10 wheel from `$CARLA_ROOT`                                       | 0.9.16 wheel from `$CARLA_ROOT`                                            |
| ROS 2 topics                                             | `/carla/hero/main_cam/image`, `/carla/hero/vehicle_control_cmd`, `/vehicle/*` (identical) |                                                                     |                                                                            |
| Spawn helper / bridge / configs / homography / launchers | everything else in this dir                                                               |                                                                     |                                                                            |

## Layout

| Path                                                                  | Role                                                                                                                                                                                                       |
| --------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `drive.sh`                                                            | **one-command orchestrator**: `up` (full fresh bring-up) / `down` (teardown) / `status`                                                                                                                    |
| `ros_carla_config.py`                                                 | spawn `hero` ego + `main_cam` on the host (pure PythonAPI, **no ROS**); spawn point from the rig JSON (`SPAWN_INDEX` overrides)                                                                            |
| `src/visionpilot_carla_bridge/`                                       | `carla_control_relay` (Float64 steer/throttle → `CarlaEgoVehicleControl`; feed-forward, watchdog, AEB) + `ego_telemetry` (PythonAPI → `/vehicle/speed` + latched max_steer, in-container) — see its README |
| `src/carla_msgs/`                                                     | vendored `CarlaEgoVehicleControl.msg` (derived from carla-simulator/ros-carla-msgs, MIT)                                                                                                                   |
| `config/visionpilot.carla.conf`, `config/visionpilot_ros2.carla.conf` | VisionPilot run-config overlay (ros2 source, sim `speed_limit`/`Lf`, `/vehicle/*` topics)                                                                                                                  |
| `config/H_carla.yaml`                                                 | main_cam ground homography (camera → world)                                                                                                                                                                |
| `gen_carla_C_matrix.py`                                               | derive preprocess C from `H_carla.yaml` (reuses VisionPilot logic) → `config/homography_C_matrix.yaml` (gitignored)                                                                                        |
| `run_visionpilot.sh`                                                  | run VisionPilot in the dev container, bind-mounting the CARLA config over the paths the binary reads                                                                                                       |
| `build_bridge.sh`                                                     | colcon-build the bridge (`carla_msgs` + `visionpilot_carla_bridge`)                                                                                                                                        |
| `carla_speed_monitor.py`                                              | optional debug tool: ground-truth speed + collision printer (PythonAPI)                                                                                                                                    |

> **Camera FOV is 60°, not the model's ~42° view.** The homography warp reprojects the camera
> into the model view, sampling slightly _beyond_ the model frame at the edges; the camera must be
> **wider** than the model view so reprojection stays inside the captured image (a narrower fov reads
> out-of-frame → mirror/ghost artifact). The warp crops to the model view.

## Prerequisites (checked by `drive.sh`/`run_visionpilot.sh`, which fail fast)

1. CARLA installed on the host at `$CARLA_ROOT` (default `~/CARLA_0.9.16`), with a wheel under
   `PythonAPI/carla/dist` matching your `SPAWN_PYTHON` version.
2. A `SPAWN_PYTHON` whose version matches one of the CARLA wheels (cp310/311/312 for 0.9.16) —
   **no ROS needed on the host**. The wheel is staged onto `PYTHONPATH` automatically and
   shadows any other installed `carla` package (also staged for the container's python 3.12,
   which runs `ego_telemetry`).
3. The dev image `autoware-visionpilot-dev:cuda` (`Docker/build.sh`).
4. VisionPilot built with the ROS 2 interface, inside the container:
   `Docker/run.sh --test` (builds `VisionPilot/build_docker_ros2/`).
5. An X display for the CARLA + VisionPilot windows (default `:1`; live runs are always windowed).

## Run

```bash
./drive.sh up        # teardown, then: CARLA (host) → ego spawn (host) → bridge (container) → VisionPilot (container)
./drive.sh status
./drive.sh down      # ALWAYS run between experiments — stale actors/DDS topics corrupt the next run
```

Knobs (env): `CARLA_ROOT`, `RIG_JSON`, `SPAWN_PYTHON`, `SPAWN_INDEX`, `DISPLAY`, `IMAGE`.
Drive speed: `speed_limit` in `config/visionpilot.carla.conf` (m/s).

The CARLA run config is **bind-mounted** into the container over the fixed CWD-relative paths the
binary reads (`config/vision_pilot.conf`, `config/vision_pilot_ros2.conf`,
`config/homography_C_matrix.yaml`, and `H_carla.yaml` over `config/H.yaml` — without that last
mount AutoSteer fits 0 lane points); the tracked `VisionPilot/` tree is left pristine.

All ROS 2 processes run with `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` (host↔container Fast-DDS shared
memory does not connect reliably across process boundaries); containers use `--net=host`.

## Troubleshooting

- **VisionPilot window freezes** → the spawn helper died (it owns the ego + camera); check
  `/tmp/carla_spawn.log`, then `./drive.sh up` again.
- **AutoSteer fits 0 lane points** → the `H_carla.yaml → config/H.yaml` mount is missing
  (`run_visionpilot.sh` asserts it) or the camera rig was changed without regenerating `H_carla.yaml`.
- **Car does not move** → bridge watchdog is holding: VisionPilot not publishing yet, or no
  `/vehicle/speed` (check `ros2 topic hz /vehicle/speed` inside the container).
- **PythonAPI `std::bad_alloc` / segfault** → client/server version mismatch; `drive.sh` stages
  the wheel from `$CARLA_ROOT` and `ros_carla_config.py` warns when versions differ.

`docs/` holds the workflow diagram. Bridge unit tests: `python3 -m pytest src/visionpilot_carla_bridge/test/ -q`.
