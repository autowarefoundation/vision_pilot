---
layout: doc
title: Simulation
permalink: /docs/simulation/
lede: Close the loop in CARLA - the same VisionPilot binary, steering and braking a virtual vehicle through a ROS 2 bridge.
description: Running Vision Pilot closed-loop in the CARLA simulator via the ROS 2 bridge, plus Zenoh and SODA.Sim support status.
---

Open-loop replay tells you what the stack _perceives_. Closed-loop simulation tells you how it
_behaves_ when its own commands move the vehicle - which is the part that actually matters.

| Simulator              | Transport | Status                                     |
| ---------------------- | --------- | ------------------------------------------ |
| **CARLA 0.9.16** (UE4) | ROS 2     | Fully supported - preferred                |
| CARLA 1.0 (UE5)        | ROS 2     | Supported - see [CARLA 1.0](#carla-10-ue5) |
| CARLA 0.9.16           | Zenoh     | Experimental                               |
| SODA.Sim               | -         | Coming soon                                |

Steps 1-6 cover CARLA 0.9.16 + ROS 2; [CARLA 1.0](#carla-10-ue5) lists what changes. The
others live under
[`Simulation/`]({{ site.links.repo }}/tree/main/Simulation) in the repository.

## 1. Install CARLA (0.9.16)

Steps 1-6 target **CARLA 0.9.16 with Unreal Engine 4**. Follow the
[official quickstart](https://carla-ue5.readthedocs.io/en/latest/start_quickstart/#) for binaries
and dependencies, and the [CARLA docs](https://carla.readthedocs.io/en/latest/) for everything else.

<div class="note" markdown="1">
**6 GB of VRAM or less?** Apply the modifications in
[this gist](https://gist.github.com/xmfcx/a5e32fdecfcd85c6cc9d472ce7a3a98d) to run CARLA in Docker
with lower VRAM requirements. Tested on a laptop RTX 3060.
</div>

## 2. Run the simulator (0.9.16)

Change the `--volume` path to wherever you downloaded CARLA:

```sh
docker run -it --rm \
  --runtime=nvidia \
  --net=host \
  --env=DISPLAY=$DISPLAY \
  --env=NVIDIA_VISIBLE_DEVICES=all \
  --env=NVIDIA_DRIVER_CAPABILITIES=all \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$HOME/Downloads/carla/CARLA_0.9.16/:/home/carla/host-carla" \
  --workdir="/home/carla/host-carla" \
  carlasim/carla:0.9.16 \
  bash CarlaUE4.sh -nosound
```

Append `--ros2` to the `CarlaUE4.sh` invocation to use the native ROS 2 interface.

| Flag                         | Why                                               |
| ---------------------------- | ------------------------------------------------- |
| `--runtime=nvidia`           | GPU access                                        |
| `--net=host`                 | Host network stack, so the bridge can reach CARLA |
| `--env=DISPLAY` + X11 volume | Forward the CARLA window to your display          |
| `-nosound`                   | Avoids audio device issues in the container       |

## 3. Build Vision Pilot with ROS 2 support

```bash
cd vision_pilot/VisionPilot/build
cmake -DONNXRUNTIME_ROOT=<ONNX_RUNTIME_ROOT_PATH> -DENABLE_ROS2_INTERFACE=ON ../
make -j$(nproc)
```

## 4. Configure

In `config/vision_pilot.conf`:

```ini
source.mode = ros2
```

In `config/vision_pilot_ros2.conf`, point at the bridge's camera topic:

```ini
source.input_camera_topic = /carla/hero/main_cam/image
```

Remember to configure **before** building - see [configuration]({{ '/docs/configuration/' | relative_url }}).

## 5. Build and run the CARLA bridge (0.9.16)

```bash
cd Simulation/CARLA/ROS2
colcon build
source ./install/setup.bash
```

```bash
ros2 launch carla_bridge_bringup carla_bridge.launch.py host:=<HOST> port:=<PORT> carla_version:=0.9.16
```

<div class="note" markdown="1">
`host` and `port` only need to be set if CARLA runs on a **different machine** to Vision Pilot.
Use the IP of the machine running CARLA and the port it publishes on.
</div>

`carla_version` (`0.9.16` or `1.0`) picks the rig JSON `config/carla_<version>.json` and its map
(`Town04` / `Town04_Opt`). Leave it out to detect the version from the server; `rig_file:=<path>`
overrides the JSON. It does not pick the `carla` Python module: that must match the server.

To load another map add `map:=<MAP>` (e.g. `map:=Town06`); without it the map comes from
`CARLA_MAP`, else the rig JSON `"map"` key. The map must exist on the server (1.0 uses the
`*_Opt` names, e.g. `Town06_Opt`).

## 6. Run Vision Pilot

From the `build` directory:

```bash
./VisionPilot
```

[![Vision Pilot driving in CARLA]({{ "/assets/img/Vision_Pilot_CARLA.jpg" | relative_url }})](https://drive.google.com/file/d/1DCtXkKnhGTcU-YRiBCTTbCYkixUw8FZW/view?usp=sharing)

_Vision Pilot closed-loop in CARLA - [watch the full video](https://drive.google.com/file/d/1DCtXkKnhGTcU-YRiBCTTbCYkixUw8FZW/view?usp=sharing)._

## CARLA 1.0 (UE5)

CARLA 1.0 is UE5-based, has improved native ROS 2 support and is compatible with ScenarioRunner:

- Sensors publish natively with `--ros2`.
- Vehicles subscribe Ackermann control directly via the `ros2_ackermann_control` blueprint attribute.
- RMW is selectable with `--rmw=fastdds|cyclonedds|zenoh`.

Vision Pilot build, config and topics are unchanged (steps 3, 4 and 6). Start order: server, spawn
script, bridge, Vision Pilot.

### Get the Town04_Opt map

`Town04_Opt` is part of the AdditionalMaps content pack, not the base `carlasim/carla:1.0` image.
Download the pack (~6.8 GB, ~7 GB extracted; keep ~15 GB free) and add it to the server.

**Docker:** build an image that includes the pack:

```sh
mkdir -p packs_dl/extracted && cd packs_dl
curl -L -C - -O https://downloads.carlasim.com/Linux/Dev/CARLA_UE5_Weekly_AdditionalMaps.tar.gz
tar -xzf CARLA_UE5_Weekly_AdditionalMaps.tar.gz -C extracted
printf 'FROM carlasim/carla:1.0\nCOPY --chmod=a+rX extracted/AdditionalMaps /workspace/CarlaUnreal/Packs/AdditionalMaps\n' > Dockerfile
docker build -t carlasim/carla:1.0-maps .
```

**Package install:** download `CARLA_UE5_Weekly.tar.gz` and
`CARLA_UE5_Weekly_AdditionalMaps.tar.gz` from <https://downloads.carlasim.com/Linux/Dev/>, then
install the pack into `<package>/CarlaUnreal/Packs/AdditionalMaps` and restart the server:

```sh
python3 <package>/Tools/carla_pack.py install CARLA_UE5_Weekly_AdditionalMaps.tar.gz --server <package>
```

Alternatives: `client.mount_content_pack('/path/to/extracted/pack')` at runtime, or the server
flag `-carla-packs=<dir>`. See [content packs](https://carla-ue5.readthedocs.io/en/latest/content_authoring_packs/)
and [map packs](https://carla-ue5.readthedocs.io/en/latest/map_content_packs/).

Gotchas:

- The pack must match the base release exactly; mismatches are refused.
- Do not install two packs that ship the same town.
- The server is slow to start: poll `client.get_available_maps()` until `Town04_Opt` appears.
- `load_world('Town04_Opt')` needs `client.set_timeout(180)`.

### Run the server

Run it in **Low** quality and **off-screen** to save GPU memory (0.9.16 runs windowed), using the
`carlasim/carla:1.0-maps` image built above:

```sh
docker run -d --name carla-server \
  --runtime=nvidia \
  --net=host \
  --user=$(id -u):$(id -g) \
  --env=DISPLAY=$DISPLAY \
  --env=NVIDIA_VISIBLE_DEVICES=all \
  --env=NVIDIA_DRIVER_CAPABILITIES=all \
  --env=FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
  carlasim/carla:1.0-maps \
  bash CarlaUnreal.sh -nosound -quality-level=Low -RenderOffScreen --ros2
```

| Flag                 | Why                                               |
| -------------------- | ------------------------------------------------- |
| `-quality-level=Low` | Lower GPU memory use                              |
| `-RenderOffScreen`   | No window; saves GPU memory                       |
| `--ros2`             | Required - publishes `/carla/hero/main_cam/image` |

### Spawn the ego

Use the CARLA 1.0 Python client matching the server (conda env or
`carla/dist/carla-1.0-cpXYZ-*.whl`); a version mismatch can segfault.

```bash
cd Simulation/CARLA/ROS2
ros2 launch carla_bridge_bringup carla_bridge.launch.py host:=<HOST> port:=<PORT> carla_version:=1.0
```

To run only the spawn script: `python src/carla_bridge_bringup/scripts/config_carla.py --carla-version 1.0`.
`config_carla.py` serves 0.9.16 and 1.0: `--carla-version` (or, without it, the server version) picks
`config/carla_1.0.json` or `config/carla_0.9.16.json`; `-f` overrides. The map comes from `-m` /
`CARLA_MAP`, else the JSON `"map"` key.

Differences from 0.9.16 (`carla_0.9.16.json`):

- Map `Town04_Opt` (plain `Town04` fails with `Map not found`); spawn index 100 is valid.
- Ego blueprint `vehicle.lincoln.mkz` (not `mkz_2020`); `vehicle.audi.a2` does not exist.
- `Sensor.enable_for_ros()` is removed: sensors publish natively, named via `ros_name` /
  `ros_topic_name` attributes.
- The ego sets `ros2_ackermann_control=true`, so it subscribes
  `/carla/hero/ackermann_control_cmd` (`AckermannDriveStamped`).

### Bridge

Same nodes (`carla_vehicle_speed_publisher`, `carla_control_publisher`), but `PYTHONPATH` must
contain the CARLA 1.0 cp312 wheel (unzip it into a directory) - the 0.9.16 wheel does not work.
In the dev image, `apt-get update && apt-get install -y ros-jazzy-ackermann-msgs` first.

### Stopping

Stop clients with Ctrl+C (SIGINT), never `kill -9`. A hard-killed client leaves orphaned sensor
streams (`Invalid session: no stream available` flood) and the server later segfaults; restart
the server after any hard kill.

### Expected result

The car drives `Town04_Opt` at 17-29 m/s with mean |cte| ~0.4 m. The camera publishes ~5 Hz
off-screen/Low, so lane error is larger than on 0.9.16.

## What to watch for

**Nothing renders, no frames arrive.** Confirm the bridge is publishing:

```bash
ros2 topic hz /carla/hero/main_cam/image
```

If it is silent, the bridge is not connected to CARLA - check `host` and `port`, and that CARLA
started with `--ros2`.

**Frames arrive but the vehicle does not move.** Vision Pilot publishes to
`vehicle_steering_topic` and `vehicle_acceleration_topic`. Verify with `ros2 topic echo`, and
check the bridge is subscribed to the same names.

**Camera geometry looks wrong.** The homography in `config/H.yaml` describes a specific camera
mounting. A simulated camera with a different FoV or pose needs its own calibration - see
[hardware and calibration]({{ '/docs/hardware/' | relative_url }}).

## Recording a run

Turn on Rerun logging to capture every frame of a closed-loop run for later inspection:

```ini
rrd_on  = true
rrd_log = carla_run.rrd
```

```bash
rerun carla_run.rrd
```
