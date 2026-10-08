---
layout: doc
title: Modules
permalink: /github-io/modules/
lede: A tour of VisionPilot/modules/ - what each component does, what it depends on, and where its own README lives.
description: >-
  Reference for the Vision Pilot module tree - sensing, engine, models, safety guardian,
  visualization, logging and middleware interfaces.
---

Vision Pilot is a CMake project made of independent modules under `VisionPilot/modules/`, linked
together by the application entry point in `VisionPilot/app/vision_pilot.cpp`.

| Module | Path | Role |
| --- | --- | --- |
| Camera interface | `sensing/camera_interface` | V4L2 frame capture |
| Vehicle interface | `sensing/vehicle_interface` | CAN bus speed in, commands out |
| Image preprocessing | `sensing/image_preprocessing` | Frame => model tensor |
| Engine | `engine` | ONNX Runtime wrapper, providers and caching |
| Models | `models` | AutoSpeed / AutoSteer / AutoDrive integration |
| Fusion | `safety_guardian/fusion` | Reconciles perception and end-to-end outputs |
| Planning | `safety_guardian/planning` | Longitudinal and lateral commands |
| Visualization | `visualization` | HUD, Occupancy BEV, WebRTC streaming |
| Logging | `logging` | Rerun `.rrd` recording |
| ROS 2 interfaces | `middleware_interfaces/ros2_interface` | Camera and vehicle topic bridges |

## Sensing

### Camera interface (V4L2)

A C++ wrapper that captures frames from a V4L2 device (`/dev/video0`, …) and hands them downstream
as OpenCV `cv::Mat`. It uses `cv::VideoCapture` with the `CAP_V4L2` backend, guards frame and
statistics access with a mutex, lets you request a target FPS and codec, and tracks frames
captured, dropped and errored.

Configured by `source.v4l2_device` and `source.v4l2_fps`.

### Vehicle interface (CAN)

Reads ego speed from the vehicle CAN bus and writes control commands back, using the CAN database
in `config/vehicle.dbc`.

### Image preprocessing

Normalisation and geometric preparation of frames before inference, including the homography
mapping from `config/H.yaml`.

## Engine

The inference layer over ONNX Runtime. It selects the execution provider (`cuda` or `cpu`),
targets a specific GPU, manages the TensorRT engine cache directory and workspace budget, and
loads either `fp32` or quantized `int8` weights.

The first run with `cuda` is slow because TensorRT engines are built and written to
`engine.cache_dir`; subsequent runs reuse them.

## Models

Integration of the three Autoware Foundation models - pre- and post-processing, tensor layouts and
output decoding.

<div class="grid grid-3">
  <a class="card" href="{{ site.links.auto_speed }}"><span class="card-tag">Perception</span><h3>AutoSpeed ↗</h3><p>Closest in-path object detection and distance.</p></a>
  <a class="card" href="{{ site.links.auto_steer }}"><span class="card-tag">Perception</span><h3>AutoSteer ↗</h3><p>Ego path future waypoints.</p></a>
  <a class="card" href="{{ site.links.auto_drive }}"><span class="card-tag">End to end</span><h3>AutoDrive ↗</h3><p>Distance, in-path presence, road curvature.</p></a>
</div>

## Safety Guardian

### Fusion

Reconciles the perception branch and the end-to-end branch into a single scene interpretation.
Where the two disagree, the conservative reading is taken forward.

### Planning

Produces the longitudinal command (from the fused lead-object distance, ego speed and the
applicable speed limit) and the lateral command (tracking the fused ego path, using the wheelbase
`L`). This is where ACC, AEB, FCW, LKAS, LDW and ISA behaviours live.

## Visualization

The largest module, and the one with the most detailed
[README in the repository]({{ site.links.repo }}/blob/main/VisionPilot/modules/visualization/README.md).
Three capabilities:

### HUD rendering

`visualization::render_frame()` draws the driver-facing overlay on each frame - object boxes, the
closest in-path object with its distance, the fused path corridor, lane departure state and the
speed limit - and manages the local OpenCV window. Controlled by `visualization_on`.

### Occupancy BEV window (optional)

Built only with `-DENABLE_OCCUPANCY=ON`. A second window named **Occupancy** appears beside the
HUD showing a heuristic 3D bird's-eye view of the scene:

- light-grey ground plane with a 0–150 m grid, and a green fused-path corridor
- AutoSpeed objects as extruded boxes - blue for traffic, red for the CIPO, white for ego

Controls: **left-drag** orbit, **right-drag** pan, **wheel** zoom, <kbd>R</kbd> resets the camera.

<div class="note" markdown="1">
This is **not** a neural occupancy network. The geometry is derived from AutoSpeed detections, the
fused path, AutoSteer lanes and lateral cross-track error. Treat it as a debugging view, not a
perception output.
</div>

### WebRTC streaming (optional)

Streams the rendered HUD to any browser on the network, which is how you monitor a headless
vehicle PC. Enabled with `webrtc_on = true`.

The pipeline is `appsrc => queue => videoconvert => vp8enc => rtpvp8pay => webrtcbin` in GStreamer,
with a `libsoup` HTTP server that serves a self-contained HTML5 client at `/` and handles
SDP/ICE signalling over a WebSocket at `/ws`. No browser-side dependencies, no external signalling
server.

```bash
# with webrtc_on = true, webrtc_port = 8080
# then open:
http://127.0.0.1:8080/
```

The local OpenCV preview is disabled while WebRTC is active.

**Dependencies:** GStreamer (`base`, `bad` plugins), `libsoup2.4`, `json-glib`, OpenCV - all
covered by the [prerequisites]({{ '/github-io/getting-started/#prerequisites' | relative_url }}).

## Logging

Streams per-frame data directly into a [Rerun](https://rerun.io) `.rrd` recording, which you can
scrub through afterwards:

```ini
rrd_on  = true
rrd_log = visionpilot.rrd
```

```bash
rerun visionpilot.rrd
```

<div class="note" markdown="1">
Logging links the Rerun C++ SDK. A clean first build fetches and builds Arrow, so the initial
build needs network access and takes noticeably longer than usual.
</div>

## Middleware interfaces

Built with `-DENABLE_ROS2_INTERFACE=ON`.

- **`camera_ros2_interface`** - subscribes to `sensor_msgs/Image` on
  `source.input_camera_topic`.
- **`vehicle_ros2_interface`** - subscribes to the speed topic and publishes steering and
  acceleration commands.

This is the path the [CARLA bridge]({{ '/github-io/simulation/' | relative_url }}) uses.
