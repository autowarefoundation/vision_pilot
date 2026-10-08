---
layout: doc
title: Configuration
permalink: /github-io/configuration/
lede: Vision Pilot reads three plain-text config files at startup. This page documents every key in them.
description: Reference for vision_pilot.conf, vision_pilot_test.conf and vision_pilot_ros2.conf.
---

Configuration lives in `VisionPilot/config/`, or in `/usr/share/visionpilot/config` for a Debian
install. The files are `key = value` text; `#` starts a comment.

| File | Read when | Purpose |
| --- | --- | --- |
| `vision_pilot.conf` | Always | Input mode, inference engine, vehicle parameters, GUI |
| `vision_pilot_test.conf` | `source.mode = video` | Recorded-video playback settings |
| `vision_pilot_ros2.conf` | `source.mode = ros2` | ROS 2 topic names |

Two more files sit alongside them:

- **`H.yaml`** - the camera homography matrix. Replace it with your own after
  [calibration]({{ '/github-io/hardware/' | relative_url }}); keep a copy of the original so you can
  still run the sample data.
- **`vehicle.dbc`** - the CAN database used by the vehicle interface.

<div class="note warn" markdown="1">
Config is read at **startup**, and for source builds it is copied into the build directory, and
for Docker into the image. Edit it *before* you build, or rebuild after editing.
</div>

## `vision_pilot.conf`

### Input source

```ini
source.mode        = video

source.v4l2_device = /dev/video0
source.v4l2_fps    = 10
```

| Key | Values | Meaning |
| --- | --- | --- |
| `source.mode` | `video`, `v4l2`, `ros2` | Where frames come from. `video` replays a file, `v4l2` reads a camera device, `ros2` subscribes to an image topic |
| `source.v4l2_device` | path | V4L2 device node, used when `source.mode = v4l2` |
| `source.v4l2_fps` | integer | Target capture rate. 10 Hz is the design cadence |

<div class="note tip" markdown="1">
`source.mode = ros2` requires a build configured with `-DENABLE_ROS2_INTERFACE=ON`.
</div>

### Inference engine

```ini
engine.provider     = cuda
engine.device_id    = 0
engine.cache_dir    = /tmp/visionpilot_trt_cache
engine.workspace_gb = 1.0

model.precision     = fp32
```

| Key | Values | Meaning |
| --- | --- | --- |
| `engine.provider` | `cuda`, `cpu` | ONNX Runtime execution provider. Must match the ONNX Runtime build you linked against |
| `engine.device_id` | integer | GPU index, for multi-GPU machines |
| `engine.cache_dir` | path | Where the TensorRT engine cache is written. First run is slow while engines are built; later runs reuse this |
| `engine.workspace_gb` | float | TensorRT workspace budget, in GiB |
| `model.precision` | `fp32`, `int8` | Weight precision. `int8` selects the quantized models - lower latency and memory, at a small accuracy cost. Lowercase only |

### Vehicle parameters

```ini
speed_limit = 33.3;   # m/s
L           = 2.860   # front axle to CoG (m)
```

| Key | Units | Meaning |
| --- | --- | --- |
| `speed_limit` | m/s | Upper bound on commanded speed. 33.3 m/s ≈ 120 km/h |
| `L` | m | Wheelbase used by the lateral controller, where `L = Lf + Lr` |

<div class="note warn" markdown="1">
`L` is vehicle-specific. Leaving it at the default while driving a different vehicle will bias
every steering command the stack produces.
</div>

### GUI, streaming and logging

```ini
visualization_on = true

webrtc_on   = false
webrtc_port = 8080

rrd_on  = false
rrd_log = visionpilot.rrd
```

| Key | Values | Meaning |
| --- | --- | --- |
| `visualization_on` | `true`, `false` | Draw the local OpenCV HUD window |
| `webrtc_on` | `true`, `false` | Stream the rendered HUD to a browser over WebRTC. Disables the local preview while active |
| `webrtc_port` | port | Port the WebRTC signalling server listens on. Open `http://127.0.0.1:<port>/` to watch |
| `rrd_on` | `true`, `false` | Log per-frame data to a [Rerun](https://rerun.io) recording |
| `rrd_log` | path | Output `.rrd` file. Open it with `rerun <file>` |

## `vision_pilot_test.conf`

Read only when `source.mode = video`.

```ini
source.video_realtime      = true
source.video_loop          = false

source.input_video         = /path/to/input.mp4
source.input_vehicle_speed = /path/to/frame_speed.txt
source.dataset             = open_lane
```

| Key | Values | Meaning |
| --- | --- | --- |
| `source.video_realtime` | `true`, `false` | Play back at wall-clock rate rather than as fast as frames can be processed |
| `source.video_loop` | `true`, `false` | Restart the sequence when it ends |
| `source.input_video` | path | The recorded video to replay |
| `source.input_vehicle_speed` | path | Per-frame ego speed log that accompanies the video |
| `source.dataset` | `open_lane`, `zod` | Which dataset convention the speed log follows |

Sample sequences for both datasets are on
[Google Drive]({{ site.links.sample_data }}).

## `vision_pilot_ros2.conf`

Read only when `source.mode = ros2`, and only meaningful in a build configured with
`-DENABLE_ROS2_INTERFACE=ON`.

```ini
source.input_camera_topic  = /camera/image

vehicle_speed_topic        = /vehicle/speed
vehicle_steering_topic     = /vehicle/steering_cmd
vehicle_acceleration_topic = /vehicle/throttle_cmd
```

| Key | Direction | Meaning |
| --- | --- | --- |
| `source.input_camera_topic` | subscribe | `sensor_msgs/Image` source of camera frames |
| `vehicle_speed_topic` | subscribe | Current ego speed |
| `vehicle_steering_topic` | publish | Lateral command out of the planner |
| `vehicle_acceleration_topic` | publish | Longitudinal command out of the planner |

For CARLA, set the camera topic to the bridge's output:

```ini
source.input_camera_topic = /carla/hero/main_cam/image
```

See the [simulation guide]({{ '/github-io/simulation/' | relative_url }}) for the full closed-loop setup.

## Common configurations

**Replay the sample data on a GPU machine**

```ini
# vision_pilot.conf
source.mode      = video
engine.provider  = cuda
visualization_on = true
```

**Live camera, no GPU, headless with a browser view**

```ini
source.mode        = v4l2
source.v4l2_device = /dev/video0
engine.provider    = cpu
visualization_on   = false
webrtc_on          = true
webrtc_port        = 8080
```

**CARLA closed loop with Rerun logging**

```ini
source.mode     = ros2
engine.provider = cuda
rrd_on          = true
rrd_log         = carla_run.rrd
```
