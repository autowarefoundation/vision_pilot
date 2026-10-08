---
layout: doc
title: Getting started
permalink: /github-io/getting-started/
lede: Install Vision Pilot one of three ways, then replay a real driving sequence through the full pipeline in about fifteen minutes.
description: Build Vision Pilot from source, install the Debian package or run the Docker image, then run it on the sample OpenLane dataset.
---

There are three ways to get a running `VisionPilot` binary. Pick one:

| | Best for | Trade-off |
| --- | --- | --- |
| [**Build from source**](#option-1--build-from-source) | Development, modifying the stack | You install the dependencies yourself |
| [**Debian package**](#option-2--debian-package) | A fresh machine, quick evaluation | Pre-built; CUDA deps come with it |
| [**Docker**](#option-3--docker) | Reproducibility, CI, trying variants | Config is baked in at image build time |

All three run the same application and read the same configuration files.

## Prerequisites

Vision Pilot is developed and tested on **Ubuntu 22.04** with **ROS 2 Humble** (ROS 2 is optional -
it is only needed for the ROS 2 input interface and simulator bridges).

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake pkg-config \
  libopencv-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  libgstreamer-plugins-bad1.0-dev \
  libsoup2.4-dev libjson-glib-dev
```

You also need **ONNX Runtime**. Download a release for your platform from the
[ONNX Runtime releases page]({{ site.links.onnxruntime_releases }}) and extract it - the path you
extract it to is `<ONNX_RUNTIME_ROOT_PATH>` below.

<div class="note" markdown="1">
Pick the **GPU** ONNX Runtime build if you intend to run with `engine.provider = cuda`, and the
CPU build otherwise. Mixing them is the single most common cause of a clean build that fails at
model load time.
</div>

## Option 1 — Build from source

```bash
git clone https://github.com/autowarefoundation/vision_pilot.git
cd vision_pilot/VisionPilot
mkdir build && cd build
```

Configure. The baseline build:

```bash
cmake -DONNXRUNTIME_ROOT=<ONNX_RUNTIME_ROOT_PATH> ../
```

With the ROS 2 interface, so Vision Pilot can subscribe to a camera topic:

```bash
cmake -DONNXRUNTIME_ROOT=<ONNX_RUNTIME_ROOT_PATH> -DENABLE_ROS2_INTERFACE=ON ../
```

With the optional **Occupancy** bird's-eye window next to the HUD (off by default):

```bash
cmake -DONNXRUNTIME_ROOT=<ONNX_RUNTIME_ROOT_PATH> -DENABLE_OCCUPANCY=ON ../
```

Then build:

```bash
make -j$(nproc)
```

The `VisionPilot` executable is written to the `build` directory.

### CMake options

| Option | Default | Effect |
| --- | --- | --- |
| `ONNXRUNTIME_ROOT` | - | **Required.** Path to the extracted ONNX Runtime |
| `ENABLE_ROS2_INTERFACE` | `OFF` | Build the ROS 2 camera/vehicle interfaces |
| `ENABLE_OCCUPANCY` | `OFF` | Build the Occupancy BEV visualisation window |
| `GPU` | `ON` | Link the GPU execution provider; set `OFF` for a CPU-only package |

<div class="note warn" markdown="1">
**Configure before you build.** When building from source, the config files under
`VisionPilot/config/` are copied into the build. Edit them *first*, or re-run the build after
changing them.
</div>

### Build a Debian package

From inside the `build` directory:

```bash
cpack -G DEB
```

For a CPU-only package, configure with `-DGPU=OFF` before `make`, then run `cpack -G DEB`.

## Option 2 — Debian package

Recommended for a new system where the CUDA dependencies are not yet installed - the package
pulls them in.

Download the prebuilt binary from the [releases page]({{ site.links.releases }}), then:

```bash
sudo apt install ./VisionPilot-1.0-x86_64.deb
```

Reboot so the CUDA dependencies are picked up, then run it from anywhere:

```bash
VisionPilot
```

For a package install, the config files live in:

```
/usr/share/visionpilot/config
```

## Option 3 — Docker

The Dockerfiles in `VisionPilot/docker/` build GPU or CPU images, with or without ROS 2, and with
the optional Occupancy window.

```bash
cd VisionPilot/docker

./build.sh --gpu --ros2          # GPU + ROS 2
./build.sh --cpu                 # CPU only
./build.sh --gpu --occupancy     # add the Occupancy BEV window
```

Run it:

```bash
./run.sh --cpu
./run.sh --gpu --ros2
```

To mount your own data directory into the container:

```bash
./run.sh --gpu --data <HOST_DIR>:<CONTAINER_DIR>
```

<div class="note warn" markdown="1">
`<CONTAINER_DIR>` must match the path you used when the image was built, and **config files must
be edited before `build.sh`** - they are copied into the image. For a CPU image set
`engine.provider = cpu` in `config/vision_pilot.conf`; for a ROS 2 image set `source.mode = ros2`.
</div>

## Run on the sample dataset

The quickest way to see the whole pipeline work is open-loop scenario testing against a recorded
sequence.

### 1. Download the sample data

Grab a sequence from the [sample data folder on Google Drive]({{ site.links.sample_data }}). Each
directory contains a video assembled from a dataset's image sequence, plus the matching per-frame
vehicle speed log.

### 2. Point the config at it

In `config/vision_pilot.conf`:

```ini
source.mode = video
```

In `config/vision_pilot_test.conf`:

```ini
source.input_video         = <INPUT_VIDEO_FILE_PATH>
source.input_vehicle_speed = <INPUT_VEHICLE_SPEED_FILE_PATH>
source.dataset             = open_lane
```

The full key reference is on the [configuration page]({{ '/github-io/configuration/' | relative_url }}).

### 3. Run

From the `build` directory:

```bash
./VisionPilot
```

An OpenCV window opens with the driver-facing HUD: detected objects, the closest in-path object
and its distance, the estimated ego path, lane departure state and the speed limit. If you built
with `-DENABLE_OCCUPANCY=ON`, a second **Occupancy** window appears beside it - orbit with
left-drag, pan with right-drag, zoom with the wheel, and press <kbd>R</kbd> to reset the camera.

## Where to go next

- Run it **closed loop** in [CARLA]({{ '/github-io/simulation/' | relative_url }}) so the stack's own
  commands move the vehicle.
- Run it on **your own camera** - [hardware and calibration]({{ '/github-io/hardware/' | relative_url }}).
- Stream the HUD to a browser over WebRTC, or log every frame to Rerun - see
  [modules]({{ '/github-io/modules/' | relative_url }}).

## Troubleshooting

**Model fails to load, or CUDA errors on start.** The ONNX Runtime build does not match
`engine.provider`. Use the GPU ONNX Runtime for `cuda`, the CPU build for `cpu`.

**Black or blank video with WebRTC enabled.** Check the frames are arriving at all by disabling
WebRTC (`webrtc_on = false`) and using the local OpenCV preview. If the preview works, the problem
is WebRTC-specific.

**`Connection refused` on the WebRTC port.** Something else is on port 8080:

```bash
lsof -i :8080
```

**Config changes appear to do nothing.** For source builds, re-run the build. For Docker, rebuild
the image. For Debian installs, edit `/usr/share/visionpilot/config`, not the repo copy.

Still stuck? Ask in the [`privately owned vehicles` Discord channel]({{ site.links.discord_channel }})
or open an [issue]({{ site.links.issues }}).
