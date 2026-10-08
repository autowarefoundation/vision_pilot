---
layout: doc
title: Architecture
permalink: /github-io/architecture/
lede: How a camera frame becomes a steering and throttle command - and why there are two AI paths running side by side.
description: The Vision Pilot hybrid end-to-end AI architecture, its three AI models, and the Safety Guardian that fuses them.
---

![Vision Pilot architecture: camera frames feeding parallel perception and end-to-end branches, fused by the Safety Guardian into longitudinal and lateral planning]({{ '/assets/img/VisionPilot_architecture.png' | relative_url }})

## Hybrid end-to-end AI

Most driving stacks pick a side. A classical **perception** pipeline is interpretable and easy to
argue about in a safety case, but brittle at the edges. A monolithic **end-to-end** model
generalises beautifully and fails opaquely.

Vision Pilot runs both, in parallel, on the same frame:

- The **perception path** produces explicit, inspectable quantities - object boxes, the closest
  in-path object, its distance, the ego path. This is the path a safety argument can be built on.
- The **end-to-end path** produces the same driving-relevant quantities directly from pixels,
  and is generally stronger in scenes the perception models find hard.

Neither output reaches the planner directly. The **Safety Guardian** fuses them, and the fused
result is what gets planned on. Where the two paths disagree, the conservative interpretation wins.

## The pipeline, stage by stage

### 1. Sensing

A single monocular camera frame enters through one of three interfaces - a recorded video file, a
V4L2 device, or a ROS 2 `sensor_msgs/Image` topic - selected by `source.mode`. Ego vehicle speed
arrives alongside it, from a speed log, a CAN bus, or a ROS 2 topic.

### 2. Preprocessing

Frames are normalised into the tensor layout the models expect. The **homography matrix**
`H.yaml` - produced by [calibration]({{ '/github-io/hardware/' | relative_url }}) - maps image
coordinates to flat road coordinates, which is what makes metric distance estimation from a single
camera possible at all.

### 3. Inference

The engine module wraps ONNX Runtime and runs the three models. It supports the CUDA/TensorRT
execution provider with an on-disk engine cache, and a CPU provider, plus `fp32` and quantized
`int8` weights.

<div class="grid grid-3">
  <a class="card" href="{{ site.links.auto_speed }}">
    <span class="card-tag">Perception</span>
    <h3>AutoSpeed ↗</h3>
    <p>Detects objects and identifies the <strong>closest in-path object</strong> (CIPO) - the one
       that governs longitudinal control - with a distance estimate.</p>
  </a>
  <a class="card" href="{{ site.links.auto_steer }}">
    <span class="card-tag">Perception</span>
    <h3>AutoSteer ↗</h3>
    <p>Predicts the <strong>ego path future waypoints</strong>: the corridor the vehicle should
       drive through, which is what lateral control follows.</p>
  </a>
  <a class="card" href="{{ site.links.auto_drive }}">
    <span class="card-tag">End to end</span>
    <h3>AutoDrive ↗</h3>
    <p>Estimates distance, in-path object presence and road curvature directly from the frame,
       in one shot.</p>
  </a>
</div>

All three are developed and released independently by the Autoware Foundation, with open weights.

### 4. Safety Guardian

Two submodules:

- **Fusion** reconciles the perception and end-to-end outputs into one consistent view of the
  scene - one lead object, one distance, one path.
- **Planning** turns that view into longitudinal and lateral commands: a target acceleration from
  the gap to the lead object and the applicable speed limit, and a steering command that tracks
  the fused path using the vehicle wheelbase `L`.

This is the layer that implements the L2 features. ACC and AEB are longitudinal behaviours over
the fused CIPO distance; LKAS is lateral tracking of the fused path; LDW and FCW are warning
states derived from the same quantities.

### 5. Output

Commands go out over the vehicle interface - CAN via `vehicle.dbc`, or ROS 2 topics. In parallel,
the visualization module renders the HUD, which can be shown locally, streamed to a browser over
WebRTC, or logged frame-by-frame to a Rerun recording.

## Design decisions worth knowing

### Mapless by construction

Vision Pilot does not require 3D high-definition maps. The road is followed in real time from the
current frame. There is no localisation stack, no map pipeline and no geofence - which removes an
entire class of operational cost, and an entire class of failure where the map and the world
disagree.

### One camera, on purpose

The sensor specification is a single front-facing RGB camera, 50–55° horizontal FoV, 2 MP. Wider
lenses are explicitly *not* suitable: they trade away the long-range detail that highway ADAS
features depend on. See [hardware]({{ '/github-io/hardware/' | relative_url }}).

### 10 Hz is a deliberate target

The stack is designed to run at 10 Hz. You can run it faster, but you do not need to: an average
human driver reacts at roughly 4 Hz, and an F1 driver at about 8 Hz. Ten hertz is already
super-human, and the headroom buys determinism.

### Certifiable, not certified

The codebase is written to be productionizable and safety-certifiable. The
[Functional Safety documentation]({{ '/github-io/functional-safety/' | relative_url }}) covers the
safety plan, the Safety Element out of Context scope, software requirements and safety metrics -
including the assumptions of use and known limitations that any integrator must read before
putting this on a road.

## Next

- [Modules]({{ '/github-io/modules/' | relative_url }}) - what each directory under
  `VisionPilot/modules/` actually contains
- [Configuration]({{ '/github-io/configuration/' | relative_url }}) - the knobs on each stage
