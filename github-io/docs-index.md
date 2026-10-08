---
layout: doc
title: Documentation
permalink: /github-io/
lede: Everything you need to build, run, configure and contribute to Vision Pilot - the single source of truth for the project.
description: Documentation index for Vision Pilot, the Autoware Foundation's open-source L2 ADAS stack.
---

Vision Pilot is a free and fully open-source **L2 ADAS** system. It is designed to be integrated
by automotive OEMs and Tier-1 suppliers into series-production passenger cars, and can optionally
be adopted for transportation and logistics use cases in buses and trucks.

The complete codebase - **including AI model weights** - is available under the permissive
Apache 2.0 licence, for commercial and research use alike.

## New here? Start with these

<div class="grid grid-2">
  <a class="card" href="{{ '/github-io/getting-started/' | relative_url }}">
    <span class="card-tag">1 · Install</span>
    <h3>Getting started →</h3>
    <p>Build from source, install the Debian package, or run the Docker image - then replay the sample dataset.</p>
  </a>
  <a class="card" href="{{ '/github-io/architecture/' | relative_url }}">
    <span class="card-tag">2 · Understand</span>
    <h3>Architecture →</h3>
    <p>How camera frames become steering and throttle commands, and where the safety path sits.</p>
  </a>
  <a class="card" href="{{ '/github-io/configuration/' | relative_url }}">
    <span class="card-tag">3 · Configure</span>
    <h3>Configuration →</h3>
    <p>Every key in the three <code>.conf</code> files, with defaults and valid values.</p>
  </a>
  <a class="card" href="{{ '/github-io/hardware/' | relative_url }}">
    <span class="card-tag">4 · Go live</span>
    <h3>Hardware and calibration →</h3>
    <p>Choosing a camera, mounting it, and computing the homography that Vision Pilot needs.</p>
  </a>
</div>

## What Vision Pilot does

Vision Pilot supports the entry-level L2 feature set for in-lane autonomous driving:

| Feature | Meaning | Control axis |
| --- | --- | --- |
| **ACC** | Autonomous cruise control | Longitudinal |
| **FCW** | Forward collision warning | Warning only |
| **AEB** | Autonomous emergency braking | Longitudinal |
| **LKAS** | Lane keep assist | Lateral |
| **ALK** | Autonomous lane keep | Lateral |
| **LDW** | Lane departure warning | Warning only |
| **LDA** | Lane departure avoidance | Lateral |
| **ISA** | Intelligent speed assist (map-based) | Longitudinal |
| **Autopilot** | Single-lane hands-free highway autopilot | Both |

<div class="note warn" markdown="1">
**Hands-free autopilot has a defined operational domain.** It is available only on highways and
motorways with clearly marked lane lines, in fair weather and visibility, across the full range of
highway driving speeds (0–70 mph), and away from construction zones and roadwork objects.
**A human driver is required to monitor and supervise the system at all times.**
</div>

## What it needs

**Sensor specification.** A single, front-facing, monocular RGB camera with a **50–55° horizontal
field of view** at **2 MP** resolution. That is the whole sensor set. See
[hardware and calibration]({{ '/github-io/hardware/' | relative_url }}) for mounting and camera selection.

**No HD maps.** Vision Pilot operates in a mapless mode and follows the road in real time. There
is no localisation stack to run and no 3D map to keep current. (Intelligent speed assist does use
speed-limit map data - that is a very different thing from an HD map.)

**Compute.** The stack is designed to run at **10 Hz** within a budget of roughly
**3–5 INT8 TOPs**. It runs on CPU via ONNX Runtime, or on NVIDIA GPUs via the CUDA/TensorRT
execution provider - see [configuration]({{ '/github-io/configuration/' | relative_url }}).

<div class="note tip" markdown="1">
For a 10-minute overview of the project's goals and design, see the
[introductory presentation]({{ site.links.presentation }}).
</div>

## Map of the repository

| Path | What lives there |
| --- | --- |
| `VisionPilot/app/` | The `VisionPilot` executable entry point |
| `VisionPilot/modules/` | Sensing, engine, models, safety guardian, visualization, logging |
| `VisionPilot/config/` | `vision_pilot.conf` and friends, `H.yaml`, `vehicle.dbc` |
| `VisionPilot/docker/` | `build.sh`, `run.sh`, GPU and CPU Dockerfiles |
| `Calibration/` | Homography calibration script and printable checkerboard |
| `Sensing/` | Camera selection and mounting guide |
| `Simulation/` | CARLA (ROS 2 and Zenoh) and SODA.Sim integrations |
| `Functional_Safety/` | Safety plan, SEooC scope, software requirements, safety metrics |
| `github-io/` | This documentation site |

## Related projects

The three AI models Vision Pilot runs are developed and released separately by the Autoware
Foundation:

- [**AutoSpeed**]({{ site.links.auto_speed }}) - closest in-path object detection
- [**AutoSteer**]({{ site.links.auto_steer }}) - ego path future waypoint detection
- [**AutoDrive**]({{ site.links.auto_drive }}) - end-to-end distance, in-path object presence and road curvature

## Getting help

- **Discord** - the [`privately owned vehicles` channel]({{ site.links.discord_channel }}) is the
  fastest route to a person. [Join the Autoware Discord]({{ site.links.discord }}).
- **Issues** - bugs and feature requests belong in the
  [issue tracker]({{ site.links.issues }}).
- **Working group meetings** - weekly, two slots, agenda and recordings are
  [public]({{ site.links.discussions }}).
