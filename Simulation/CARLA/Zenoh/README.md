# Zenoh

This directory provides a Zenoh-based CARLA bridge and related pipelines.

## Demo

* CARLA + Zenoh: Vision Pipelines Demo

[![CARLA + Zenoh Vision Pipelines Demo](https://img.youtube.com/vi/TQ6JwexlXvo/hqdefault.jpg)](https://youtu.be/TQ6JwexlXvo "CARLA + Zenoh: Vision Pipelines Demo")

## Dependencies

Please refer to the dependencies described in the following page:

[VisionPilot/Middleware_Recipes/Zenoh](../../../../VisionPilot/Middleware_Recipes/Zenoh)

## Usage

### Setup (run once)

```sh
just setup
```

### Build

```sh
# Build all
export LIBTORCH_INSTALL_ROOT=/path/to/libtorch/
export ONNXRUNTIME_ROOTDIR=/path/to/onnxruntime-linux-x64-gpu-1.22.0
just build
# Optional (build components separately)
just build_bridge
just build_video_pubsub
just build_models
```

### Run

#### Start CARLA server

```sh
just run_carla
```

Start the CARLA simulator.
The Docker image may take a long time to download on the first run.

#### Start Zenoh CARLA bridge

```sh
just run_zenoh
```

Start the CARLA Python agent with pygame control and also launch the Zenoh CARLA bridge.

#### Run pipelines

```sh
# Raw camera view
just run_carla_sub

# SceneSeg
just run_carla_sceneseg

# DomainSeg
just run_carla_domainseg

# Scene3D
just run_carla_scene3d

# Egolanes
just run_carla_egolanes
```

### Cleanup

```sh
just clean
```

