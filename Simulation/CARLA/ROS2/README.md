# ROS2

## CARLA simulator

Run CARLA server with `--ros2` arg, to enable ROS2 integration.

```
docker run -it --rm \
  --runtime=nvidia \                        # Use NVIDIA runtime for GPU access
  --net=host \                              # Use the host's network stack (helps with networking/performance)
  --env=DISPLAY=$DISPLAY \                  # Pass the host's DISPLAY environment variable (for GUI forwarding)
  --env=NVIDIA_VISIBLE_DEVICES=all \        # Expose all GPUs to the container
  --env=NVIDIA_DRIVER_CAPABILITIES=all \    # Enable all driver capabilities (graphics, compute, etc.)
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \ # Mount X11 UNIX socket to enable GUI apps to display
  --volume="$HOME/Downloads/carla/CARLA_0.9.16/:/home/carla/host-carla" \ 
                                            # CHANGE AS NEEDED: Mount your local CARLA folder into the container
  --workdir="/home/carla/host-carla" \      # Set the working directory to the mounted CARLA folder
  carlasim/carla:0.9.16 \                   # Use the official CARLA Docker image, version 0.9.16
  bash CarlaUE4.sh -nosound --ros2          # Run the CARLA startup script with -nosound flag
```

## Scenario runner

Clone the ScenarioRunner repository.

```
git clone https://github.com/carla-simulator/scenario_runner.git
cd scenario_runner
python3 -m venv .venv 
pip3 install -r requirements.txt
export CARLA_ROOT=<CARLA ROOT directory>
export PYTHONPATH=$CARLA_ROOT/PythonAPI/carla
```

Run scenario for example

```
python3 ./scenario_runner.py --openscenario ./srunner/examples/LaneChangeSimple.xosc
```

## ROS2 bridge

Run VisionPilot ROS2 bridge

```
cd autoware.privately-owned-vehicles/VisionPilot/Simulation/CARLA/ROS2
git clone https://github.com/carla-simulator/ros-carla-msgs.git
colcon build
source install/setup.bash
ros2 launch carla_bridge_bringup carla_bridge.launch.py
```

## VisionPilot 0.5

Run VisionPilot 0.5

```
cd autoware.privately-owned-vehicles/VisionPilot/Middleware_Recipes/ROS2/VisionPilot_0.5
mkdir build && cd build
cmake ..
make
```

Update VisionPilot configuration

```
autoware.privately-owned-vehicles/VisionPilot/Middleware_Recipes/ROS2/VisionPilot_0.5/visionpilot.conf
```

Update TensorRT cache directory 

```
models.egolanes.cache_dir=/home/<USER>/.trt_cache
```

Update VisionPilot configuration for model directories

```
models.egolanes.path=/usr/share/visionpilot/models/EgoLanes_FP32.onnx
models.autosteer.path=/usr/share/visionpilot/models/AutoSteer_FP32.onnx
```

Update the default velocity if needed

```
velocity=50 # km/h
```

Run VisionPilot

```
./visionpilot ../visionpilot.conf
```

**_Note_**: Remove ctrl_shm in case not cleaned up

```
rm /dev/shm/ctrl_shm
```