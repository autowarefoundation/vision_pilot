# CARLA Simulator
## Version 0.10.0

CARLA 0.10.0 with Unreal Engine 5: https://carla-ue5.readthedocs.io/en/latest/#

![](../Media/carla_ros.gif)

### Installation
1. Download binaries and dependencies following official documentation: https://carla-ue5.readthedocs.io/en/latest/start_quickstart/

2. Follow the modifications specified in https://gist.github.com/xmfcx/a5e32fdecfcd85c6cc9d472ce7a3a98d to run CARLA with lower VRAM requirements using docker (tested with RTX3060 laptop version)

### How to run
Change the file path below to where CARLA is downloaded and run
```sh
docker run -it --rm \
  --runtime=nvidia \                        # Use NVIDIA runtime for GPU access
  --net=host \                              # Use the host's network stack (helps with networking/performance)
  --env=DISPLAY=$DISPLAY \                  # Pass the host's DISPLAY environment variable (for GUI forwarding)
  --env=NVIDIA_VISIBLE_DEVICES=all \        # Expose all GPUs to the container
  --env=NVIDIA_DRIVER_CAPABILITIES=all \    # Enable all driver capabilities (graphics, compute, etc.)
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \ # Mount X11 UNIX socket to enable GUI apps to display
  --volume="$HOME/Downloads/carla/Carla-0.10.0-Linux-Shipping/:/home/carla/host-carla" \ 
                                            # CHANGE AS NEEDED: Mount your local CARLA folder into the container
  --workdir="/home/carla/host-carla" \      # Set the working directory to the mounted CARLA folder
  carlasim/carla:0.10.0 \                   # Use the official CARLA Docker image, version 0.10.0
  bash CarlaUnreal.sh -nosound              # Run the CARLA startup script with -nosound flag
```
To run with ROS2 native interface, add `--ros2` at the end


## Version 0.9.16

CARLA 0.9.16 with Unreal Engine 4: https://carla.readthedocs.io/en/latest/

### Installation
1. Download binaries and dependencies following official documentation: https://carla-ue5.readthedocs.io/en/latest/start_quickstart/# 

2. Follow the modifications specified in https://gist.github.com/xmfcx/a5e32fdecfcd85c6cc9d472ce7a3a98d to run CARLA with lower VRAM requirements using docker (tested with RTX3060 laptop version)

### How to run
Change the file path below to where CARLA is downloaded and run
```sh
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
  bash CarlaUE4.sh -nosound                 # Run the CARLA startup script with -nosound flag
```
To run with ROS2 native interface, add `--ros2` at the end


