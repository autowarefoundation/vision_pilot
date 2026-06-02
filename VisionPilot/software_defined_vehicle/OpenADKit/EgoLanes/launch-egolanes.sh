#!/bin/bash

# Run the container
podman run -it --rm \
    --device nvidia.com/gpu=all \
    -p 6080:6080 \
    -v "$PWD"/model-weights:/autoware/model-weights:z \
    -v "$PWD"/../Test:/autoware/test:z \
    -e PYTHONPATH=/autoware:/autoware/Models \
    ghcr.io/autowarefoundation/visionpilot:latest \
    python3 /autoware/Models/visualizations/EgoLanes/video_visualization.py -v -p /autoware/model-weights/egolanes.pth -i /autoware/test/traffic-driving.mp4 -o /autoware/test/output_egolanes.avi
