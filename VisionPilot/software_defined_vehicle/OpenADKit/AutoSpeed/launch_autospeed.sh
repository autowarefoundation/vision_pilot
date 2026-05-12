#!/bin/bash

# Run the container
docker run -it --rm \
    -p 6080:6080 \
    -v "$PWD"/model-weights:/autoware/model-weights:z \
    -v "$PWD"/launch:/autoware/launch:z \
    -v "$PWD"/../Test:/autoware/test \
    ghcr.io/autowarefoundation/visionpilot:latest \
    /autoware/launch/run_objectFinder.sh
