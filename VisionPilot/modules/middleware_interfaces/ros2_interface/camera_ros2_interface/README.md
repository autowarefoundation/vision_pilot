# Camera Subscriber Module

## I. Overview

This camera subscriber module is a ROS2-based middleware component that does these following:

1. Subscribing and listening to an exposed ROS2 topic (for example, `sensor_msgs/image`) from any ROS2 image source (simulators like CARLA, or hardware cameras, rosbag playback, etc.).
2. Transforming received image messages into OpenCV format `cv::Mat` objects for processing.
3. Maintaining a thread-safe queue with time-critical buffer ti ensure low-latency processing.

This module is intended for ROS2-OpenCV image bridge used during closed-loop testing.

Other features:

- Stream status tracking which confirms whether the ROS2 image message stream has been started.
- Validation of received frames, whether they are available and have been read appropriately.
- Able to handle multiple encodings, supporting multiple image formats like RGB, Mono, etc.
- Also some nice statistics of subscription health (frame reception, drop, error metrics etc.).

## II. Architecture & module structure

### 1. Architecture

```

ROS2 publisher (camera, simulator, rosbag replay etc.)
            ↓
            ↓
    [ROS2 image topic]
            ↓
            ↓
ROS2ImageSubscriber (camera subscriber module)
            |
            |---- [is_stream_started] flag : stream active?
            |---- [is_valid_frame]    flag : frame valid?
            │
            |---- [cv_bridge conversion]
            │
            |---- [Thread-safe queue - size = 1 (time critical)]
            │
            |---- [Frame metadata tracking]
            ↓
            ↓
OpenCV cv::Mat stream
            ↓
            ↓
VisionPilot pipeline (E2E models inference and other processing)

```

### 2. Module structure

```

camera_ros2_interface/
├── CMakeLists.txt
├── README.md
├── include/
│   └── camera_ros2_interface/
│       └── camera_ros2_interface.hpp
└── src/
    └── camera_ros2_interface.cpp

```

## III. Build

### 1. Prerequisites

- ROS2 (tested on ROS2 Humble / Ubuntu 22.04).
    - `source /opt/ros/humble/setup.bash`
- Required packages:
    - `rclcpp`
    - `sensor_msgs`
    - `cv_bridge`
    - `OpenCV`

### 2. Steps

```bash

# 1. From root to release directory
cd VisionPilot/development_releases/1.0

# 2. Create build dir
mkdir -p build && cd build

# 3. Source ROS2 just in case you forgot to
source /opt/ros/humble/setup.bash

# 4. CMake configure
cmake ..

# 5. Compile
make -j$(nproc)

```

### 3. Expected output

```bash

[ 97%] Building CXX object app/CMakeFiles/VisionPilot.dir/vision_pilot.cpp.o
[100%] Linking CXX executable ../VisionPilot
[100%] Built target VisionPilot

```

## IV. Test

This module features ROS2 => OpenCV stream conversion, thus a ROS2 topic publishing images is required.

You can do a simple test using a local video by simply just:

1. Publishing it as ROS2 image messages via [`ros-<distro>-image-publisher](https://docs.ros.org/en/humble/p/image_publisher/).

```bash

# 1. Install package, assuming you are using ROS2 Humble
sudo apt update
sudo apt install ros-humble-image-publisher

# 2. Establish ROS2 topic (assuming the topic is `/camera/image`)
ros2 run image_publisher image_publisher_node --ros-args \
-p filename:="<video absolute path>" \
-p publish_rate:=30.0 \
-p frame_id:="camera_link" \
-r image_raw:=/camera/image

```

With this the topic `/camera/image` is now active and publishing frames from that local video, as ROS2 messages. You should 

2. Then, use the provided `camera_viewer_node` executable to visualize the `cv::Mat` received from the bridge.

```bash

# 1. From root to auxiliaries directory
cd VisionPilot/development_releases/auxiliaries

# 2. Create build dir
mkdir -p build && cd build

# 3. Source ROS2 just in case you forgot to
source /opt/ros/humble/setup.bash

# 4. CMake configure
cmake ..

# 5. Compile
make -j$(nproc)

# 6. Execute
./camera_viewer_node /camera/image 1

```