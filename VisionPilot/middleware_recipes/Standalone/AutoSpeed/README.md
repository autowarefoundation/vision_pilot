# AutoSpeed Standalone Inference

Multi-threaded inference pipeline with ONNX Runtime (CPU/TensorRT) for real-time object detection and tracking.

## Features

- **ONNX Runtime Backend**: Supports CPU and TensorRT execution providers
- **Multi-threaded Pipeline**: Separate threads for capture, inference, and display
- **Object Tracking**: Kalman filter-based tracking with cut-in detection
- **CIPO Detection**: Closest In-Path Object identification with distance/velocity estimation
- **GStreamer Support**: RTSP streams, USB cameras, video files

## Build

### Prerequisites

1. **ONNX Runtime** (GPU version with TensorRT support)
   - Download from: https://github.com/microsoft/onnxruntime/releases
   - Extract to: `/path/to/onnxruntime-linux-x64-gpu-X.X.X`

2. **cuDNN** (for TensorRT provider)
   - Install cuDNN matching your CUDA version
   - Typically: `/usr/lib/x86_64-linux-gnu/libcudnn.so.X`

3. **Dependencies**
   - OpenCV 4.x
   - GStreamer 1.0 (including `gstreamer-app-1.0`)
   - yaml-cpp

   **Fedora/RHEL:**
   ```bash
   sudo dnf install opencv-devel gstreamer1-devel gstreamer1-plugins-base-devel gstreamer1-plugins-good yaml-cpp-devel cmake gcc-c++
   ```

   **Debian/Ubuntu:**
   ```bash
   sudo apt install libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-good libyaml-cpp-dev cmake g++
   ```

### Build Steps

```bash
cd VisionPilot/Standalone/AutoSpeed
mkdir build && cd build

# Set ONNX Runtime path (REQUIRED)
export ONNXRUNTIME_ROOT=/path/to/onnxruntime-linux-x64-gpu-1.22.0

# Configure and build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

## Usage

### Quick Start (Shell Script)

Edit `run_objectFinder.sh` to configure:
The H matrix is in homography.yaml for waiymo/OpeLane dataset (front camera):

```bash
# ===== Required Parameters =====
VIDEO_PATH="/path/to/video.mp4"
MODEL_PATH="/path/to/model.onnx"
PROVIDER="tensorrt"       # 'cpu' or 'tensorrt'
PRECISION="fp16"          # 'fp32' or 'fp16' (TensorRT only)
HOMOGRAPHY_YAML="/path/to/homography.yaml"

# ===== ONNX Runtime Options =====
DEVICE_ID="0"             # GPU device ID
CACHE_DIR="./trt_cache"   # TensorRT engine cache directory

# ===== Pipeline Options =====
REALTIME="true"           # Real-time playback
MEASURE_LATENCY="false"   # Enable latency metrics
ENABLE_VIZ="true"         # Enable visualization
SAVE_VIDEO="false"        # Save output video
OUTPUT_VIDEO="output.mp4"
```

Run:
```bash
export ONNXRUNTIME_ROOT=/path/to/onnxruntime-linux-x64-gpu-1.22.0
bash run_objectFinder.sh
```

### Direct Execution

```bash
export ONNXRUNTIME_ROOT=/path/to/onnxruntime-linux-x64-gpu-1.22.0

./build/autospeed_infer_stream \
    <video_source> \
    <model.onnx> \
    <provider> \
    <precision> \
    <homography.yaml> \
    [device_id] \
    [cache_dir] \
    [realtime] \
    [measure_latency] \
    [enable_viz] \
    [save_video] \
    [output_video]
```

Example:
```bash
./build/autospeed_infer_stream \
    video.mp4 \
    model.onnx \
    tensorrt \
    fp16 \
    homography.yaml \
    0 \
    ./trt_cache \
    true \
    false \
    true \
    false \
    output.mp4
```

## Configuration

### Execution Providers

- **CPU**: Uses ONNX Runtime CPU provider (MLAS)
  - No GPU required
  - Slower (~50ms per frame)

- **TensorRT**: Uses TensorRT execution provider
  - Requires NVIDIA GPU with CUDA/cuDNN
  - Fast (~3-5ms per frame with FP16)
  - Engine cached for fast subsequent runs

### TensorRT Cache

- First run: Builds and caches engine (~30 seconds)
- Subsequent runs: Loads from cache (<1 second)
- Cache location: `./trt_cache/onnxrt_fp16_*.engine` (or `fp32_*`)

### Precision Options

- **FP32**: Full precision (slower, higher accuracy)
- **FP16**: Half precision (2x faster, minimal accuracy loss)

## Output

- **Console**: Main CIPO status (track ID, distance, velocity)
- **Visualization**: Bounding boxes, distance, speed, cut-in warnings
- **Video**: Optional output video with annotations

## Controls

- Press `q` in video window to quit (if visualization enabled)

## Troubleshooting

### TensorRT Provider Fails

1. **Check cuDNN**: Ensure `libcudnn.so` is in library path
   ```bash
   export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/x86_64-linux-gnu
   ```

2. **Clear Cache**: If switching precision, clear old cache
   ```bash
   rm -rf ./trt_cache/*.engine
   ```

3. **Use CPU Fallback**: Set `PROVIDER="cpu"` in script

### ONNX Runtime Not Found

```bash
# Set ONNXRUNTIME_ROOT before building/running
export ONNXRUNTIME_ROOT=/path/to/onnxruntime-linux-x64-gpu-1.22.0
```

