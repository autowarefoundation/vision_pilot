## SodaSim ROS 2 GStreamer Bridge

Standalone ROS 2 package that rebroadcasts the Soda.Sim ROS 2 camera topic (default: `/vehicle/camera`) into a GStreamer pipeline.

Two output modes are supported:

| Mode | Use case |
|------|----------|
| `udp` | Stream H.264/RTP over UDP — view with VLC or any RTP-capable player |
| `v4l2` | Write raw frames to a v4l2loopback device — feed into standalone apps (e.g. VisionPilot 0.9) |

### Dependencies (Ubuntu 22.04)

```bash
sudo apt install \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-tools gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
  v4l2loopback-dkms v4l2loopback-utils \
  ros-humble-cv-bridge ros-humble-image-transport
```

### Build

```bash
source /opt/ros/humble/setup.bash
cd <SODASIM_REPO_ROOT>/ros2_gstreamer
unset CXX CC   # avoid stale ccache env vars
colcon build --symlink-install
source install/setup.bash
```

### Launch — UDP mode (stream to network)

```bash
ros2 launch sodasim_gstreamer image_to_gstreamer.launch.py \
  mode:=udp host:=<VIEWER_IP> port:=5600
```

**VLC validation:**

Create `test.sdp`:
```sdp
v=0
o=- 0 0 IN IP4 0.0.0.0
s=ROS Camera
c=IN IP4 0.0.0.0
t=0 0
m=video 5600 RTP/AVP 96
a=rtpmap:96 H264/90000
a=fmtp:96 packetization-mode=1
a=recvonly
```

Open with VLC: `vlc test.sdp`

If you don't see video, check the network path and firewall (UDP port 5600).

### Launch — v4l2 mode (virtual camera for VisionPilot 0.9)

**1. Create the virtual device (once per boot):**

```bash
sudo modprobe v4l2loopback devices=1 video_nr=10 card_label="SodaSim"
sudo chmod 666 /dev/video10
```

**2. Launch the bridge:**

```bash
ros2 launch sodasim_gstreamer image_to_gstreamer.launch.py \
  mode:=v4l2 device:=/dev/video10
```

**3. Test before connecting to VisionPilot:**

```bash
# Visual check
ffplay /dev/video10

# GStreamer check
gst-launch-1.0 v4l2src device=/dev/video10 ! videoconvert ! autovideosink
```

**4. Configure VisionPilot 0.9** (`visionpilot_sodasim.conf`):

```ini
mode=camera
source.camera.auto_select=false
source.camera.device_id=/dev/video10
```

### All launch arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `mode` | `udp` | Output mode: `udp` or `v4l2` |
| `input_topic` | `/vehicle/camera` | ROS2 image topic to read from |
| `target_fps` | `30.0` | Target output framerate |
| `host` | `127.0.0.1` | [udp] Destination host |
| `port` | `5600` | [udp] Destination UDP port |
| `device` | `/dev/video10` | [v4l2] V4L2 loopback device path |
