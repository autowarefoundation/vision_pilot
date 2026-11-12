# VISUALIZATION MODULE

## Acknowledgement

I would like to thank [Ethan](https://dev.to/ethand91) and his blog post of [Streaming Camera with C++ WebRTC GStreamer](https://dev.to/ethand91/streaming-camera-with-c-webrtc-gstreamer-pof).

Your implementation was truly helpful and inspiring for me to complete this module.


## I. Overview

The WebRTC Visualization Module provides a real-time video streaming capability for the VisionPilot pipeline via WebRTC protocol. It serves the following core functions:

1. **Real-time frame capture and encoding** which accepts OpenCV `cv::Mat` frames and encodes them to VP8 video codec via GStreamer.
2. **WebRTC peer-to-peer streaming** which establishes a WebRTC peer connection between the server (VisionPilot app) and browser clients, enabling live video delivery over the internet or LAN.
3. **Lightweight browser client** which serves a minimal, self-contained HTML5 page with built-in WebRTC JavaScript client without external dependencies required for the browser.
4. Implements WebSocket-based signaling for SDP (Session Description Protocol) offer/answer negotiation and ICE (Interactive Connectivity Establishment) candidate exchange.
5. **Thread-safe frame streaming** that manages concurrent frame pushes from the main application thread while running a GStreamer pipeline and event loop in separate threads.

This module is essential for downstream remote monitoring, debugging, and visualization of autonomous driving pipelines during development and testing phases.


## II. Architecture && Module structure

### 1. Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    VisionPilot Application                      │
│                  (vision_pilot.cpp main thread)                 │
│                                                                 │
│   DATA CAPTURE                                                  │
│   ┌──────────────────────┐                                      │
│   │  V4L2/ROS2 Camera    │                                      │
│   │      Source          │                                      │
│   └──────────┬───────────┘                                      │
│              │                                                  │
│              │ cv::Mat frames (33ms loop)                       │
│              │                                                  │
│              ▼                                                  │
│   ┌──────────────────────────────────────────┐                  │
│   │  (Various upstream modules, like         │                  │
│   │  model inference, processing, calc etc.) │                  │
│   └──────────┬───────────────────────────────┘                  │
│              │                                                  │
│              │ Frames & longitudinal/lateral planning results   │
│              │                                                  │
│              ▼                                                  │
│   VISUALIZATION                                                 │
│   ┌──────────────────────────────────────────┐                  │
│   │  visualization::render_frame()           │                  │
│   │  (draw frame + planning results)         │                  │
│   └──────────┬───────────────────────────────┘                  │
│              │                                                  │
│              ▼                                                  │
│   ┌──────────────────────────────────────────┐                  │
│   │  WebRTCStreamer::push_frame()            │                  │
│   │  (stream to endpoint via WebRTC)         │                  │
│   └──────────┬───────────────────────────────┘                  │
│              │                                                  │
└──────────────┼──────────────────────────────────────────────────┘
               │
               │ BGR frames + metadata
               │
    ┌──────────▼───────────────────────────────────────────┐
    │         WebRTCStreamer::Impl (Internal)              │
    │                                                      │
    │  ┌────────────────────────────────────────────────┐  │
    │  │  GStreamer Pipeline (separate thread)          │  │
    │  │                                                │  │
    │  │  appsrc => queue => videoconvert => vp8enc =>  │  │
    │  │  rtpvp8pay => webrtcbin                        │  │
    │  │                                                │  │
    │  │  ┌──────────────────────────────────────────┐  │  │
    │  │  │ WebRTC peer connection (GStreamer)       │  │  │
    │  │  │  - Manages media stream                  │  │  │
    │  │  │  - Generates SDP offers                  │  │  │
    │  │  │  - Gathers ICE candidates                │  │  │
    │  │  └──────────────────────────────────────────┘  │  │
    │  └────────────────────────────────────────────────┘  │
    │                                                      │
    │  ┌────────────────────────────────────────────────┐  │
    │  │  Signaling Layer (SoupServer + WebSocket)      │  │
    │  │                                                │  │
    │  │  HTTP handler:                                 │  │
    │  │    GET / => serves kBrowserHtml                │  │
    │  │                                                │  │
    │  │  WebSocket handler:                            │  │
    │  │    - Receives: SDP answer, ICE candidates      │  │
    │  │    - Sends: SDP offer, ICE candidates          │  │
    │  │    - Queue + flush mechanism for ordering      │  │
    │  │                                                │  │
    │  └────────────────────────────────────────────────┘  │
    │                                                      │
    └───────────────┬──────────────────────────────────────┘
                    │
        ┌───────────┴──────────────┐
        │                          │
        ▼                          ▼
   ┌─────────────────┐        ┌──────────────────┐
   │  Browser Client │        │  Network         │
   │  (HTML5 + JS)   │ <====> │  (Internet/LAN)  │
   │                 │        └──────────────────┘
   │ ┌─────────────┐ │        
   │ │ RTCPeerConn │ │
   │ │ (signaling) │ │
   │ ├─────────────┤ │
   │ │ WebSocket   │ │
   │ │ (SDP/ICE)   │ │
   │ ├─────────────┤ │
   │ │ <video>     │ │
   │ │ (playback)  │ │
   │ └─────────────┘ │
   └─────────────────┘
```

### 2. Flow summary

1. `VisionPilot` application calls `webrtc_streamer->push_frame(cv::Mat)` in its main loop. This frame is generated from `visualization::render_frame()`.
2. Frame is validated, converted to BGR, and pushed to the GStreamer pipeline's `appsrc` element.
3. GStreamer encodes the frame using VP8 codec and feeds it to the `webrtcbin` element.
4. On the first frame, `webrtcbin` triggers `on-negotiation-needed`, which creates an SDP offer.
5. SDP offer is queued and sent to the browser client via WebSocket.
6. Browser responds with SDP answer and ICE candidates.
7. Server receives answer, sets remote description, and flushes any pending ICE candidates.
8. Media stream begins flowing from server to browser via the established peer connection.

### 3. Module structure

```
visualization/
├── CMakeLists.txt
├── README.md (this file)
├── include/
│   └── visualization/
│       ├── visualization.hpp           (visualization header)
│       ├── visualization_to_webrtc.hpp (WebRTC header)
│       ├── occupancy_view.hpp          (optional Occupancy BEV panel)
│       └── occupancy_bridge.hpp        (pipeline → Occupancy Scene)
└── src/
    ├── visualization.cpp               (visualization drawing, OpenCV window management)
    ├── visualization_to_webrtc.cpp     (WebRTC implementation)
    ├── occupancy_view.cpp              (optional; ENABLE_OCCUPANCY=ON)
    └── occupancy_bridge.cpp            (optional; ENABLE_OCCUPANCY=ON)
```


## II-b. Optional Occupancy BEV window

Heuristic 3D / bird's-eye Occupancy panel shown beside the main HUD. It is **not** a neural occupancy network — geometry is built from AutoSpeed detections, fused path, AutoSteer lanes, and lateral CTE.

**Build** (default OFF — stock HUD only):

```bash
cmake .. -DENABLE_OCCUPANCY=ON ...
```

Docker GPU image:

```bash
docker build -f docker/Dockerfile --build-arg ENABLE_OCCUPANCY=ON ...
```

When enabled, a second OpenCV window named **Occupancy** appears:

- light-gray ground + grid (0–150 m), green fused-path corridor
- AutoSpeed objects as extruded boxes (blue traffic, red CIPO, white ego)
- Mouse: L-drag orbit, R-drag pan, wheel zoom; key `R` resets the camera

Files: `occupancy_view.hpp` / `.cpp` (render + interaction), `occupancy_bridge.hpp` / `.cpp` (scene from pipeline outputs). Hook: one `occupancy::publish(...)` call in `Visualization::build_frame`.


## III. Build

### 1. Prerequisites

- `ROS2 Humble` (tested on Ubuntu 22.04)
    - `source /opt/ros/humble/setup.bash`
- `GStreamer` development libraries:
    - `libgstreamer1.0-dev`
    - `libgstreamer-plugins-base1.0-dev`
    - `libgstreamer-plugins-bad1.0-dev`
- `libsoup 2.4` (HTTP/WebSocket server):
    - `libsoup2.4-dev`
- `JSON-GLib` (JSON signaling message handling):
    - `libjson-glib-dev`
- `OpenCV`:
  - `libopencv-dev`
- `Standard build tools`:
  - `build-essential`, `cmake` (≥3.22.1), `pkg-config`

Install all at once:

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake pkg-config \
  libopencv-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  libgstreamer-plugins-bad1.0-dev \
  libsoup2.4-dev libjson-glib-dev
```

### 2. Steps

```bash
# 1. Navigate to workspace root
cd /path/to/VisionPilot/development_releases/1.0

# 2. Source ROS2
source /opt/ros/humble/setup.bash

# 3. Build (from workspace root; CMake will configure all modules)
mkdir -p build && cd build
cmake .. -DONNXRUNTIME_ROOT=$your_ONNXRUNTIME_path
make -j$(nproc)
```

### 3. Expected Output

```bash
[ 83%] Built target visualization
[ 89%] Building CXX object app/CMakeFiles/VisionPilot.dir/vision_pilot.cpp.o
[ 97%] Linking CXX executable ../VisionPilot
[100%] Built target VisionPilot
```

Binary location: `build/VisionPilot`


## IV. Test/demo

### 1. Running with WebRTC enabled

This demo shall guide you through testing this WebRTC streaming with a V4L2 mount, streamed 
via a combination of the `v4l2loopback` kernel module and FFmpeg.

With this demo, you will:
1. Publish a V4L2 video streaming mount from a local video.
2. Use `VisionPilot` application to subscribe to that streaming mount, process and stream frames to a local host.

```bash
# 1. Navigate to build directory
cd /path/to/VisionPilot/development_releases/1.0/build


# 2. Initiate V4L2 streaming mount

# a. Install package
sudo apt update
sudo apt install ffmpeg -y
sudo apt install v4l2loopback-dkms -y

# b. Load the module (assuming you gonna stream it at `/dev/video9`)
sudo modprobe -r v4l2loopback
sudo modprobe v4l2loopback video_nr=9 card_label="Virtual Camera" exclusive_caps=1

# c. Publish looping video at that mount
ffmpeg -re -stream_loop -1 -i <absolute path to local video> -f v4l2 -pix_fmt yuv420p /dev/video9


# 3. Kickstart VisionPilot app with V4L2 subscription to that mount, and stream frames to http://127.0.0.1:8080/
./VisionPilot 1 /dev/video0 10 1 8080
```

**Arguments:**

- `1`: V4L2 mode (use `0` for ROS 2 mode)
- `/dev/video0`: V4L2 device path (if ROS 2 mode, this second arg will be ROS 2 topic name)
- `10`: target FPS
- `1`: enable WebRTC (use `0` to disable)
- `8080`: WebRTC server port (not available if WebRTC is disabled)

**Expected terminal output:**

```bash
Starting in V4L2 mode with device: /dev/video9 and FPS: 10
[V4L2Reader INFO] Initializing V4L2 Reader
[V4L2Reader INFO]   Device Path: /dev/video9
[V4L2Reader INFO]   Target FPS: 10
[V4L2Reader INFO] V4L2 device configured successfully
[V4L2Reader INFO]   Received resolution: 2560x1440
[V4L2Reader INFO]   Received FPS: 10.000000
Starting WebRTC streamer on port: 8080
[WebRTCStreamer] soup_server created
[WebRTCStreamer] soup_server listening on port 8080 and handlers installed
[WebRTCStreamer] pipeline created, appsrc=0x57add3f02e20 webrtc=0x57add3f34110
[WebRTCStreamer] pipeline set to PLAYING
Open browser at: http://127.0.0.1:8080/
Local OpenCV preview is disabled while WebRTC is enabled.
```

### 2. Accessing the stream

1. Open a web browser and navigate to: `http://127.0.0.1:8080/`
2. The minimal HTML client page will load.
3. WebRTC negotiation will begin automatically.
4. Once offer/answer exchange completes, the video stream should appear in the `<video>` element.

### 3. Troubleshooting

These are known/encountered bugs and errors. If you encounter a completely new one, try posting it as new issue
at [Autoware VisionPilot repository](https://github.com/autowarefoundation/autoware_vision_pilot).

1. **Black/blank video**: frame may not be arriving from the camera. Test with WebRTC disabled:
  
    ```bash
    ./VisionPilot 1 /dev/video0 10 0
    ```
    
    If OpenCV preview works, the camera is fine, then the issue might be somewhat WebRTC-specific.
  
2. **Connection refused**: ensure port `8080` is not in use:
  
    ```bash
    lsof -i :8080
    ```


## V. Technical details

### 1. Core components & data flow

#### a. WebRTCStreamer public interface

**Location:** `include/visualization/visualization_to_webrtc.hpp`

The `WebRTCStreamer` class provides a clean public API:

```cpp
class WebRTCStreamer {
    struct Config { ... };                  // Configuration: host, port, path, frame_rate
    WebRTCStreamer(Config config);          // Constructor
    ~WebRTCStreamer();                      // Destructor
    
    bool start();                           // Initialize server, GStreamer, event loop
    bool stop();                            // Cleanup and shutdown
    bool push_frame(const cv::Mat& frame);  // Submit frame for encoding/streaming
    bool is_running() const;                // Check if server is active
    bool has_client() const;                // Check if browser is connected
    std::string browser_url() const;        // Get URL for browser to connect
    
private:
    std::unique_ptr<Impl> impl;             // Private implementation pattern
};
```

Above private implementation `impl` is used to separate public interface from internal complexity.

#### b. WebRTCStreamer::Impl structure

**Location:** `src/visualization_to_webrtc.cpp` (lines ~196–250)

The `Impl` struct encapsulates all internal state and logic:

```cpp
struct WebRTCStreamer::Impl {

    // ===== CONFIGURATION =====
    Config config;                              // User-provided settings
    
    // ===== SIGNALING (HTTP + WebSocket) =====
    SoupServer *server;                         // libsoup HTTP server
    GMainLoop *main_loop;                       // GLib event loop
    std::thread server_thread;                  // Thread running the event loop
    
    // ===== GSTREAMER PIPELINE =====
    GstElement *pipeline;                       // Top-level pipeline element
    GstElement *appsrc;                         // Input: receives frames from app
    GstElement *webrtc;                         // webrtcbin: handles RTC logic
    
    // ===== SIGNALING STATE =====
    mutable std::mutex signal_mutex;
    SoupWebsocketConnection *client_connection; // Active WS connection to browser
    std::vector<std::string> pending_signals;   // Queued SDP/ICE messages
    
    // ===== REMOTE DESCRIPTION & ICE STATE =====
    std::mutex remote_candidate_mutex;
    std::vector<std::pair<int, std::string>> pending_remote_candidates;
    std::atomic<bool> remote_description_ready; // Flag: can add ICE candidates?
    
    // ===== FRAME STREAMING STATE =====
    std::atomic<bool> running;                  // Pipeline active?
    std::atomic<uint64_t> frame_index;          // Monotonic frame counter for PTS
    bool caps_configured;                       // GStreamer caps set?
    int configured_width, configured_height;    // Last configured frame dimensions
};
```

**Key design decisions:**

- **Mutex for signal_mutex** protects concurrent access to `client_connection` and `pending_signals` (app thread pushes frames; server thread sends signals).
- **Atomic flags** (`running`, `remote_description_ready`, `frame_index`) provide lock-free read/write for frequently checked state.
- **Pending queues** decouple frame pushing (app thread) from signaling (server thread).

#### c. Frame timestamping & synchronization

In `push_frame()` (lines ~770–820), each frame is timestamped:

```cpp
const guint64 duration_ns = config.frame_rate > 0.0
    ? static_cast<guint64>(GST_SECOND / config.frame_rate)
    : GST_CLOCK_TIME_NONE;

const guint64 pts_ns = (
    duration_ns == GST_CLOCK_TIME_NONE ? 0 : 
    frame_index.fetch_add(1, std::memory_order_acq_rel) * duration_ns
);

GST_BUFFER_PTS(buffer) = pts_ns;
GST_BUFFER_DTS(buffer) = pts_ns;
GST_BUFFER_DURATION(buffer) = duration_ns;
```

Explanation:

- **PTS (Presentation Time Stamp)** tells decoder when to display the frame.
- **DTS (Decoding Time Stamp)** tells decoder when to decode the frame.
- **Duration** is frame display duration (inverse of framerate).
- **Monotonic `frame_index`** ensures PTS increases, preventing jitter.

For example, at 10 FPS = 100 ms per frame:

- Frame 0: PTS = 0 ns
- Frame 1: PTS = 100,000,000 ns (100 ms)
- Frame 2: PTS = 200,000,000 ns (200 ms)
- etc.

This allows the browser to play frames at the correct speed.

---

#### d. Format validation `ensure_bgr_frame()`

In the anonymous namespace (lines ~148–190), frames are converted to BGR:

```cpp
cv::Mat ensure_bgr_frame(const cv::Mat& frame) {
    if (frame.empty()) return frame;
    if (frame.type() == CV_8UC3) return frame.isContinuous() ? frame : frame.clone();
    
    cv::Mat converted;
    if (frame.type() == CV_8UC1)        // Grayscale
        cv::cvtColor(frame, converted, cv::COLOR_GRAY2BGR);
    else if (frame.type() == CV_8UC4)   // BGRA
        cv::cvtColor(frame, converted, cv::COLOR_BGRA2BGR);
    else                                 // Unknown
        frame.convertTo(converted, CV_8UC3);
    
    return converted;
}
```

Since GStreamer expects BGR for the `videoconvert` element, I shall ensure that.

---

### 2. WebRTC signaling flow

#### a. SDP offer/answer exchange

##### i. Sequence (lines ~461–510 and ~768–820)

1. On first frame push (line ~768):
   - `webrtcbin` element detects media stream and emits `on-negotiation-needed` signal.
   
2. Upon `on_negotiation_needed()` callback (lines ~518–530):
   ```cpp
   g_signal_emit_by_name(impl->webrtc, "create-offer", nullptr, promise);
   ```
   - Creates a GStreamer promise to generate an SDP offer asynchronously.
   
3. Upon `on_offer_created()` callback (lines ~475–510):
   - Receives the generated SDP offer.
   - Sets it as the local description: `g_signal_emit_by_name(impl->webrtc, "set-local-description", ...)`.
   - Queues the offer to be sent to browser: `impl->queue_signal(make_offer_message(sdp_text))`.
   
4. Browser receives offer via WebSocket (lines ~346–408):
   - In JavaScript: `pc.setRemoteDescription({type: 'offer', sdp: ...})`.
   - Browser's RTCPeerConnection generates an answer.
   - Browser sends answer back via WebSocket.
   
5. Server receives answer (lines ~389–402):
   - Parses incoming JSON: `handle_remote_description(impl, sdp_answer_text)`.
   - Creates `GstWebRTCSessionDescription` and sets it: `g_signal_emit_by_name(impl->webrtc, "set-remote-description", ...)`.
   - Marks `remote_description_ready = true` and flushes pending ICE candidates.

#### b. ICE candidate exchange

##### i. Server-side (lines ~537–551)

When server's `webrtcbin` gathers an ICE candidate:
```cpp
void on_ice_candidate(GstElement *element, guint mline_index, gchar *candidate, gpointer user_data) {
    impl->queue_signal(make_candidate_message(mline_index, candidate));
}
```

Candidate is sent to browser as JSON:
```json
{ "type": "candidate", "sdpMLineIndex": 0, "candidate": "candidate:..." }
```

##### ii. Browser-side (lines ~65–89 in kBrowserHtml):

```javascript
if (p.type === 'candidate') {
    const candidate = {
        candidate: p.candidate,
        sdpMLineIndex: p.sdpMLineIndex
    };
    if (!pc.remoteDescription) {
        pendingCandidates.push(candidate);  // Queue until offer is set
        return;
    }
    try {
        await pc.addIceCandidate(candidate);
    } catch (e) {
        console.error('Error adding ICE candidate:', e);
    }
}
```

Candidates are queued until `remoteDescription` is set, then all flushed. This prevents race conditions where ICE candidates arrive before SDP offer.

### 3. Signaling infra: HTTP + WebSocket

#### a. HTTP server (`libsoup`)

In `start()` (lines ~576–606):

```cpp
server = soup_server_new("server-header", "VisionPilot", NULL);
soup_server_listen_local(server, config.port, SOUP_SERVER_LISTEN_IPV4_ONLY, &listen_error);
soup_server_add_handler(server, "/", root_http_handler, this, nullptr);
soup_server_add_websocket_handler(server, config.websocket_path.c_str(), ..., websocket_handler, ...);
```

- `root_http_handler()` (lines ~253–276)

    - Responds to `GET /` with the embedded HTML client (`kBrowserHtml`).
    - Serves as a static content server; minimal overhead.

#### b. WebSocket signaling

- `websocket_handler()` (lines ~415–459):
    - Triggered when browser connects to Websocket.
    - Registers message and close callbacks.
    - Sets keepalive interval to detect dead connections.

- `on_websocket_message()` (lines ~346–408):
    - Parses incoming JSON (SDP answer or ICE candidate).
    - Dispatches to appropriate handler (`handle_remote_description()` or `handle_remote_candidate()`).

- `on_websocket_closed()` (lines ~283–296):
    - Cleans up connection reference when browser disconnects.

#### c. Message queueing & thread safety

- `queue_signal()` (lines ~861–884):
    ```cpp
    void queue_signal(const std::string& signal) {
        std::lock_guard<std::mutex> lock(signal_mutex);
        if (client_connection != nullptr && is_open(client_connection)) {
            soup_websocket_connection_send_text(client_connection, signal.c_str());
        } else {
            pending_signals.push_back(signal);
        }
    }
    ```

- `flush_pending_signals()` (lines ~887–912):
    - Called when new client connects.
    - Sends all queued signals (e.g., SDP offer generated before client was ready).
    - Allows graceful handshake even if timing is loose.

### 4. Browser client with a minimal piece of HTML5

#### a. Structure

`kBrowserHtml` constant (lines ~27–95)

```html
<!doctype html>
<html>
  <head>
    <title>VisionPilot</title>
    <style>/* Full-screen video */</style>
  </head>
  <body>
    <video id="video" autoplay playsinline muted></video>
    <script>/* WebRTC client logic */</script>
  </body>
</html>
```

#### b. Key JavaScript elements

1. RTCPeerConnection (`pc`)
   ```javascript
   const pc = new RTCPeerConnection();
   pc.ontrack = e => { video.srcObject = e.streams[0]; };
   ```
   - Creates peer connection.
   - On incoming track (video stream), assigns to `<video>` element.

2. WebSocket (`ws`)
   ```javascript
   const ws = new WebSocket(scheme + location.host + '/ws');
   ```
   - Connects to server's `/ws` endpoint.
   - Bidirectional signaling.

3. Offer/Answer handling
   ```javascript
   if (p.type === 'offer') {
       await pc.setRemoteDescription({type: 'offer', sdp: p.sdp});
       const a = await pc.createAnswer();
       await pc.setLocalDescription(a);
       ws.send(JSON.stringify({type: 'answer', sdp: pc.localDescription.sdp}));
   }
   ```

4. ICE candidate queueing
   ```javascript
   async function drainPendingCandidates() {
       while (pendingCandidates.length > 0) {
           const c = pendingCandidates.shift();
           await pc.addIceCandidate(c);
       }
   }
   ```
   - Ensures candidates are added only after remote description is set.

### 5. Threading model

#### a. Threading flow

```
┌─ Application thread (main)
│  │
│  ├─ Calls: webrtc_streamer.push_frame() [=> impl->push_frame()]
│  │          │
│  │          ├─ Validates frame
│  │          ├─ Calls: gst_app_src_push_buffer()
│  │          │  (thread-safe; enqueues to pipeline's queue element)
│  │          └─ Returns
│  │
│  └─ (Continues app loop, sleeps 33ms)
│
├─ GStreamer pipeline thread (spawned by g_main_loop_run)
│  │
│  ├─ Receives buffers from appsrc
│  ├─ Processes through pipeline: queue => videoconvert => vp8enc => webrtcbin
│  ├─ Emits signals: on-negotiation-needed, on-ice-candidate
│  ├─ Callbacks invoked synchronously in this thread context
│  └─ (Runs until pipeline state => NULL)
│
└─ libsoup event loop thread (spawned in start())
   │
   ├─ g_main_loop_run() processes I/O events
   ├─ Handles incoming HTTP GET / WebSocket messages
   ├─ Calls: root_http_handler, websocket_handler, on_websocket_message
   ├─ Accesses impl->client_connection (protected by signal_mutex)
   └─ (Runs until g_main_loop_quit())
```

#### b. Concurrency

- Push frame (app thread) => Queue in GStreamer (thread-safe with GStreamer's internal queues).
- ICE candidate emission (GStreamer thread) => Queue signal (protected by `signal_mutex`).
- WebSocket receive (libsoup thread) => Update state (protected by `signal_mutex`, `remote_candidate_mutex`).

### 6. Configuration

The `Config` struct allows customization:

```cpp
struct Config {
    std::string host = "127.0.0.1";
    uint16_t port = 8080;
    std::string websocket_path = "/ws";
    double frame_rate = 10.0;
};
```