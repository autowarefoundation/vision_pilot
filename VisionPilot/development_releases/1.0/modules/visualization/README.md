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