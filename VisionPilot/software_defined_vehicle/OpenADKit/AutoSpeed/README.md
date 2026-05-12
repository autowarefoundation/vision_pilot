# AutoSpeed - Open AD Kit Demo

Containerized AutoSpeed Demo, closest in-path object detection and tracking.

## Prerequisites

- Download the [AutoSpeed ONNX model weights](https://drive.google.com/file/d/1Zhe8uXPbrPr8cvcwHkl1Hv0877HHbxbB/view?usp=drive_link) and place it in the `model-weights` directory with the name `autospeed.onnx`.

    ```bash
    mkdir -p model-weights
    curl "https://drive.usercontent.google.com/download?id=1Zhe8uXPbrPr8cvcwHkl1Hv0877HHbxbB&confirm=xxx" -o model-weights/autospeed.onnx
    ```

## Usage

```bash
./launch-autospeed.sh
```

## Output

After the container is running, you can access the visualization by opening the following URL in your browser:

<http://127.0.0.1:6080/vnc.html?resize=scale&autoconnect=true&password=visualizer>

> **Note:** Use `127.0.0.1` instead of `localhost`. On systems where `localhost` resolves to IPv6 (`::1`), the connection will fail as Podman's `pasta` network backend only handles IPv4.

The output shows closest in-path object detection and tracking of the input video in real-time.
