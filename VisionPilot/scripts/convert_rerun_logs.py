#!/usr/bin/env python3
"""Convert VisionPilot rerun_logs into a native Rerun recording (.rrd).

Requires:
    python3 -m pip install rerun-sdk

Usage:
    python3 scripts/convert_rerun_logs.py --input rerun_logs --output visionpilot.rrd
    rerun visionpilot.rrd

Live mode:
    python3 scripts/convert_rerun_logs.py --input rerun_logs --output visionpilot.rrd --follow
    rerun visionpilot.rrd --follow
"""

from __future__ import annotations

import argparse
import json
import logging
import time
from pathlib import Path
from typing import Any

import rerun as rr


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert VisionPilot rerun_logs into a native Rerun recording (.rrd)."
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=Path("rerun_logs"),
        help="Directory containing per-frame rerun_logs output.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("visionpilot.rrd"),
        help="Output Rerun recording file path.",
    )
    parser.add_argument(
        "--application-id",
        type=str,
        default="visionpilot",
        help="Rerun application id to use for the recording.",
    )
    parser.add_argument(
        "--follow",
        action="store_true",
        help="Watch input dir and stream new frames as they appear.",
    )
    parser.add_argument(
        "--poll-interval",
        type=float,
        default=1.0,
        help="Seconds to wait between scans when --follow is enabled.",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable verbose logging.",
    )
    return parser.parse_args()


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def read_bytes(path: Path) -> bytes:
    return path.read_bytes()


def frame_dirs(input_dir: Path) -> list[Path]:
    if not input_dir.exists():
        raise FileNotFoundError(f"Input dir not found: {input_dir}")
    dirs = [p for p in sorted(input_dir.iterdir()) if p.is_dir() and p.name.startswith("frame_")]
    return sorted(dirs, key=lambda p: int(p.name.split("frame_")[1]))


# Warning enum (mirrors common/types.hpp)
WARNING_NAMES = {0: "None", 1: "FCW", 2: "AEB", 3: "LLDW", 4: "RLDW"}

# AutoSteer/AutoSpeed 1024×512 input frame
NET_W = 1024
NET_H = 512


def log_scalar(rec: rr.RecordingStream, path: str, value: float) -> None:
    rec.log(path, rr.Scalars(float(value)))


def log_frame(rec: rr.RecordingStream, frame_dir: Path, frame_id: int) -> None:
    rr.set_time("frame", sequence=frame_id, recording=rec)

    # 1. Images
    # - raw           = orig camera frame
    # - cropped       = top-crop + resize (AutoSteer/AutoSpeed inference)
    # - warped_bev    = BEV warp (AutoDrive inference)
    # - visualization = viz stuffs
    images_dir = frame_dir / "images"
    image_map = {
        "camera_frame": "images/raw",
        "resized": "images/cropped",
        "warped_bev": "images/warped_bev",
        "visualization": "images/visualization",
    }
    if images_dir.exists():
        for name, entity in image_map.items():
            path = images_dir / f"{name}.png"
            if path.exists():
                rec.log(
                    entity,
                    rr.EncodedImage(contents=read_bytes(path), media_type="image/png"),
                )
                logging.debug("Logged image %s -> %s", path, entity)

    # 2. Model inference results
    inference_path = frame_dir / "inference.json"
    inference: dict[str, Any] = {}
    if inference_path.exists():
        inference = load_json(inference_path)

        # Latency (timeseries)
        for k in ("wall_ms", "pre_ms", "ad_ms", "as_ms", "asp_ms"):
            log_scalar(rec, f"inference/latency/{k}", inference.get(k, 0.0))

        # AutoDrive raw preds
        auto_drive = inference.get("auto_drive", {})
        log_scalar(rec, "inference/auto_drive/dist_normalized", auto_drive.get("dist_normalized", 0.0))
        log_scalar(rec, "inference/auto_drive/curvature_raw", auto_drive.get("curvature_raw", 0.0))
        log_scalar(rec, "inference/auto_drive/flag_prob", auto_drive.get("flag_prob", 0.0))

        # AutoSteer raw preds - egopath waypoints + cropped img
        # - xp[i] : normalized lateral x at fixed image row i
        # - v     : linspace
        auto_steer = inference.get("auto_steer", {})
        xp = auto_steer.get("xp", [])
        h_vector = auto_steer.get("h_vector", [])
        if xp:
            n = len(xp)
            pts = []
            for i, x in enumerate(xp):
                conf = h_vector[i] if i < len(h_vector) else 1.0
                if conf < 0.5:
                    continue
                u = float(x) * NET_W
                v = (float(i) / max(n - 1, 1)) * (NET_H - 1)
                pts.append([u, v])
            if pts:
                rec.log(
                    "images/cropped/auto_steer_waypoints",
                    rr.Points2D(pts, radii=3.0, colors=[0, 200, 255]),
                )
        log_scalar(rec, "inference/auto_steer/num_waypoints", len(xp))

        # AutoSpeed raw preds - detection bboxes + cropped img
        auto_speed = inference.get("auto_speed", {})
        detections = auto_speed.get("detections", [])
        if detections:
            boxes, labels = [], []
            for det in detections:
                boxes.append([float(det["x1"]), float(det["y1"]), float(det["x2"]), float(det["y2"])])
                labels.append(f"cls{det.get('class_id', 0)} {float(det.get('score', 0.0)):.2f}")
            rec.log(
                "images/cropped/auto_speed_detections",
                rr.Boxes2D(array=boxes, array_format=rr.Box2DFormat.XYXY, labels=labels),
            )
        log_scalar(rec, "inference/auto_speed/num_detections", len(detections))

        # Validity flags
        rec.log(
            "inference/flags",
            rr.AnyValues(
                auto_drive_valid=bool(auto_drive.get("valid", False)),
                auto_steer_valid=bool(auto_steer.get("valid", False)),
                auto_speed_valid=bool(auto_speed.get("valid", False)),
                cipo_valid=bool(inference.get("cipo", {}).get("valid", False)),
                lateral_valid=bool(inference.get("lateral", {}).get("valid", False)),
                lateral_path_valid=bool(inference.get("lateral", {}).get("path_valid", False)),
            ),
        )

        # 3. Fusion/Safety Guardian
        cipo = inference.get("cipo", {})
        log_scalar(rec, "fusion/cipo/distance_m", cipo.get("distance_m", 0.0))      # in-path object distance
        log_scalar(rec, "fusion/cipo/velocity_ms", cipo.get("velocity_ms", 0.0))    # in-path object speed

        lateral = inference.get("lateral", {})
        log_scalar(rec, "fusion/lateral/cte_m", lateral.get("cte_m", 0.0))          # cross-track error
        log_scalar(rec, "fusion/lateral/yaw_rad", lateral.get("yaw_rad", 0.0))      # yaw error
        log_scalar(rec, "fusion/lateral/curvature", lateral.get("curvature", 0.0))  # curvature

        # BEV coords of filtered path
        path_bev = lateral.get("path_bev", [])
        if path_bev:
            strip = [[-float(y), -float(x)] for (x, y) in path_bev]
            rec.log("bev/filtered_path", rr.LineStrips2D([strip], colors=[255, 220, 0], radii=0.15))
            rec.log("bev/ego", rr.Points2D([[0.0, 0.0]], radii=0.4, colors=[0, 255, 0]))

    # 4. Vehicle data
    vehicle_path = frame_dir / "vehicle.json"
    if vehicle_path.exists():
        vehicle = load_json(vehicle_path)
        log_scalar(rec, "vehicle/ego_speed_ms", vehicle.get("ego_speed_ms", 0.0))

    # 5. Planner results
    plan_path = frame_dir / "plan.json"
    plan: dict[str, Any] = {}
    if plan_path.exists():
        plan = load_json(plan_path)
        steering = plan.get("steering", [])
        # Desired tyre angle = first element of steering sequence [rad]
        tyre_angle = float(steering[0]) if steering else 0.0
        log_scalar(rec, "planner/tyre_angle_rad", tyre_angle)
        log_scalar(rec, "planner/acceleration", plan.get("acceleration", 0.0))

        warnings = plan.get("warnings", [])
        warn_names = [WARNING_NAMES.get(int(w), str(w)) for w in warnings if int(w) != 0]
        if warn_names:
            rec.log("planner/warnings", rr.TextLog(", ".join(warn_names), level=rr.TextLogLevel.WARN))

    if inference_path.exists() or plan_path.exists() or vehicle_path.exists():
        rec.flush()


def main() -> int:
    args = parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO, format="%(message)s")

    frame_paths = frame_dirs(args.input)
    if not frame_paths and not args.follow:
        raise RuntimeError(f"No frame directories found in {args.input}")

    logging.info("Writing Rerun recording to %s", args.output)

    rec = rr.RecordingStream(args.application_id, make_default=True)
    rec.save(args.output)

    with rec:
        processed = set()
        while True:
            frame_paths = frame_dirs(args.input)
            if frame_paths:
                for frame_dir in frame_paths:
                    frame_id = int(frame_dir.name.split("frame_")[1])
                    if frame_id in processed:
                        continue
                    logging.info("Logging frame %d", frame_id)
                    log_frame(rec, frame_dir, frame_id)
                    processed.add(frame_id)
            if not args.follow:
                break
            time.sleep(args.poll_interval)

    logging.info("Saved Rerun recording to %s", args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
