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


def log_frame(rec: rr.RecordingStream, frame_dir: Path, frame_id: int) -> None:
    rr.set_time("frame", sequence=frame_id, recording=rec)

    # Images
    images_dir = frame_dir / "images"
    if images_dir.exists():
        for name in ["camera_frame", "warped_bev", "resized"]:
            path = images_dir / f"{name}.png"
            if path.exists():
                rec.log(
                    f"camera/{name}",
                    rr.EncodedImage(contents=read_bytes(path), media_type="image/png"),
                )
                logging.debug("Logged image %s", path)

    # Inference metadata
    inference_path = frame_dir / "inference.json"
    if inference_path.exists():
        inference = load_json(inference_path)
        values = {
            "frame_id": int(inference.get("frame_id", frame_id)),
            "wall_ms": float(inference.get("wall_ms", 0.0)),
            "pre_ms": float(inference.get("pre_ms", 0.0)),
            "ad_ms": float(inference.get("ad_ms", 0.0)),
            "as_ms": float(inference.get("as_ms", 0.0)),
            "asp_ms": float(inference.get("asp_ms", 0.0)),
            "auto_drive_valid": bool(inference.get("auto_drive", {}).get("valid", False)),
            "auto_speed_valid": bool(inference.get("auto_speed", {}).get("valid", False)),
            "cipo_valid": bool(inference.get("cipo", {}).get("valid", False)),
            "lateral_path_valid": bool(inference.get("lateral", {}).get("path_valid", False)),
        }

        auto_drive = inference.get("auto_drive", {})
        values.update(
            {
                "dist_normalized": float(auto_drive.get("dist_normalized", 0.0)),
                "curvature_raw": float(auto_drive.get("curvature_raw", 0.0)),
                "flag_prob": float(auto_drive.get("flag_prob", 0.0)),
            }
        )

        cipo = inference.get("cipo", {})
        values.update(
            {
                "cipo_distance_m": float(cipo.get("distance_m", 0.0)),
                "cipo_velocity_ms": float(cipo.get("velocity_ms", 0.0)),
            }
        )

        lateral = inference.get("lateral", {})
        values.update(
            {
                "lateral_path_a": float(lateral.get("path_a", 0.0)),
                "lateral_path_b": float(lateral.get("path_b", 0.0)),
                "lateral_path_c": float(lateral.get("path_c", 0.0)),
                "lateral_raw_cte_m": float(lateral.get("raw_cte_m", 0.0)),
                "lateral_cte_m": float(lateral.get("cte_m", 0.0)),
            }
        )

        rec.log("metadata/inference", rr.AnyValues(**values))

        auto_speed = inference.get("auto_speed", {})
        detections = auto_speed.get("detections", [])
        if detections:
            boxes = []
            labels = []
            for det in detections:
                boxes.append([float(det["x1"]), float(det["y1"]), float(det["x2"]), float(det["y2"])])
                labels.append(str(det.get("class_id", "")))

            rec.log(
                "camera/auto_speed/detections",
                rr.Boxes2D(
                    array=boxes,
                    array_format=rr.Box2DFormat.XYXY,
                    labels=labels,
                ),
            )

    # Plan metadata
    plan_path = frame_dir / "plan.json"
    if plan_path.exists():
        plan = load_json(plan_path)
        warnings = plan.get("warnings", [])
        rec.log(
            "metadata/plan",
            rr.AnyValues(
                acceleration=float(plan.get("acceleration", 0.0)),
                steering_count=int(len(plan.get("steering", []))),
                warnings_count=int(len(warnings)),
            ),
        )
        rec.log(
            "metadata/plan/text",
            rr.TextLog(
                f"frame={frame_id} accel={plan.get('acceleration', 0.0):.3f} "
                f"steering={len(plan.get('steering', []))} warnings={warnings}",
            ),
        )

    if inference_path.exists() or plan_path.exists():
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
