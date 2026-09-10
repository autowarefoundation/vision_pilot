#!/usr/bin/env python3
"""Generate the CARLA preprocess homography C from H_carla.yaml.

Reuses VisionPilot's committed C-derivation (the fixed model-view V and
find_homography_C_matrix) without modifying VisionPilot. Writes the result next to
this script's config/, which the drive bind-mounts (read-only) over
/workspace/VisionPilot/config/homography_C_matrix.yaml.

    python3 gen_carla_C_matrix.py            # config/H_carla.yaml -> config/homography_C_matrix.yaml
"""
import argparse
import sys
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[3]  # repo root
sys.path.insert(0, str(ROOT / "VisionPilot" / "scripts"))
from find_homography_C_matrix import find_homography_C_matrix, load_homography_H_matrix  # noqa: E402


def main() -> None:
    cfg = Path(__file__).resolve().parent / "config"
    p = argparse.ArgumentParser(description="Build CARLA C from H_carla.yaml + fixed VP model V")
    p.add_argument("--ground-h", type=Path, default=cfg / "H_carla.yaml")
    p.add_argument("--output", type=Path, default=cfg / "homography_C_matrix.yaml")
    args = p.parse_args()

    C = find_homography_C_matrix(load_homography_H_matrix(args.ground_h))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fs = cv2.FileStorage(str(args.output.resolve()), cv2.FILE_STORAGE_WRITE)
    fs.write("C", C.astype("float32"))
    fs.release()
    print(f"CARLA C matrix saved to {args.output.resolve()} (ground H: {args.ground_h})")


if __name__ == "__main__":
    main()
