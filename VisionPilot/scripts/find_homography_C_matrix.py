"""Generate preprocess homography C: dataset ground H (YAML) + fixed VisionPilot model V."""

import argparse
from pathlib import Path

import cv2
import numpy as np

CANONICAL_WORLD_PTS = np.array(
    [[15, 5, 1], [150, 5, 1], [15, -5, 1], [150, -5, 1]], dtype=np.float32
)

# DO NOT MODIFY! VisionPilot model-view homography (1024x512 pixel -> world). Zenseact Open Dataset
V = np.array(
    [
        [0.00209514907, -0.000941721466, -9.24906396],
        [0.00662758637, -0.000352940531, -3.33396502],
        [0.000120077371, -0.00411343505, 1.0],
    ],
    dtype=np.float32,
)


def load_homography_H_matrix(path: Path) -> np.ndarray:
    # Load the homography matrix from the YAML file
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    H = fs.getNode("H").mat()
    fs.release()
    return H


def find_homography_C_matrix(H: np.ndarray) -> np.ndarray:
    H_inv = np.linalg.inv(H)
    V_inv = np.linalg.inv(V)

    uv = H_inv @ CANONICAL_WORLD_PTS.T
    uv_pts = (uv[:2] / uv[2]).T
    pq = V_inv @ CANONICAL_WORLD_PTS.T
    pq_pts = (pq[:2] / pq[2]).T

    C, _ = cv2.findHomography(uv_pts, pq_pts, method=0)
    return C


def main() -> None:
    parser = argparse.ArgumentParser(description="Build C from --ground-h and fixed VP model V")
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    H = load_homography_H_matrix(Path('../config/H.yaml'))
    C = find_homography_C_matrix(H)

    out = args.output
    out.parent.mkdir(parents=True, exist_ok=True)
    fs = cv2.FileStorage(str(out.resolve()), cv2.FILE_STORAGE_WRITE)
    fs.write("C", C.astype(np.float32))
    fs.release()
    print(f"Transformation C matrix saved to {out.resolve()}")


if __name__ == "__main__":
    main()
