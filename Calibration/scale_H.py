import cv2
import argparse
import sys
from os import path
import numpy as np

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="Path to the input yaml file")
    parser.add_argument("--output", default="H_scaled.yaml", help="Path to the output yaml file")
    parser.add_argument("--cal-res", nargs=2, required=True, help="Camera resolution (H x V) used during the calibration")
    parser.add_argument("--vp-res", nargs=2, required=True, help="Camera resolution (H x V) that will be used with VisionPilot")
    args = parser.parse_args()

    # Convert arguments to list of floats
    cal_res = list(map(float, args.cal_res))
    vp_res = list(map(float, args.vp_res))

    # Aspect ratios
    cal_ratio = cal_res[0] / cal_res[1]
    vp_ratio = vp_res[0] / vp_res[1]

    # Stop script if aspect ratios do not match
    if cal_ratio != vp_ratio:
        sys.exit("ERROR: Calibration aspect ratio does not match VisionPilot aspect ratio.")
    print("Aspect ratio matches.")

    fs = cv2.FileStorage(args.input, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        sys.exit(f"ERROR: Could not open {args.input}")
    print(f"{args.input} was found.\n")

    # Try to get homography matrix with either name ('homography' or 'H')
    data = fs.getNode("homography").mat()

    if data is None:
        data = fs.getNode("H").mat()

    if data is None:
        sys.exit("ERROR: Homography matrix could not be found. Make sure that the tag is either 'homography or 'H'.")

    fs.release()
    print("Old H:")
    print(data)

    # Perform scaling
    ratio = vp_res[0] / cal_res[0]
    ratio_inv = np.diag([1/ratio, 1/ratio, 1.0])
    H_new = data @ ratio_inv

    print("\nNew H:")
    print(H_new)

    # Save yaml
    fs = cv2.FileStorage(args.output, cv2.FILE_STORAGE_WRITE)
    fs.write(name='H', val=H_new)
    fs.release()

    abs_path = path.abspath(args.output)
    print(f"\nHomography matrix was saved in {abs_path}\n")

if __name__ == "__main__":
    main()