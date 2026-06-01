import os
import cv2
import json
import numpy as np

# =================================================
# CONFIGURATION
# =================================================

# Read data root from environment variable
BASE_DATA_DIR = os.environ.get("WOODSCAPE_DATA_DIR")

if BASE_DATA_DIR is None:
    raise RuntimeError(
        "Environment variable WOODSCAPE_DATA_DIR is not set.\n"
        "Please run:\n"
        "  export WOODSCAPE_DATA_DIR=/path/to/WoodScape/data/all_images"
    )

if not os.path.isdir(BASE_DATA_DIR):
    raise RuntimeError(
        "WOODSCAPE_DATA_DIR does not exist or is not accessible:\n"
        f"  {BASE_DATA_DIR}"
    )

FOLDERS = {
    "1": "left_fisheye",
    "2": "right_fisheye"
}

WINDOW_NAME = "Annotate"
WINDOW_TITLE_BASE = "Annotate | t = OCCUPIED (True), f = FREE (False)"

# =================================================
# Color Overlay Function
# =================================================

def apply_overlay(img, occupied, alpha=0.25):
    """
    Apply a semi-transparent overlay for visual confirmation.
    Red   -> OCCUPIED
    Green -> FREE
    """
    overlay = img.copy()
    color = (0, 0, 255) if occupied else (0, 255, 0)
    overlay[:] = color
    return cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0)

# =================================================
# MAIN
# =================================================

def main():

    # -------------------------------
    # Select camera folder
    # -------------------------------
    print("\nSelect input folder to annotate:")
    print("1) left_fisheye")
    print("2) right_fisheye")

    choice = ""
    while choice not in FOLDERS:
        choice = input("Enter choice (1 or 2): ").strip()

    DATA_ROOT = os.path.join(BASE_DATA_DIR, FOLDERS[choice])
    IMAGES_DIR = os.path.join(DATA_ROOT, "images")
    VIS_DIR = os.path.join(DATA_ROOT, "annotation_visualizations")
    JSON_PATH = os.path.join(DATA_ROOT, "occupancy_annotations.json")

    # -------------------------------
    # Validate folder structure
    # -------------------------------
    if not os.path.isdir(IMAGES_DIR):
        raise RuntimeError(
            "Expected images directory not found:\n"
            f"  {IMAGES_DIR}\n"
            "Check that WOODSCAPE_DATA_DIR is set correctly."
        )

    os.makedirs(VIS_DIR, exist_ok=True)

    print(f"\n✅ Annotating folder: {DATA_ROOT}")

    # -------------------------------
    # Load or initialize annotations
    # -------------------------------
    if os.path.exists(JSON_PATH):
        with open(JSON_PATH, "r") as f:
            annotations = json.load(f)
        print(f"✅ Loaded {len(annotations)} existing annotations")
    else:
        annotations = {}
        print("🆕 Starting new annotation file")

    # -------------------------------
    # Load image list
    # -------------------------------
    images = sorted(
        f for f in os.listdir(IMAGES_DIR)
        if f.lower().endswith(".png")
    )

    if not images:
        raise RuntimeError(f"No PNG images found in: {IMAGES_DIR}")

    print(f"✅ Found {len(images)} images")

    # -------------------------------
    # Create stable OpenCV window
    # -------------------------------
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1280, 800)
    cv2.moveWindow(WINDOW_NAME, 100, 100)

    # -------------------------------
    # Annotation loop
    # -------------------------------
    for idx, fname in enumerate(images, start=1):

        image_id = os.path.splitext(fname)[0]

        # Resume support
        if image_id in annotations:
            continue

        img_path = os.path.join(IMAGES_DIR, fname)
        img = cv2.imread(img_path)

        if img is None:
            print(f"⚠️ Could not read {fname}, skipping")
            continue

        # Update window title with legend and progress
        title = f"{WINDOW_TITLE_BASE} | {idx}/{len(images)}"
        cv2.setWindowTitle(WINDOW_NAME, title)

        # Show original image
        cv2.imshow(WINDOW_NAME, img)

        # ---------------------------
        # Wait for user input
        # ---------------------------
        while True:
            key = cv2.waitKey(0) & 0xFF

            if key == ord("t"):
                label = True
                break
            elif key == ord("f"):
                label = False
                break
            elif key == ord("q"):
                print("\n⏹ Annotation stopped by user")
                with open(JSON_PATH, "w") as f:
                    json.dump(annotations, f, indent=2)
                cv2.destroyAllWindows()
                return

        # ---------------------------
        # Apply & preview overlay (Variant A)
        # ---------------------------
        vis = apply_overlay(img, label)
        cv2.imshow(WINDOW_NAME, vis)
        cv2.waitKey(300)

        # ---------------------------
        # Save results
        # ---------------------------
        annotations[image_id] = label
        cv2.imwrite(os.path.join(VIS_DIR, fname), vis)

        with open(JSON_PATH, "w") as f:
            json.dump(annotations, f, indent=2)

        print(f"✅ {image_id}: {label}")

    # -------------------------------
    # Clean exit
    # -------------------------------
    cv2.destroyAllWindows()
    print("\n🎉 Annotation complete")

# =================================================
# ENTRY POINT
# =================================================

if __name__ == "__main__":
    main()
