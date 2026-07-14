import cv2
import numpy as np
import argparse
import sys

def sort_corners(centers):
    """
    Sorts 4 points into Top-Left, Top-Right, Bottom-Left, Bottom-Right.
    Assumes the camera is looking generally forward at a flat plane.
    """
    # Sort by Y-coordinate to separate top and bottom rows
    centers = sorted(centers, key=lambda p: p[1])
    
    top_two = centers[:2]
    bottom_two = centers[2:]
    
    # Sort by X-coordinate to separate left and right
    tl, tr = sorted(top_two, key=lambda p: p[0])
    bl, br = sorted(bottom_two, key=lambda p: p[0])
    
    return [tl, tr, bl, br]

def find_2x2_checkerboard_centers(image_path):
    img = cv2.imread(image_path)
    if img is None:
        print(f"Error: Could not read image at {image_path}")
        sys.exit(1)
        
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # A 2x2 checkerboard has exactly 1 internal corner
    pattern_size = (1, 1)
    centers = []
    
    # Radius to mask out detected corners so they aren't found twice
    mask_radius = max(gray.shape) // 20 
    
    print("Detecting 2x2 checkerboards...")
    for i in range(4):
        ret, corners = cv2.findChessboardCorners(
            gray, 
            pattern_size, 
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )
        
        if ret:
            # Refine corner location to sub-pixel accuracy
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            center = corners[0][0]
            centers.append(center)
            
            # Draw a white circle over the detected center to mask it out
            cv2.circle(gray, (int(center[0]), int(center[1])), mask_radius, 255, -1)
        else:
            print(f"Error: Could only find {i} checkerboards. Ensure four 2x2 checkerboards are visible.")
            sys.exit(1)

    return sort_corners(centers), img

def project_world_to_image(world_pts, H_inv):
    """
    Manually projects world points to image points using H_inv.
    Handles the homogeneous coordinate division and filters out points
    that fall behind the camera plane (w <= 0).
    """
    ones = np.ones((world_pts.shape[0], 1))
    homog_world = np.hstack([world_pts, ones])  # Shape: (N, 3)
    
    # Project: x_img = H_inv * x_world (using row vectors: homog_world @ H_inv^T)
    homog_img = homog_world @ H_inv.T  # Shape: (N, 3)
    
    # We only care about points in front of the camera projection plane (w > 0)
    valid = homog_img[:, 2] > 1e-5
    
    img_pts = np.zeros_like(world_pts)
    img_pts[valid, 0] = homog_img[valid, 0] / homog_img[valid, 2]
    img_pts[valid, 1] = homog_img[valid, 1] / homog_img[valid, 2]
    
    return img_pts, valid

def draw_world_line(img, p1, p2, H_inv, color=(0, 255, 0), thickness=2):
    """
    Samples points along a 2D world line, projects them, and draws the visible segments.
    This prevents visual artifacts caused by lines crossing the horizon line.
    """
    # Sample 100 points along the segment in world coordinates
    t = np.linspace(0, 1, 100)[:, None]
    line_world = p1 + t * (p2 - p1)
    
    img_pts, valid = project_world_to_image(line_world, H_inv)
    
    # Draw valid segments
    for i in range(len(valid) - 1):
        if valid[i] and valid[i+1]:
            pt1 = (int(round(img_pts[i][0])), int(round(img_pts[i][1])))
            pt2 = (int(round(img_pts[i+1][0])), int(round(img_pts[i+1][1])))
            
            # Simple boundary check to prevent overflow drawing crashes
            h, w = img.shape[:2]
            if (-w < pt1[0] < w*2 and -h < pt1[1] < h*2 and
                -w < pt2[0] < w*2 and -h < pt2[1] < h*2):
                cv2.line(img, pt1, pt2, color, thickness)

def main():
    parser = argparse.ArgumentParser(description="Calculate and visualize homography from four 2x2 checkerboards.")
    parser.add_argument('--img', type=str, required=True, help="Path to the input image.")
    parser.add_argument('--out', '-o', type=str, default="H.yaml", help="Output path for the homography YAML file (default: H.yaml).")
    parser.add_argument('--tl', nargs=2, type=float, required=True, help="World X Y for Top-Left board")
    parser.add_argument('--tr', nargs=2, type=float, required=True, help="World X Y for Top-Right board")
    parser.add_argument('--bl', nargs=2, type=float, required=True, help="World X Y for Bottom-Left board")
    parser.add_argument('--br', nargs=2, type=float, required=True, help="World X Y for Bottom-Right board")
    args = parser.parse_args()

    # 1. Parse world points
    world_pts = np.array([args.tl, args.tr, args.bl, args.br], dtype=np.float32)

    # 2. Extract image points and fetch the original image for visualization
    image_pts, original_img = find_2x2_checkerboard_centers(args.img)
    image_pts = np.array(image_pts, dtype=np.float32)

    print("\nMatched Coordinates:")
    labels = ["Top-Left", "Top-Right", "Bottom-Left", "Bottom-Right"]
    for i in range(4):
        print(f"{labels[i]}: Image (u,v) = ({image_pts[i][0]:.1f}, {image_pts[i][1]:.1f}) --> World (X,Y) = ({world_pts[i][0]}, {world_pts[i][1]})")

    # 3. Calculate Homography (Image -> World mapping)
    H, mask = cv2.findHomography(image_pts, world_pts)

    if H is None:
        print("Error: Could not calculate homography.")
        sys.exit(1)

    # 4. Save to target output file
    fs = cv2.FileStorage(args.out, cv2.FILE_STORAGE_WRITE)
    fs.write("homography", H)
    fs.release()
    print(f"\nSuccess! Homography matrix saved to: {args.out}")

    # 5. Calculate Inverse Homography (World -> Image mapping) for backprojection
    try:
        H_inv = np.linalg.inv(H)
    except np.linalg.LinAlgError:
        print("Error: Homography matrix is singular and cannot be inverted.")
        sys.exit(1)

    # 6. Generate and Draw the Backprojected Grid
    vis_img = original_img.copy()
    
    # Define bounds of the grid in world space based on inputs
    X_min, X_max = np.min(world_pts[:, 0]), np.max(world_pts[:, 0])
    Y_min, Y_max = np.min(world_pts[:, 1]), np.max(world_pts[:, 1])
    
    # Setup grid density (e.g., a 10x10 grid)
    grid_lines_count = 10
    xs = np.linspace(X_min, X_max, grid_lines_count)
    ys = np.linspace(Y_min, Y_max, grid_lines_count)
    
    print("\nGenerating backprojected grid overlay...")
    
    # Draw vertical lines (constant X, varying Y)
    for x in xs:
        p1 = np.array([x, Y_min])
        p2 = np.array([x, Y_max])
        draw_world_line(vis_img, p1, p2, H_inv, color=(0, 255, 0), thickness=2)
        
    # Draw horizontal lines (constant Y, varying X)
    for y in ys:
        p1 = np.array([X_min, y])
        p2 = np.array([X_max, y])
        draw_world_line(vis_img, p1, p2, H_inv, color=(0, 255, 0), thickness=2)

    # Draw the physical locations of the calibration points as small red dots for context
    for i, pt in enumerate(image_pts):
        cv2.circle(vis_img, (int(round(pt[0])), int(round(pt[1]))), 6, (0, 0, 255), -1)
        cv2.putText(vis_img, labels[i], (int(round(pt[0])) + 10, int(round(pt[1])) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

    # Save the visualization image next to the yaml file
    vis_filename = args.out.rsplit('.', 1)[0] + "_visualization.png"
    cv2.imwrite(vis_filename, vis_img)
    print(f"Visualization saved to: {vis_filename}")

    # Display the final output
    print("Displaying visualization window. Press any key in the window to exit...")
    cv2.namedWindow("Backprojected Grid Visualization", cv2.WINDOW_NORMAL)
    cv2.imshow("Backprojected Grid Visualization", vis_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()