#pragma once

#include <tracking/kalman_filter.hpp>
#include <tracking/tracking_utils.hpp>
#include <tracking/cipo_history.hpp>
#include <tracking/feature_matching_utils.hpp>

#include <models/auto_speed.hpp>   // visionpilot::models::Detection

#include <opencv2/opencv.hpp>
#include <chrono>
#include <string>
#include <vector>

namespace visionpilot::tracking {

// ─── Per-track state ───────────────────────────────────────────────────────────
struct TrackedObject {
    int   track_id;
    int   class_id;
    float confidence;
    cv::Rect bbox;
    float distance_m;
    float velocity_ms;   // negative = closing
    int   frames_tracked;
    int   frames_unmatched;
    KalmanFilter kalman;
    std::chrono::steady_clock::time_point last_update_time;
};

// ─── CIPO output ───────────────────────────────────────────────────────────────
struct CIPOInfo {
    bool  exists     = false;
    int   track_id   = -1;
    int   class_id   = -1;
    float distance_m = 0.f;
    float velocity_ms = 0.f;
};

// ─── Full tracking result (atomic — tracked_objects and cipo are always in sync) ─
struct TrackingResult {
    std::vector<TrackedObject> tracked_objects;
    CIPOInfo cipo;
    bool cut_in_detected = false;
    bool kalman_reset    = false;
};

// ─── ObjectFinder ──────────────────────────────────────────────────────────────
//
// Multi-object tracker fusing:
//   1. AutoSpeed YOLO detections (input: scaled back to original image resolution)
//   2. Homography-based distance estimation  (bottom-centre (u,v) → (X,Y) world)
//   3. Per-track 1D Kalman filter            ([distance, velocity] state)
//   4. ORB feature matching for cut-in detection
//
// Usage per frame:
//   TrackingResult r = tracker.update_and_get_cipo(detections, original_frame);
//
class ObjectFinder {
public:
    // homography_yaml : path to YAML containing H (3×3 matrix)
    // image_w/h      : original frame dimensions (for matching score normalisation)
    // debug          : verbose per-frame logging
    ObjectFinder(const std::string& homography_yaml,
                 int image_w = 1920,
                 int image_h = 1080,
                 bool debug  = false);

    // Combines tracking update + CIPO selection atomically.
    // Detections must already be in original-frame pixel coordinates.
    TrackingResult update_and_get_cipo(
        const std::vector<visionpilot::models::Detection>& detections,
        const cv::Mat& frame);

private:
    cv::Point2f image_to_world(const cv::Point2f& pt) const;
    float       distance(const cv::Point2f& world_pt) const;

    std::vector<std::pair<int,int>> associate(
        const std::vector<visionpilot::models::Detection>& dets) const;

    bool should_track(int class_id) const {
        return class_id == 1 || class_id == 2;
    }

    cv::Mat H_;                           // homography float32 3×3
    std::vector<TrackedObject> tracks_;   // current tracks
    std::vector<TrackedObject> prev_;     // tracks from last frame (for association)
    CIPOHistory history_{30};

    int   next_id_          = 0;
    int   image_w_          = 1920;
    int   image_h_          = 1080;
    float match_threshold_  = 0.55f;
    int   max_unmatched_    = 3;
    float feat_threshold_   = 0.30f;
    bool  debug_            = false;

    bool cut_in_detected_   = false;
    bool kalman_reset_      = false;
};

}  // namespace visionpilot::tracking
