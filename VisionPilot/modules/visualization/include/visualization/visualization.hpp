#ifndef VISIONPILOT_VISUALIZATION_HPP
#define VISIONPILOT_VISUALIZATION_HPP

#include <common/types.hpp>
#include <models/inference.hpp>
#include <opencv2/opencv.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace visualization {

// ─── Production renderer ─────────────────────────────────────────────────────
// Build from pipeline output + plan, then call render() or use visualize().

struct ProductionView {
    double               ego_speed_ms = 0.0;
    double               acceleration = 0.0;
    std::vector<uint8_t> warnings;   // Warning enum values (FCW=1 … RLDW=4)

    float path_a       = 0.f;
    float path_b       = 0.f;
    float path_c       = 0.f;
    float path_x_min_m = 2.f;
    float path_x_max_m = 0.f;
    bool  path_valid   = false;

    struct BBox {
        float x1 = 0.f, y1 = 0.f, x2 = 0.f, y2 = 0.f;
        float score = 0.f;
        int   class_id = 0;
    };
    std::vector<BBox> detections;

    struct CIPOState {
        bool  valid = false;
        float distance_m = 0.f;
        float velocity_ms = 0.f;
        bool  cipo_raw_found = false;
        float cipo_raw_dist_m = 0.f;
        bool  cut_in_detected = false;
    };
    CIPOState cipo;

    std::string icons_dir;

    static ProductionView from(
        const visionpilot::models::InferenceFrameResult& result,
        const Plan& plan,
        double ego_speed_ms);

    // Draw production UI onto warped BGR frame and show the window.
    bool render(cv::Mat& frame) const;

    // One-shot: from(result, plan, ego_v) + render(frame).
    static bool visualize(
        cv::Mat& frame,
        const visionpilot::models::InferenceFrameResult& result,
        const Plan& plan,
        double ego_speed_ms);
};

void init_production_assets(const std::string& icons_dir = "");

// Raw warped frame (e.g. before two-frame buffer is warm).
bool show_frame(
    const cv::Mat& frame,
    const std::string& window_name = "VisionPilot");

void close_windows();

}  // namespace visualization

#endif  // VISIONPILOT_VISUALIZATION_HPP
