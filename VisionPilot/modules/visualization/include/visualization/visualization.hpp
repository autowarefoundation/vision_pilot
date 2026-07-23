#ifndef VISIONPILOT_VISUALIZATION_HPP
#define VISIONPILOT_VISUALIZATION_HPP

#include <common/types.hpp>
#include <models/inference.hpp>
#include <opencv2/opencv.hpp>

#include <cstdint>
#include <string>
#include <vector>
#include <visualization/visual_interface.hpp>

namespace visualization {

// ─── Production renderer ─────────────────────────────────────────────────────
// Build from pipeline output + plan, then call render() or use visualize().

struct ProductionView {
    double               ego_speed_ms   = 0.0;
    double               speed_limit_ms = 0.0;   // 0 = not shown
    double               acceleration   = 0.0;
    std::vector<uint8_t> warnings;   // Warning enum values (FCW=1 … RLDW=4)

    // AutoDrive-only CIPO: AD detects in-path object but AutoSpeed has no bbox.
    bool  ad_cipo_only  = false;
    float ad_distance_m = 0.f;

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

    // World → display-pixel homography for path corridor projection.
    // Populated by from() when H_resized is provided; falls back to the
    // internal warped-BEV H when empty (legacy warped-frame mode).
    cv::Mat H_world2px;

    // H_resized (resized-px → world) used to build H_world2px = inv(H_resized).
    static ProductionView from(
        const visionpilot::models::InferenceFrameResult& result,
        const Plan& plan,
        double ego_speed_ms,
        const cv::Mat& H_resized = {},
        double speed_limit_ms = 0.0);

    // Draw production UI onto the display frame and show the window.
    // frame should be the resized frame when H_resized was supplied to from().
    cv::Mat render(cv::Mat& frame) const;

    // One-shot: from(result, plan, ego_v, H_resized) + render(frame).
    static cv::Mat visualize(
        cv::Mat& frame,
        const visionpilot::models::InferenceFrameResult& result,
        const Plan& plan,
        double ego_speed_ms,
        const cv::Mat& H_resized = {},
        double speed_limit_ms = 0.0);
};

struct Config
{
    bool webrtc_on = false;
    int webrtc_port;
    bool show_window = true;   // false → LocalDisplay runs headless (no window)
};

void init_production_assets(const std::string& icons_dir = "");

// Raw warped frame (e.g. before two-frame buffer is warm).
bool show_frame(
    const cv::Mat& frame,
    const std::string& window_name = "VisionPilot");


class Visualization
{
public:
    Visualization(Config cfg);
    ~Visualization() = default;

    std::unique_ptr<VisualInterface> visual_interface;
    cv::Mat build_frame(cv::Mat& frame,
        const visionpilot::models::InferenceFrameResult& result,
        const Plan& plan,
        double ego_speed_ms,
        const cv::Mat& H_resized,
        double speed_limit_ms = 0.0);
    bool render_frame(const cv::Mat& display_frame);
    bool stop() const;
};

}  // namespace visualization

#endif  // VISIONPILOT_VISUALIZATION_HPP
