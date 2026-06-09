#pragma once

#include <fusion/lateral_fusion.hpp>
#include <fusion/longitudinal_fusion.hpp>
#include <models/auto_drive.hpp>
#include <models/auto_steer.hpp>
#include <models/auto_speed.hpp>
#include <opencv2/core.hpp>

#include <cstdint>
#include <optional>
#include <string>

namespace visionpilot::engine {
class OnnxEngine;
}

namespace visionpilot::models {

struct InferenceConfig {
    std::string autodrive_model;
    std::string autosteer_model;
    std::string autospeed_model;
    std::string homography_path;
    bool        fusion_debug = false;
};

struct LatencyStats {
    double pre{0}, ad{0}, as{0}, asp{0}, wall{0};

    void update(double pre_, double ad_, double as_, double asp_, double wall_);
    void print() const;
    void reset();
};

struct InferenceFrameResult {
    uint64_t    frame_id = 0;
    double      wall_ms  = 0;
    double      pre_ms   = 0;
    double      ad_ms    = 0;
    double      as_ms    = 0;
    double      asp_ms   = 0;

    AutoDriveOutput              auto_drive;
    AutoSteerOutput              auto_steer;
    AutoSpeedOutput              auto_speed;
    fusion::CIPOFusionEstimate   cipo;
    fusion::LateralFusionEstimate  lateral;
};

// Two-frame buffer → parallel ONNX → longitudinal + lateral fusion.
class InferencePipeline {
public:
    InferencePipeline(engine::OnnxEngine& engine, const InferenceConfig& cfg);

    // nullopt until two frames collected (AutoDrive needs t-1 and t).
    std::optional<InferenceFrameResult> process(const cv::Mat& preprocessed);

    void reset();
    const LatencyStats& latency() const { return stats_; }

private:
    AutoDrive          auto_drive_;
    AutoSteer          auto_steer_;
    AutoSpeed          auto_speed_;
    fusion::LongitudinalFusion long_fusion_;
    fusion::LateralFusion      lat_fusion_;
    LatencyStats       stats_;
    uint64_t           frame_count_ = 0;

    cv::Mat prev_frame_;
    cv::Mat curr_frame_;
    int     frame_buf_count_ = 0;
};

}  // namespace visionpilot::models
