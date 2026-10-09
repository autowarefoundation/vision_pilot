#pragma once

#include <engine/onnx_engine.hpp>
#include <fusion/lateral_fusion.hpp>
#include <fusion/longitudinal_fusion.hpp>
#include <models/auto_drive.hpp>
#include <models/auto_steer.hpp>
#include <models/auto_speed.hpp>
#include <models/backend.hpp>
#include <opencv2/core.hpp>

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace visionpilot::engine {
class OnnxEngine;
}

namespace visionpilot::models {

struct Config {
    std::string precision    = "fp32";
    bool        fusion_debug = false;
    float       cte_bias_m   = 0.0f;  // camera mounting offset [m] — subtracted from raw CTE before filter
    // Run all three networks in one session over a merged graph. Required for
    // the renesas provider, which permits only one NPU session per process.
    bool        merged = false;
    // Merged .onnx path. Ignored under the renesas provider, which uses
    // engine.artifacts_dir instead.
    std::string merged_path;
    // "auto" resolves <merged_path>.contract.json or
    // <artifacts_dir>/contract.json. "none" forces plain-merged mode.
    // Anything else is treated as an explicit contract path.
    std::string contract = "auto";
};

// The MergedBackend construction target, resolved purely from configuration
// -- no ORT session is touched. std::nullopt means "construct a
// SplitBackend instead"; resolving that case also refuses
// engine_cfg.provider == "renesas", since a three-session split would place
// two of the three networks on a silent CPU fallback.
//
// model_or_dir is engine_cfg.artifacts_dir under the renesas provider, else
// cfg.merged_path; either being empty is a startup error naming the
// missing key. contract_path follows cfg.contract, matched
// case-insensitively against the "auto"/"none" sentinels so a typo like
// "Auto" is not silently misread as a literal path: "auto" resolves via
// resolve_contract_path() (against the artifacts directory under renesas,
// else the merged_path's ".contract.json" sidecar), "none" yields ""
// (forces plain-merged mode), and anything else is passed through as an
// explicit path.
//
// Throws std::runtime_error naming the missing config key, or the refused
// engine.provider = renesas / model.merged combination.
struct MergedTarget {
    std::string model_or_dir;
    std::string contract_path;
};

std::optional<MergedTarget> resolve_merged_target(const engine::Config& engine_cfg,
                                                   const Config&         cfg);

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
    InferencePipeline(engine::OnnxEngine& engine, const Config& cfg);

    // nullopt until two frames collected (AutoDrive needs t-1 and t).
    // warped  : BEV 1024×512 image → AutoDrive only.
    // resized : plain-resized 1024×512 image → AutoSteer + AutoSpeed.
    //           If empty, falls back to warped for all networks (legacy behaviour).
    std::optional<InferenceFrameResult> process(const cv::Mat& warped,
                                                const cv::Mat& resized = {});

    // Compute and apply H_resized to both fusion modules so that AutoSteer /
    // AutoSpeed outputs are projected correctly when they run on a resized
    // (non-BEV) image.  Call once after the first preprocess() when using
    // the resized routing.
    //   C        : raw-camera → warped-BEV homography (from ImagePreprocessor)
    //   raw_size : original frame dimensions before top-crop + resize
    void set_H_resized(const cv::Mat& H, cv::Size raw_size);

    // H that maps resized-image pixel → world (set after set_H_resized()).
    // Empty until set_H_resized() is called.
    const cv::Mat& H_resized() const { return H_resized_; }

    // Inverse of H_resized: world → resized-image pixel.
    // Used by both debug and production visualizers for path projection.
    const cv::Mat& H_world2resized() const { return H_world2resized_; }

    void reset();
    const LatencyStats& latency() const { return stats_; }

private:
    cv::Mat H_resized_;
    cv::Mat H_world2resized_;
    std::unique_ptr<ModelBackend> backend_;
    fusion::LongitudinalFusion long_fusion_;
    fusion::LateralFusion      lat_fusion_;
    LatencyStats       stats_;
    uint64_t           frame_count_ = 0;

    cv::Mat prev_frame_;
    cv::Mat curr_frame_;
    int     frame_buf_count_ = 0;

    // Input tensors, allocated once in the constructor and reused: each is
    // 6 MB at 1024x512, and value-initialising three fresh ones per frame cost
    // more than the conversion that fills them. process() hands the backend
    // raw pointers into these, so they are never resized after construction.
    //
    // imn_ is a two-slot ping-pong: the ImageNet tensor computed for frame N
    // is exactly the previous-frame input that frame N+1 needs, so imn_curr_
    // names the slot this frame writes and the other already holds the last
    // one.
    //
    // imn_prev_src_ is the pixel buffer that cached tensor was built from, and
    // the cache is trusted only while it still matches prev_frame_.data. That
    // makes the reuse self-checking rather than resting on the two-frame
    // buffering in process() keeping its current shape: if prev_frame_ ever
    // stops being the cv::Mat that was curr_frame_ on the previous call, the
    // pointers differ and the tensor is simply recomputed. It is null before
    // the first inferred frame and after reset().
    std::vector<float>   imn_[2];
    std::vector<float>   unit_;
    int                  imn_curr_     = 0;
    const unsigned char* imn_prev_src_ = nullptr;

    bool offload_verified_ = false;
};

}  // namespace visionpilot::models
