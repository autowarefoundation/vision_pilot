#include <models/inference.hpp>

#include <common/utils.hpp>
#include <logging/logger.hpp>
#include <models/backend.hpp>
#include <models/merged_backend.hpp>
#include <models/split_backend.hpp>
#include <models/tensor_prep.hpp>

#include <cctype>
#include <chrono>
#include <stdexcept>
#include <vector>

namespace visionpilot::models {

namespace {

constexpr int CHW_SIZE = AutoDrive::CHW_SIZE;

// Case-insensitive match against a sentinel that is itself lower-case, so a
// typo like "Auto" or "NONE" is recognised as the sentinel rather than
// silently falling through to the explicit-path branch and failing later
// with a message that names neither the real mistake nor the valid choices.
bool ieq_sentinel(const std::string& value, const char* sentinel)
{
    size_t i = 0;
    for (; i < value.size() && sentinel[i] != '\0'; ++i) {
        if (std::tolower(static_cast<unsigned char>(value[i])) != sentinel[i])
            return false;
    }
    return i == value.size() && sentinel[i] == '\0';
}

}  // namespace

void LatencyStats::update(double pre_, double ad_, double as_, double asp_, double wall_)
{
    pre = pre_; ad = ad_; as = as_; asp = asp_; wall = wall_;
}

void LatencyStats::print() const
{
    const double total = pre + wall;
    const double fps   = total > 0 ? 1000.0 / total : 0.0;

    // A merged backend runs one fused session, so per-branch times do not
    // exist; ad carries the whole Run.
    if (as == 0.0 && asp == 0.0) {
        VP_INFO("Latency  pre=%.1f ms  merged=%.1f ms  wall=%.1f ms  %.0f fps",
                pre, ad, total, fps);
        return;
    }
    VP_INFO("Latency  pre=%.1f ms  AD=%.1f ms  AS=%.1f ms  ASp=%.1f ms  "
            "parallel=%.1f ms  wall=%.1f ms  %.0f fps",
            pre, ad, as, asp, wall, total, fps);
}

void LatencyStats::reset() { *this = {}; }

std::optional<MergedTarget> resolve_merged_target(const engine::Config& engine_cfg,
                                                   const Config&         cfg)
{
    if (!cfg.merged) {
        if (engine_cfg.provider == "renesas") {
            throw std::runtime_error(
                "[InferencePipeline] engine.provider = renesas requires "
                "model.merged = true. The Renesas execution provider permits "
                "one NPU session per process, so a three-session split would "
                "place two of the three networks on a silent CPU fallback.");
        }
        return std::nullopt;
    }

    const bool renesas = engine_cfg.provider == "renesas";
    const std::string model_or_dir =
        renesas ? engine_cfg.artifacts_dir : cfg.merged_path;
    if (model_or_dir.empty()) {
        throw std::runtime_error(
            renesas
                ? "[InferencePipeline] engine.artifacts_dir is required "
                  "when engine.provider = renesas"
                : "[InferencePipeline] model.merged_path is required when "
                  "model.merged = true");
    }

    std::string contract_path;
    if (ieq_sentinel(cfg.contract, "auto")) {
        contract_path = resolve_contract_path(
            renesas ? std::string{} : model_or_dir,
            renesas ? model_or_dir : std::string{});
    } else if (!ieq_sentinel(cfg.contract, "none")) {
        contract_path = cfg.contract;
    }

    return MergedTarget{model_or_dir, contract_path};
}

InferencePipeline::InferencePipeline(engine::OnnxEngine& engine, const Config& cfg)
{
    if (auto target = resolve_merged_target(engine.config(), cfg)) {
        backend_ = std::make_unique<MergedBackend>(engine, target->model_or_dir,
                                                   target->contract_path);
    } else {
        backend_ = std::make_unique<SplitBackend>(engine, cfg.precision);
    }

    fusion::LongitudinalFusion::Config lc = cfg.long_fusion;
    lc.debug = cfg.fusion_debug;
    long_fusion_ = fusion::LongitudinalFusion{lc};

    fusion::LateralFusion::Config latc;
    latc.debug      = cfg.fusion_debug;
    latc.cte_bias_m = cfg.cte_bias_m;
    lat_fusion_ = fusion::LateralFusion{latc};

    // Allocate the input tensors once. Each is 6 MB, and process() hands the
    // backend raw pointers into them, so they must not be reallocated.
    imn_[0].resize(CHW_SIZE);
    imn_[1].resize(CHW_SIZE);
    unit_.resize(CHW_SIZE);
}

// V matrix — warped BEV 1024×512 → world.  Matches lateral/longitudinal fusion H_.
// DO NOT MODIFY — must stay in sync with the hardcoded H_ in both fusion modules.
static const cv::Matx33d kV(
     0.00209514907, -0.000941721466, -9.24906396,
     0.00662758637, -0.000352940531, -3.33396502,
     0.000120077371, -0.00411343505,  1.0);

void InferencePipeline::set_H_resized(const cv::Mat& H, cv::Size raw_size)
{
    // H_resized: resized_px → world  (AutoSteer / AutoSpeed path)
    // Preprocessor: top-crop to 2:1, then resize → 1024×512.
    //   u_raw = u_r · (raw_w / 1024)
    //   v_raw = v_r · (crop_h / 512) + crop_top
    //   world = H × raw_px  ⟹  H_resized = H × T
    cv::Mat H64;
    H.convertTo(H64, CV_64F);

    const int crop_top = compute_top_crop_2_1(raw_size.height, raw_size.width);
    const double crop_h = static_cast<double>(raw_size.height - crop_top);
    const double sx = static_cast<double>(raw_size.width) / 1024.0;
    const double sy = crop_h / 512.0;

    const cv::Matx33d T(sx, 0,  0,
                        0,  sy, static_cast<double>(crop_top),
                        0,   0, 1);

    const cv::Mat H_resized = H64 * cv::Mat(T);

    H_resized_ = H_resized.clone();
    cv::Mat H64_inv = H_resized.inv();   // MatExpr → cv::Mat
    H64_inv.convertTo(H_world2resized_, CV_32F);
    lat_fusion_.set_H(H_resized_);
    long_fusion_.set_H(H_resized_);
    VP_INFO("[Pipeline] H_resized set — raw=%dx%d  top_crop=%d  sx=%.4f sy=%.4f",
            raw_size.width, raw_size.height, crop_top, sx, sy);
}

std::optional<InferenceFrameResult> InferencePipeline::process(const cv::Mat& warped,
                                                               const cv::Mat& resized,
                                                               float ego_speed_ms,
                                                               bool has_ego_speed)
{
    using Clock = std::chrono::steady_clock;
    using Ms    = std::chrono::duration<double, std::milli>;

    // Two-frame buffer is warped (for AutoDrive only)
    prev_frame_ = curr_frame_.empty() ? warped.clone() : curr_frame_;
    curr_frame_ = warped.clone();
    if (frame_buf_count_ < 1) frame_buf_count_ = 1;
    else                       frame_buf_count_ = 2;

    ++frame_count_;
    if (frame_buf_count_ < 2) return std::nullopt;

    // AutoSteer + AutoSpeed use resized if provided, else fall back to warped
    const cv::Mat& as_input = (!resized.empty()) ? resized : warped;

    auto t0 = Clock::now();

    // prev_frame_ is the very cv::Mat that was curr_frame_ on the previous
    // call, so its ImageNet tensor is bit for bit the one computed then. Keep
    // two slots and swap rather than converting the same image twice.
    const int curr_slot = imn_curr_;
    const int prev_slot = 1 - curr_slot;
    float* prev_imn   = imn_[prev_slot].data();
    float* curr_imn   = imn_[curr_slot].data();
    // AutoSteer and AutoSpeed take the same image, so they share one tensor.
    float* curr_01_as = unit_.data();

    // prev_frame_ holds a reference to that pixel buffer for as long as this
    // comparison lasts, so a match cannot be a recycled allocation: it really
    // is the image converted last call. Anything else -- the first inferred
    // frame, or a future change to the frame buffering above -- just converts
    // it again.
    const bool prev_is_cached =
        imn_prev_src_ != nullptr && imn_prev_src_ == prev_frame_.data;
    if (!prev_is_cached)
        to_chw_imagenet(prev_frame_, prev_imn, imn_[prev_slot].size());
    to_chw_imagenet(curr_frame_, curr_imn, imn_[curr_slot].size());
    to_chw_unit(as_input, curr_01_as, unit_.size());

    // This frame's curr tensor is next frame's prev. Swap now, while the state
    // still matches the frame buffers -- a backend failure below does not
    // invalidate the conversion that has already happened.
    imn_curr_     = prev_slot;
    imn_prev_src_ = curr_frame_.data;

    const double ms_pre = Ms(Clock::now() - t0).count();

    if (!offload_verified_) {
        if (auto* merged = dynamic_cast<MergedBackend*>(backend_.get())) {
            merged->verify_offload(prev_imn, curr_imn, curr_01_as);
        }
        // Set only once the gate has actually passed. Setting it first would
        // let any caller that catches per-frame exceptions skip the gate for
        // the rest of the run and drive on exactly the silent CPU fallback
        // the gate exists to catch.
        offload_verified_ = true;
    }

    auto t_wall = Clock::now();
    const BackendOutputs r = backend_->run(prev_imn, curr_imn, curr_01_as);
    const double ms_wall = Ms(Clock::now() - t_wall).count();

    InferenceFrameResult out;
    out.frame_id   = frame_count_;
    out.wall_ms    = ms_wall;
    out.pre_ms     = ms_pre;
    out.ad_ms      = r.ad_ms;
    out.as_ms      = r.as_ms;
    out.asp_ms     = r.asp_ms;
    out.auto_drive = r.drive;
    out.auto_steer = r.steer;
    out.auto_speed = r.speed;

    out.lateral = lat_fusion_.update(r.steer, r.drive);

    const bool radar_on = long_fusion_.config().radar_enabled;
    fusion::PathPoly path;
    if (radar_on && out.lateral.path_valid) {
        path.valid = true;
        path.a = out.lateral.path_a;
        path.b = out.lateral.path_b;
        path.c = out.lateral.path_c;
        path.x_max_m = out.lateral.path_x_max_m;
    }
    const std::vector<RadarPoint>* radar_ptr =
        radar_on ? &radar_points_ : nullptr;
    const fusion::PathPoly* path_ptr =
        (radar_on && path.valid) ? &path : nullptr;
    const float* ego_ptr = has_ego_speed ? &ego_speed_ms : nullptr;
    out.cipo = long_fusion_.update(r.drive, r.speed, warped, 0.f, radar_ptr, path_ptr, ego_ptr);

    stats_.update(ms_pre, r.ad_ms, r.as_ms, r.asp_ms, ms_wall);
    return out;
}

void InferencePipeline::reset()
{
    prev_frame_.release();
    curr_frame_.release();
    // The cached tensor belongs to the frames just dropped, so it must go with
    // them or the next first frame would be paired with a stale predecessor.
    imn_prev_src_ = nullptr;
    imn_curr_     = 0;
    frame_buf_count_ = 0;
    frame_count_ = 0;
    stats_.reset();
    long_fusion_.reset();
    lat_fusion_.reset();
    radar_points_.clear();
}

}  // namespace visionpilot::models
