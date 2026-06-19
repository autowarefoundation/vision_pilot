#include <models/inference.hpp>

#include <logging/logger.hpp>

#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cstring>
#include <future>
#include <utility>
#include <vector>

namespace visionpilot::models {

namespace {

constexpr int NET_W    = AutoDrive::NET_W;
constexpr int NET_H    = AutoDrive::NET_H;
constexpr int CHW_SIZE = AutoDrive::CHW_SIZE;

constexpr float MEAN[3] = {0.485f, 0.456f, 0.406f};
constexpr float STD[3]  = {0.229f, 0.224f, 0.225f};

std::vector<float> chw_imagenet(const cv::Mat& bgr)
{
    cv::Mat rgb, f32;
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    rgb.convertTo(f32, CV_32FC3, 1.0 / 255.0);
    std::vector<cv::Mat> ch(3);
    cv::split(f32, ch);
    std::vector<float> out(CHW_SIZE);
    for (int c = 0; c < 3; ++c) {
        float* dst = out.data() + c * NET_H * NET_W;
        const float* src = reinterpret_cast<const float*>(ch[c].data);
        for (int i = 0; i < NET_H * NET_W; ++i)
            dst[i] = (src[i] - MEAN[c]) / STD[c];
    }
    return out;
}

std::vector<float> chw_01(const cv::Mat& bgr)
{
    cv::Mat rgb, f32;
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    rgb.convertTo(f32, CV_32FC3, 1.0 / 255.0);
    std::vector<cv::Mat> ch(3);
    cv::split(f32, ch);
    std::vector<float> out(CHW_SIZE);
    for (int c = 0; c < 3; ++c)
        std::memcpy(out.data() + c * NET_H * NET_W, ch[c].data,
                    static_cast<std::size_t>(NET_H * NET_W) * sizeof(float));
    return out;
}

}  // namespace

void LatencyStats::update(double pre_, double ad_, double as_, double asp_, double wall_)
{
    pre = pre_; ad = ad_; as = as_; asp = asp_; wall = wall_;
}

void LatencyStats::print() const
{
    const double total = pre + wall;
    VP_INFO("Latency  pre=%.1f ms  AD=%.1f ms  AS=%.1f ms  ASp=%.1f ms  "
            "parallel=%.1f ms  wall=%.1f ms  %.0f fps",
            pre, ad, as, asp, wall, total, total > 0 ? 1000.0 / total : 0.0);
}

void LatencyStats::reset() { *this = {}; }

InferencePipeline::InferencePipeline(engine::OnnxEngine& engine, const InferenceConfig& cfg)
    : auto_drive_(engine, valid_model_path("modules/models/weights/autodrive_" + cfg.precision + ".onnx"))
    , auto_steer_(engine, valid_model_path("modules/models/weights/autosteer_" + cfg.precision + ".onnx"))
    , auto_speed_(engine, valid_model_path("modules/models/weights/autospeed_" + cfg.precision + ".onnx"))
{
    fusion::LongitudinalFusion::Config lc;
    lc.debug           = cfg.fusion_debug;
    long_fusion_ = fusion::LongitudinalFusion{lc};

    fusion::LateralFusion::Config latc;
    latc.debug           = cfg.fusion_debug;
    lat_fusion_ = fusion::LateralFusion{latc};
}

std::optional<InferenceFrameResult> InferencePipeline::process(const cv::Mat& preprocessed)
{
    using Clock = std::chrono::steady_clock;
    using Ms    = std::chrono::duration<double, std::milli>;

    prev_frame_ = curr_frame_.empty() ? preprocessed.clone() : curr_frame_;
    curr_frame_ = preprocessed.clone();
    if (frame_buf_count_ < 1) frame_buf_count_ = 1;
    else                       frame_buf_count_ = 2;

    ++frame_count_;
    if (frame_buf_count_ < 2) return std::nullopt;

    auto t0       = Clock::now();
    auto prev_imn    = chw_imagenet(prev_frame_);
    auto curr_imn    = chw_imagenet(curr_frame_);
    auto curr_01_as  = chw_01(curr_frame_);          // AutoSteer's private copy
    auto curr_01_asp = curr_01_as;                   // AutoSpeed's private copy
    const double ms_pre = Ms(Clock::now() - t0).count();

    auto t_wall = Clock::now();
    auto f_drive = std::async(std::launch::async, [&] {
        auto t = Clock::now();
        auto r = auto_drive_.infer(prev_imn.data(), curr_imn.data());
        return std::make_pair(std::move(r), Ms(Clock::now() - t).count());
    });
    auto f_steer = std::async(std::launch::async, [&] {
        auto t = Clock::now();
        auto r = auto_steer_.infer(curr_01_as.data());
        return std::make_pair(std::move(r), Ms(Clock::now() - t).count());
    });
    auto f_speed = std::async(std::launch::async, [&] {
        auto t = Clock::now();
        auto r = auto_speed_.infer(curr_01_asp.data());
        return std::make_pair(std::move(r), Ms(Clock::now() - t).count());
    });

    auto [res_drive, ms_drive] = f_drive.get();
    auto [res_steer, ms_steer] = f_steer.get();
    auto [res_speed, ms_speed] = f_speed.get();
    const double ms_wall = Ms(Clock::now() - t_wall).count();

    InferenceFrameResult out;
    out.frame_id   = frame_count_;
    out.wall_ms    = ms_wall;
    out.pre_ms     = ms_pre;
    out.ad_ms      = ms_drive;
    out.as_ms      = ms_steer;
    out.asp_ms     = ms_speed;
    out.auto_drive = res_drive;
    out.auto_steer = res_steer;
    out.auto_speed = res_speed;
    out.cipo       = long_fusion_.update(res_drive, res_speed, preprocessed);
    out.lateral    = lat_fusion_.update(res_steer, res_drive);

    stats_.update(ms_pre, ms_drive, ms_steer, ms_speed, ms_wall);
    return out;
}

void InferencePipeline::reset()
{
    prev_frame_.release();
    curr_frame_.release();
    frame_buf_count_ = 0;
    frame_count_ = 0;
    stats_.reset();
    long_fusion_.reset();
    lat_fusion_.reset();
}

}  // namespace visionpilot::models
