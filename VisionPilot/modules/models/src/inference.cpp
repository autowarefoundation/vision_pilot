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

std::string find_model(const std::string& filename) {
    const std::string local  = "modules/models/weights/" + filename;
    const std::string system = "/usr/share/visionpilot/modules/models/weights/" + filename;

    if (std::filesystem::exists(local))  return local;
    if (std::filesystem::exists(system)) return system;

    throw std::runtime_error("Config file not found: " + filename);
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

InferencePipeline::InferencePipeline(engine::OnnxEngine& engine, const Config& cfg)
    : auto_drive_(engine, find_model("autodrive_" + cfg.precision + ".onnx"))
    , auto_steer_(engine, find_model("autosteer_" + cfg.precision + ".onnx"))
    , auto_speed_(engine, find_model("autospeed_" + cfg.precision + ".onnx"))
{
    fusion::LongitudinalFusion::Config lc;
    lc.debug           = cfg.fusion_debug;
    long_fusion_ = fusion::LongitudinalFusion{lc};

    fusion::LateralFusion::Config latc;
    latc.debug      = cfg.fusion_debug;
    latc.cte_bias_m = cfg.cte_bias_m;
    lat_fusion_ = fusion::LateralFusion{latc};
}

// V matrix — warped BEV 1024×512 → world.  Matches lateral/longitudinal fusion H_.
// DO NOT MODIFY — must stay in sync with the hardcoded H_ in both fusion modules.
static const cv::Matx33d kV(
     0.00209514907, -0.000941721466, -9.24906396,
     0.00662758637, -0.000352940531, -3.33396502,
     0.000120077371, -0.00411343505,  1.0);

void InferencePipeline::set_H_resized(const cv::Mat& H, cv::Size raw_size)
{
    // H_resized: resized_px → world
    //   world = V × C × raw_px
    //   raw_px = inv(T_resize) × resized_px   where T_resize scales raw to 1024×512
    //   ⟹  H_resized = V × C × inv(T_resize)
    cv::Mat H64;
    H.convertTo(H64, CV_64F);

    int input_image_w = raw_size.width;
    int input_image_h = raw_size.height;
    double crop_h = (input_image_h - input_image_w / 2) / 2;

    const double sx = 1024.0 / raw_size.width;
    const double sy =  512.0 / raw_size.height;
    // inv(T_resize) — maps 1024×512 px back to raw px
    // const cv::Matx33d T_inv(1.0/sx, 0,      0,
    //                          0,      1.0/sy,  0,
    //                          0,      0,       1);

    const cv::Matx33d T(1.0, 0,    0,
                        0,   1.0,  crop_h,
                        0,   0,    1);

    const cv::Mat H_resized = H64 * cv::Mat(T);

    H_resized_ = H_resized.clone();
    cv::Mat H64_inv = H_resized.inv();   // MatExpr → cv::Mat
    H64_inv.convertTo(H_world2resized_, CV_32F);
    lat_fusion_.set_H(H_resized_);
    long_fusion_.set_H(H_resized_);
    VP_INFO("[Pipeline] H_resized set — raw=%dx%d  sx=%.4f sy=%.4f",
            raw_size.width, raw_size.height, sx, sy);
}

std::optional<InferenceFrameResult> InferencePipeline::process(const cv::Mat& warped,
                                                               const cv::Mat& resized)
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

    auto t0          = Clock::now();
    auto prev_imn    = chw_imagenet(prev_frame_);
    auto curr_imn    = chw_imagenet(curr_frame_);
    auto curr_01_as  = chw_01(as_input);
    auto curr_01_asp = curr_01_as;   // shared preprocessing (same image)
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
    out.cipo       = long_fusion_.update(res_drive, res_speed, warped);
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
