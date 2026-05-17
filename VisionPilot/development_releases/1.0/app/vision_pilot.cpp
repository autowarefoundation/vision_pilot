#include "vision_pilot_config.hpp"

#include <camera_subscriber/ros2_to_opencv.hpp>
#include <v4l2_interface/v4l2_reader.hpp>
#include <visualization/visualization.hpp>

#include <engine/onnx_engine.hpp>
#include <models/auto_drive.hpp>
#include <models/auto_steer.hpp>
#include <models/auto_speed.hpp>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <future>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace {

// First argv token that is not a config flag (expected: capture mode 0 or 1).
int index_of_mode_arg(int argc, char** argv)
{
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--config" || arg == "-c") {
            ++i;
            continue;
        }
        if (arg.rfind("--config=", 0) == 0) {
            continue;
        }
        return i;
    }
    return -1;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Latency
// ─────────────────────────────────────────────────────────────────────────────

using Clock    = std::chrono::steady_clock;
using Ms       = std::chrono::duration<double, std::milli>;

struct FrameLatency {
    double preprocess_ms  = 0.0;  // CHW conversion before launch
    double autodrive_ms   = 0.0;  // AutoDrive wall time (inside thread)
    double autosteer_ms   = 0.0;
    double autospeed_ms   = 0.0;
    double pipeline_ms    = 0.0;  // wall time from dispatch to last .get()
};

// Rolling stats (exponential moving average, α = 0.1)
struct LatencyStats {
    double ema_preprocess  = 0.0;
    double ema_autodrive   = 0.0;
    double ema_autosteer   = 0.0;
    double ema_autospeed   = 0.0;
    double ema_pipeline    = 0.0;
    uint64_t n = 0;

    static constexpr double ALPHA = 0.1;

    void update(const FrameLatency& f)
    {
        if (n == 0) {
            ema_preprocess = f.preprocess_ms;
            ema_autodrive  = f.autodrive_ms;
            ema_autosteer  = f.autosteer_ms;
            ema_autospeed  = f.autospeed_ms;
            ema_pipeline   = f.pipeline_ms;
        } else {
            ema_preprocess = ALPHA * f.preprocess_ms + (1.0 - ALPHA) * ema_preprocess;
            ema_autodrive  = ALPHA * f.autodrive_ms  + (1.0 - ALPHA) * ema_autodrive;
            ema_autosteer  = ALPHA * f.autosteer_ms  + (1.0 - ALPHA) * ema_autosteer;
            ema_autospeed  = ALPHA * f.autospeed_ms  + (1.0 - ALPHA) * ema_autospeed;
            ema_pipeline   = ALPHA * f.pipeline_ms   + (1.0 - ALPHA) * ema_pipeline;
        }
        ++n;
    }

    void print() const
    {
        // pipeline_ms = wall clock while all three models run in parallel
        // (≈ max(drive, steer, speed), not their sum)
        const double parallel_max = std::max({ema_autodrive, ema_autosteer, ema_autospeed});
        printf("[Latency EMA] preprocess=%.2f ms | "
               "drive=%.2f  steer=%.2f  speed=%.2f ms (parallel) | "
               "wall=%.2f ms  max_model=%.2f ms  (%.1f fps)\n",
               ema_preprocess,
               ema_autodrive, ema_autosteer, ema_autospeed,
               ema_pipeline, parallel_max,
               ema_pipeline > 0.0 ? 1000.0 / ema_pipeline : 0.0);
    }

    void reset() { *this = {}; }
};

// ─────────────────────────────────────────────────────────────────────────────
// Per-frame result bundle
// ─────────────────────────────────────────────────────────────────────────────

struct FrameOutputs {
    visionpilot::models::AutoDriveOutput auto_drive;
    visionpilot::models::AutoSteerOutput auto_steer;
    visionpilot::models::AutoSpeedOutput auto_speed;
    uint64_t     frame_id = 0;
    FrameLatency latency;
};

// ─────────────────────────────────────────────────────────────────────────────
// InferencePipeline
//
// Owns the 2-slot circular frame buffer and the three model instances.
// Caller is responsible for spatial pre-processing (warp/crop/resize to
// NET_W × NET_H) before calling process().  Normalization to CHW float32
// is done here, once per normalization variant, before any threads start.
//
// Parallelism:
//   All three models run concurrently via std::async.
//   process() blocks until the slowest model finishes:
//     wall-time ≈ max(T_autodrive, T_autosteer, T_autospeed)
//
// Thread safety:
//   process() is NOT thread-safe; call from a single capture/inference thread.
// ─────────────────────────────────────────────────────────────────────────────

class InferencePipeline {
public:
    static constexpr int NET_H    = 512;
    static constexpr int NET_W    = 1024;
    static constexpr int CHW_SIZE = 3 * NET_H * NET_W;  // 1 572 864 floats

    InferencePipeline(visionpilot::engine::OnnxEngine& ort_engine, const VisionPilotConfig& cfg)
        : auto_drive_(ort_engine, cfg.autodrive_model)
        , auto_steer_(ort_engine, cfg.autosteer_model)
        , auto_speed_(ort_engine, cfg.autospeed_model)
    {}

    // Push a pre-processed BGR frame (must already be NET_W × NET_H).
    // Returns nullopt on the very first call — AutoDrive needs prev + curr.
    // record_latency=false for GPU warmup passes (not counted in EMA).
    std::optional<FrameOutputs> process(const cv::Mat& frame_bgr, bool record_latency = true)
    {
        // ── 1. Write into the circular buffer ────────────────────────────
        // Slots alternate: slot 0 ↔ slot 1 every frame.
        // After the write:
        //   frame_buffer_[write_idx_]          = current frame
        //   frame_buffer_[(write_idx_+1) % 2]  = previous frame
        write_idx_ = (write_idx_ + 1) % 2;
        frame_buffer_[write_idx_] = frame_bgr.clone();
        ++frame_count_;

        if (frame_count_ < 2) {
            // First frame stored; no previous frame yet — skip inference.
            return std::nullopt;
        }

        const int prev_idx = (write_idx_ + 1) % 2;

        // ── 2. Build CHW tensors ──────────────────────────────────────────
        auto t_pre_start = Clock::now();
        std::vector<float> prev_imagenet = to_chw_imagenet(frame_buffer_[prev_idx]);
        std::vector<float> curr_imagenet = to_chw_imagenet(frame_buffer_[write_idx_]);
        std::vector<float> curr_01       = to_chw_01(frame_buffer_[write_idx_]);
        const double preprocess_ms = Ms(Clock::now() - t_pre_start).count();

        // ── 3. Launch all three inferences concurrently ──────────────────
        auto t_dispatch = Clock::now();

        auto f_drive = std::async(std::launch::async, [&] {
            auto t0 = Clock::now();
            auto result = auto_drive_.infer(prev_imagenet.data(), curr_imagenet.data());
            return std::make_pair(result, Ms(Clock::now() - t0).count());
        });

        auto f_steer = std::async(std::launch::async, [&] {
            auto t0 = Clock::now();
            auto result = auto_steer_.infer(curr_01.data());
            return std::make_pair(result, Ms(Clock::now() - t0).count());
        });

        auto f_speed = std::async(std::launch::async, [&] {
            auto t0 = Clock::now();
            auto result = auto_speed_.infer(curr_01.data());
            return std::make_pair(result, Ms(Clock::now() - t0).count());
        });

        // ── 4. Barrier — wall time = max(T_drive, T_steer, T_speed) ──────
        auto [res_drive, ms_drive] = f_drive.get();
        auto [res_steer, ms_steer] = f_steer.get();
        auto [res_speed, ms_speed] = f_speed.get();
        const double pipeline_ms = Ms(Clock::now() - t_dispatch).count();

        FrameOutputs out;
        out.auto_drive       = res_drive;
        out.auto_steer       = res_steer;
        out.auto_speed       = res_speed;
        out.frame_id         = frame_count_;
        out.latency = { preprocess_ms, ms_drive, ms_steer, ms_speed, pipeline_ms };
        if (record_latency) {
            latency_stats_.update(out.latency);
        }

        return out;
    }

    void reset()
    {
        frame_buffer_[0].release();
        frame_buffer_[1].release();
        write_idx_       = 1;
        frame_count_     = 0;
        latency_stats_.reset();
    }

    const LatencyStats& latency_stats() const { return latency_stats_; }

private:
    // ── Normalization helpers ─────────────────────────────────────────────

    static constexpr float MEAN[3] = {0.485f, 0.456f, 0.406f};
    static constexpr float STD[3]  = {0.229f, 0.224f, 0.225f};

    // BGR → RGB → float32 → ImageNet normalise → CHW
    static std::vector<float> to_chw_imagenet(const cv::Mat& bgr)
    {
        cv::Mat rgb;
        cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);

        cv::Mat f32;
        rgb.convertTo(f32, CV_32FC3, 1.0 / 255.0);

        std::vector<cv::Mat> ch(3);
        cv::split(f32, ch);

        std::vector<float> out(CHW_SIZE);
        for (int c = 0; c < 3; ++c) {
            float*       dst = out.data() + c * NET_H * NET_W;
            const float* src = reinterpret_cast<const float*>(ch[c].data);
            for (int i = 0; i < NET_H * NET_W; ++i)
                dst[i] = (src[i] - MEAN[c]) / STD[c];
        }
        return out;
    }

    // BGR → RGB → float32 [0, 1] → CHW
    static std::vector<float> to_chw_01(const cv::Mat& bgr)
    {
        cv::Mat rgb;
        cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);

        cv::Mat f32;
        rgb.convertTo(f32, CV_32FC3, 1.0 / 255.0);

        std::vector<cv::Mat> ch(3);
        cv::split(f32, ch);

        std::vector<float> out(CHW_SIZE);
        for (int c = 0; c < 3; ++c) {
            float* dst = out.data() + c * NET_H * NET_W;
            std::memcpy(dst, ch[c].data, NET_H * NET_W * sizeof(float));
        }
        return out;
    }

    // ── State ─────────────────────────────────────────────────────────────

    visionpilot::models::AutoDrive auto_drive_;
    visionpilot::models::AutoSteer auto_steer_;
    visionpilot::models::AutoSpeed auto_speed_;

    // 2-slot circular buffer of pre-processed BGR frames.
    // write_idx_ starts at 1 so the first write goes to slot 0.
    std::array<cv::Mat, 2> frame_buffer_;
    int          write_idx_    = 1;
    uint64_t     frame_count_  = 0;
    LatencyStats latency_stats_;
};

// ─────────────────────────────────────────────────────────────────────────────
// Spatial pre-processing  (warp / crop / resize → NET_W × NET_H)
//
// In production this will apply the homography warp + 50° FOV crop.
// For now: simple resize as a placeholder.
// ─────────────────────────────────────────────────────────────────────────────

static cv::Mat spatial_preprocess(const cv::Mat& raw)
{
    cv::Mat out;
    cv::resize(raw, out,
               cv::Size(InferencePipeline::NET_W, InferencePipeline::NET_H),
               0, 0, cv::INTER_LINEAR);
    return out;
}

static void print_latency_detail(const FrameLatency& lt, const char* label)
{
    const double parallel_max = std::max({lt.autodrive_ms, lt.autosteer_ms, lt.autospeed_ms});
    printf("  %s latency (models run in parallel — wall ≈ max, not sum):\n", label);
    printf("    preprocess=%.2f ms\n", lt.preprocess_ms);
    printf("    autodrive=%.2f ms  autosteer=%.2f ms  autospeed=%.2f ms\n",
           lt.autodrive_ms, lt.autosteer_ms, lt.autospeed_ms);
    printf("    parallel wall=%.2f ms  max_model=%.2f ms  (%.1f fps)\n",
           lt.pipeline_ms, parallel_max,
           lt.pipeline_ms > 0.0 ? 1000.0 / lt.pipeline_ms : 0.0);
}

static void print_inference_summary(const FrameOutputs& out)
{
    const auto& d  = out.auto_drive;
    const auto& s  = out.auto_steer;
    const auto& sp = out.auto_speed;

    printf("[VisionPilot] Initial inference check (frame %llu, steady-state after GPU warmup)\n",
           static_cast<unsigned long long>(out.frame_id));
    printf("  AutoDrive  valid=%d  dist_norm=%.6f  curvature=%.6f  flag_prob=%.5f\n",
           d.valid, d.dist_normalized, d.curvature_raw, d.flag_prob);
    printf("  AutoSteer  valid=%d  xp[0]=%.6f  h_vector[0]=%.6f\n",
           s.valid, s.xp[0], s.h_vector[0]);
    printf("  AutoSpeed  valid=%d  detections=%zu\n",
           sp.valid, sp.detections.size());
    print_latency_detail(out.latency, "Steady-state");

    if (!d.valid || !s.valid || !sp.valid) {
        std::cerr << "[VisionPilot] WARNING: one or more models returned invalid output\n";
    }
}

// Discard N full inference cycles so CUDA kernels are compiled before we measure.
static void gpu_warmup(InferencePipeline& pipeline, cv::VideoCapture& cap, int cycles)
{
    if (cycles <= 0) return;

    printf("[VisionPilot] GPU warmup: %d inference cycle(s) (not timed)\n", cycles);
    cv::Mat frame;
    for (int c = 0; c < cycles; ++c) {
        if (!cap.read(frame) || frame.empty()) break;
        pipeline.process(spatial_preprocess(frame), false);
        if (!cap.read(frame) || frame.empty()) break;
        pipeline.process(spatial_preprocess(frame), false);
    }
    cap.set(cv::CAP_PROP_POS_FRAMES, 0);
    pipeline.reset();
}

// Warmup → measured inference → rewind video.
static bool run_initial_inference_check(
    InferencePipeline& pipeline, cv::VideoCapture& cap)
{
    // First CUDA runs are ~500 ms+ (kernel compile); discard before measuring.
    gpu_warmup(pipeline, cap, 2);

    cv::Mat frame;
    if (!cap.read(frame) || frame.empty()) {
        std::cerr << "[VisionPilot] Initial check: cannot read first frame\n";
        return false;
    }
    pipeline.process(spatial_preprocess(frame), false);

    if (!cap.read(frame) || frame.empty()) {
        std::cerr << "[VisionPilot] Initial check: cannot read second frame\n";
        return false;
    }
    auto result = pipeline.process(spatial_preprocess(frame), true);
    if (!result) {
        std::cerr << "[VisionPilot] Initial check: inference returned no output\n";
        return false;
    }

    print_inference_summary(*result);

    cap.set(cv::CAP_PROP_POS_FRAMES, 0);
    pipeline.reset();
    std::cout << "[VisionPilot] Initial check OK — restarting video from frame 0\n";
    return true;
}

static std::string fmt(double v, int dec = 2)
{
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.*f", dec, v);
    return buf;
}

static std::vector<std::string> make_inference_overlay(
    const FrameOutputs& result, const std::string& source_label)
{
    const auto& lt = result.latency;
    std::vector<std::string> lines = {
        source_label + "  #" + std::to_string(result.frame_id),
    };
    if (result.auto_drive.valid) {
        lines.push_back(
            "AutoDrive  dist=" + fmt(result.auto_drive.dist_normalized, 3) +
            "  curv=" + fmt(result.auto_drive.curvature_raw, 5) +
            "  flag=" + fmt(result.auto_drive.flag_prob, 3) +
            "  [" + fmt(lt.autodrive_ms) + " ms]");
    }
    if (result.auto_steer.valid) {
        lines.push_back(
            "AutoSteer  xp[0]=" + fmt(result.auto_steer.xp[0], 4) +
            "  [" + fmt(lt.autosteer_ms) + " ms]");
    }
    if (result.auto_speed.valid) {
        lines.push_back(
            "AutoSpeed  dets=" + std::to_string(result.auto_speed.detections.size()) +
            "  [" + fmt(lt.autospeed_ms) + " ms]");
    }
    const double wall_ms = lt.pipeline_ms;
    lines.push_back(
        "Parallel wall " + fmt(wall_ms) + " ms  (" + fmt(1000.0 / wall_ms, 1) + " fps)");
    return lines;
}

static int run_video_mode(
    InferencePipeline& pipeline,
    const VisionPilotConfig& cfg,
    const std::string& video_path_override)
{
    std::string video_path = video_path_override.empty()
        ? cfg.source.video_path : video_path_override;

    cv::VideoCapture cap(video_path);
    if (!cap.isOpened()) {
        std::cerr << "[VisionPilot] Failed to open video: " << video_path << "\n";
        return 1;
    }

    const double file_fps = cap.get(cv::CAP_PROP_FPS);
    const int    total_frames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
    const int    width  = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    const int    height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));

    std::cout << "[VisionPilot] Video mode\n"
              << "  path: " << video_path << "\n"
              << "  " << width << "x" << height
              << "  fps=" << file_fps
              << "  frames=" << total_frames
              << "  realtime=" << (cfg.source.video_realtime ? "yes" : "no")
              << "  loop=" << (cfg.source.video_loop ? "yes" : "no") << "\n";

    if (cfg.pipeline.initial_inference_check) {
        if (!run_initial_inference_check(pipeline, cap)) {
            return 1;
        }
    }

    const auto frame_period = (cfg.source.video_realtime && file_fps > 1.0)
        ? std::chrono::duration<double>(1.0 / file_fps)
        : std::chrono::duration<double>(0);

    while (true) {
        auto tick_start = std::chrono::steady_clock::now();

        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            if (cfg.source.video_loop) {
                cap.set(cv::CAP_PROP_POS_FRAMES, 0);
                pipeline.reset();
                continue;
            }
            std::cout << "[VisionPilot] End of video\n";
            break;
        }

        cv::Mat preprocessed = spatial_preprocess(frame);
        auto result = pipeline.process(preprocessed);

        if (result) {
            // Print latency to console every 30 frames
            if (result->frame_id % 30 == 0) {
                pipeline.latency_stats().print();
            }
            auto overlay = make_inference_overlay(*result, "video");
            visualization::render_frame(frame, "VisionPilot", overlay);
        } else {
            visualization::render_frame(
                frame, "VisionPilot",
                {"video: " + video_path, "warming up (need 2 frames)..."});
        }

        if (cfg.source.video_realtime && file_fps > 1.0) {
            const auto elapsed = std::chrono::steady_clock::now() - tick_start;
            const auto sleep_for = frame_period - elapsed;
            if (sleep_for.count() > 0) {
                std::this_thread::sleep_for(sleep_for);
            }
        }
    }

    visualization::close_windows();
    return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    const int mode_idx = index_of_mode_arg(argc, argv);

    const std::string config_path = resolve_vision_pilot_config_path(argc, argv);
    if (config_path.empty()) {
        std::cerr << "[VisionPilot] No config file found.\n"
                  << "  cp config/vision_pilot.conf.example config/vision_pilot.conf\n"
                  << "  Or: export VISIONPILOT_CONFIG=/path/to/vision_pilot.conf\n";
        return 1;
    }

    VisionPilotConfig cfg;
    try {
        cfg = load_vision_pilot_config(config_path);
    } catch (const std::exception& e) {
        std::cerr << "[VisionPilot] Config error: " << e.what() << "\n";
        return 1;
    }

    SourceMode mode = cfg.source.mode;
    if (mode_idx >= 0 && mode_idx < argc) {
        mode = static_cast<SourceMode>(std::stoi(argv[mode_idx]));
    }

    std::cout << "[VisionPilot] Config: " << config_path << "\n"
              << "  autodrive: " << cfg.autodrive_model << "\n"
              << "  autosteer: " << cfg.autosteer_model << "\n"
              << "  autospeed: " << cfg.autospeed_model << "\n"
              << "  provider:  " << cfg.engine_cfg.provider << "\n"
              << "  source:    ";
    switch (mode) {
        case SourceMode::Ros2:  std::cout << "ros2\n"; break;
        case SourceMode::V4l2:  std::cout << "v4l2\n"; break;
        case SourceMode::Video: std::cout << "video  " << cfg.source.video_path << "\n"; break;
    }

    visionpilot::engine::OnnxEngine ort_engine(cfg.engine_cfg);
    InferencePipeline  pipeline(ort_engine, cfg);

    // ════════════════════════════════════════════════════════════════════════
    // VIDEO MODE  (ZOD mp4 / any OpenCV-readable file)
    // ════════════════════════════════════════════════════════════════════════
    if (mode == SourceMode::Video) {
        const std::string video_override =
            (mode_idx >= 0 && mode_idx + 1 < argc) ? argv[mode_idx + 1] : "";
        return run_video_mode(pipeline, cfg, video_override);
    }

    // ════════════════════════════════════════════════════════════════════════
    // ROS2 MODE
    // ════════════════════════════════════════════════════════════════════════
    if (mode == SourceMode::Ros2) {

        const std::string topic = (mode_idx >= 0 && mode_idx + 1 < argc)
            ? argv[mode_idx + 1] : cfg.source.ros2_topic;
        std::cout << "[VisionPilot] ROS2 mode | topic: " << topic << "\n";

        camera_subscriber::ROS2ImageSubscriber ros2_sub(topic);

        while (true) {
            auto [has_frame, frame] = ros2_sub.get_latest_frame();

            if (!has_frame || frame.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            cv::Mat preprocessed = spatial_preprocess(frame);
            auto result = pipeline.process(preprocessed);

            if (!result) {
                // First frame — buffer warming up, nothing to output yet.
                continue;
            }

            // ── Downstream: use result->auto_drive / auto_steer / auto_speed
            // TODO: hand off to planning / control / visualization modules.

            auto stats = ros2_sub.get_stats();
            std::vector<std::string> overlay = {
                "topic: "            + topic,
                "frame: "            + std::to_string(result->frame_id),
                "frames received: "  + std::to_string(stats.frames_received),
                "frames dropped: "   + std::to_string(stats.frames_dropped),
            };
            visualization::render_frame(frame, "VisionPilot", overlay);
        }

        visualization::close_windows();

    // ════════════════════════════════════════════════════════════════════════
    // V4L2 MODE
    // ════════════════════════════════════════════════════════════════════════
    } else if (mode == SourceMode::V4l2) {

        const std::string device_path = (mode_idx >= 0 && mode_idx + 1 < argc)
            ? argv[mode_idx + 1] : cfg.source.v4l2_device;
        const uint32_t target_fps = (mode_idx >= 0 && mode_idx + 2 < argc)
            ? static_cast<uint32_t>(std::stoi(argv[mode_idx + 2]))
            : static_cast<uint32_t>(cfg.source.v4l2_fps);

        std::cout << "[VisionPilot] V4L2 mode | device: " << device_path
                  << "  fps: " << target_fps << "\n";

        v4l2_interface::V4L2Reader v4l2_reader(device_path, target_fps);
        if (!v4l2_reader.is_device_open()) {
            std::cerr << "[VisionPilot] Failed to open V4L2 device: " << device_path << "\n";
            return 1;
        }

        while (true) {
            auto [has_frame, frame] = v4l2_reader.get_latest_frame();

            if (!has_frame || frame.empty()) continue;

            cv::Mat preprocessed = spatial_preprocess(frame);
            auto result = pipeline.process(preprocessed);

            if (!result) continue;

            // ── Downstream: use result->auto_drive / auto_steer / auto_speed
            // TODO: hand off to planning / control / visualization modules.

            auto stats = v4l2_reader.get_stats();
            std::vector<std::string> overlay = {
                "device: "    + device_path,
                "frame: "     + std::to_string(result->frame_id),
                "fps: "       + std::to_string(static_cast<int>(stats.current_fps)),
                "resolution: "+ std::to_string(stats.current_width) + "x"
                              + std::to_string(stats.current_height),
            };
            visualization::render_frame(frame, "VisionPilot", overlay);
        }

        visualization::close_windows();

    } else {
        std::cerr << "[VisionPilot] Invalid mode.\n"
                  << "Usage: " << argv[0] << " [--config path] [mode] [args...]\n"
                  << "  Omit mode to use source.mode from config (default: video).\n"
                  << "  mode 0 / ros2  : [topic]\n"
                  << "  mode 1 / v4l2  : [device] [fps]\n"
                  << "  mode 2 / video : [path]  (overrides source.video_path)\n";
        return 1;
    }

    return 0;
}
