// VisionPilot — preprocess → inference → fusion → display
#include <config/vision_pilot_config.hpp>
#include <debug/debug_draw.hpp>
#include <engine/onnx_engine.hpp>
#include <image_preprocessing/image_preprocessor.hpp>
#include <logging/logger.hpp>
#include <models/inference.hpp>
#include <planning/planning.hpp>
#include <visualization/visualization.hpp>

#include <camera_interface/frame_source.hpp>
#ifdef ENABLE_WEBRTC
#include <visualization/visualization_to_webrtc.hpp>
#endif

#include <chrono>
#include <memory>
#include <thread>

#include <fstream>

namespace ve = visionpilot::engine;
namespace vm = visionpilot::models;
namespace vd = visionpilot::debug;

std::vector<double> readSpeeds(const std::string& filename) {
    std::vector<double> speeds;
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: could not open file " << filename << std::endl;
        return speeds;
    }

    std::string line;
    while (std::getline(file, line)) {
        // skip empty lines
        if (line.find_first_not_of(" \t\r\n") == std::string::npos) {
            continue;
        }
        try {
            double value = std::stod(line);
            speeds.push_back(value);
        } catch (const std::exception& e) {
            std::cerr << "Warning: skipping invalid line: \"" << line
                      << "\" (" << e.what() << ")" << std::endl;
        }
    }

    file.close();
    return speeds;
}

int main(int argc, char** argv)
{
    // ── 1. Config ─────────────────────────────────────────────────────────────
    const std::string cfg_path = resolve_vision_pilot_config_path(argc, argv);
    if (cfg_path.empty()) {
        VP_ERROR("No config — cp config/vision_pilot.conf.example config/vision_pilot.conf");
        return 1;
    }

    VisionPilotConfig cfg;
    try { cfg = load_vision_pilot_config(cfg_path); }
    catch (const std::exception& e) { VP_ERROR("Config: %s", e.what()); return 1; }

    // ── 2. Pipeline (preprocess + ONNX + inference/fusion) ────────────────────
    ImagePreprocessor preprocessor;
    ve::OnnxEngine engine(cfg.engine);
    vm::InferencePipeline pipeline(engine, cfg.inference);

    vd::init_wheel_assets(cfg.wheel_dir);
    vd::init_homography();

    // ── 3. Display output ─────────────────────────────────────────────────────
    bool show_window = true;
#ifdef ENABLE_WEBRTC
    std::unique_ptr<visualization::WebRTCStreamer> webrtc;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--webrtc") show_window = false;
        if (std::string(argv[i]) == "--webrtc-port" && i + 1 < argc) {
            webrtc = std::make_unique<visualization::WebRTCStreamer>();
            if (!webrtc->init(static_cast<uint16_t>(std::stoi(argv[++i])))) return 1;
        }
    }
#endif

    // ── 4. Frame source (video / V4L2 / ROS2) ───────────────────────────────
    auto source = camera_interface::open_frame_source(cfg.source);
    if (!source || !source->is_device_open()) {
        VP_ERROR("Cannot open frame source");
        return 1;
    }

    const cv::Size net_size(vm::AutoDrive::NET_W, vm::AutoDrive::NET_H);
    const std::string label = source_label(cfg.source);
    cv::Mat frame, warped, resized;

    Planner planner;
    // ── 5. Main loop ────────────────────────────────────────────────────────
    int frame_number = 0;
    std::vector<double> speeds = readSpeeds("/data/DEVELOPMENT/AUTONOMOUS/AUTOWARE/TOOLS/wod/extracted_2/frame_speed.txt");
    while (true) {
        auto [ok, frame] = source->get_latest_frame();
        if (!ok || frame.empty()) {
            if (cfg.source.mode == SourceMode::Video && !cfg.source.video_loop) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        preprocessor.preprocess(frame, warped, resized, net_size);

        if (const auto r = pipeline.process(warped)) {
            pipeline.latency().print();
            vd::annotate_frame(warped, vd::debug_view_from(
                *r, label, cfg.wheel_dir));

            double cte = r->lateral.cte_m;
            double epsi = r->lateral.yaw_rad;
            double kappa = r->lateral.curvature;
            double ego_v = speeds[frame_number++];
            double cipo_v = r->cipo.velocity_ms;
            double cipo_distance = r->cipo.distance_m;
            bool has_cipo = r->cipo.cipo_raw_found;


            // std::pair<double, std::vector<double>> plan = planner.compute_plan(cte, epsi, kappa, ego_v, has_cipo, ego_v + cipo_v, cipo_distance);
            auto [acceleration, steering] = planner.compute_plan(cte, epsi, kappa, ego_v, has_cipo, ego_v + cipo_v, cipo_distance);
            std::cout << "Steering: " << steering[1] * 180.0/M_PI << "  Acceleration: " << acceleration << "  EGO speed: " << ego_v << "  CIPO speed: " << ego_v + cipo_v << std::endl;
        }

        if (show_window) visualization::render_frame(warped, "VisionPilot", {});
#ifdef ENABLE_WEBRTC
        if (webrtc) webrtc->push_frame(warped);
#endif
    }

    visualization::close_windows();
    return 0;
}
