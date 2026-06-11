// VisionPilot — preprocess → inference → fusion → display
#include <config/vision_pilot_config.hpp>
#include <debug/debug_draw.hpp>
#include <engine/onnx_engine.hpp>
#include <image_preprocessing/image_preprocessor.hpp>
#include <logging/logger.hpp>
#include <models/inference.hpp>
#include <visualization/visualization.hpp>

#include <camera_interface/frame_source.hpp>
#ifdef ENABLE_WEBRTC
#include <visualization/visualization_to_webrtc.hpp>
#endif

#include <chrono>
#include <memory>
#include <thread>

namespace ve = visionpilot::engine;
namespace vm = visionpilot::models;
namespace vd = visionpilot::debug;

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
    ve::OnnxEngine engine(cfg.engine_cfg);
    vm::InferencePipeline pipeline(engine, {
        cfg.autodrive_model, cfg.autosteer_model, cfg.autospeed_model,
        cfg.homography_path, cfg.fusion_debug,
    });

    vd::init_wheel_assets(cfg.wheel_dir);
    vd::init_homography(cfg.homography_path);

    // ── 3. Display output ─────────────────────────────────────────────────────
    bool show_window = true;
    bool debug_ui = false;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--debug-ui") {
            debug_ui = true;
        }
    }

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

    // ── 5. Main loop ────────────────────────────────────────────────────────
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
            
            if (debug_ui) {
                // If developer mode is toggled, generate the teammate's HUD annotations
                vd::annotate_frame(display_frame, vd::debug_view_from(
                    *r, label, cfg.wheel_dir, cfg.homography_path));
            } else {

                // ====================== VISUALIZATION CALL ======================

                // 1. AutoSpeed detections => YOLO bbox mapping
                std::vector<visualization::YoloBoundingBox> bboxes;
                for (const auto& det : r->auto_speed.detections) {

                    visualization::YoloBoundingBox bbox;
                    bbox.class_id = det.class_id;

                    // Convert absolute pixel coords back to normalized
                    bbox.center_x = (det.x1 + det.x2) / (2.0f * 1024.0f);
                    bbox.center_y = (det.y1 + det.y2) / (2.0f * 512.0f);
                    bbox.width    = (det.x2 - det.x1) / 1024.0f;
                    bbox.height   = (det.y2 - det.y1) / 512.0f;

                    bboxes.push_back(bbox);

                }

                // 2. Map lane shape 

            }

        }

        // Visualization is called here

        if (show_window) visualization::render_frame(warped, "VisionPilot", {});

#ifdef ENABLE_WEBRTC
        if (webrtc) webrtc->push_frame(warped);
#endif

    }

    visualization::close_windows();
    return 0;
}
