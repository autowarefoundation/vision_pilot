// VisionPilot — preprocess → inference → fusion → display (+ optional control)
#include "control_bridge.hpp"

#include <config/vision_pilot_config.hpp>
#include <control/control_command.hpp>
#include <control/lateral_control.hpp>
#include <control/longitudinal_control.hpp>
#include <debug/debug_draw.hpp>
#include <engine/onnx_engine.hpp>
#include <image_preprocessing/image_preprocessor.hpp>
#include <logging/logger.hpp>
#include <models/inference.hpp>
#include <planning/planning.hpp>
#include <visualization/visualization.hpp>

#include <camera_interface/frame_source.hpp>
#ifdef ENABLE_ROS2_INTERFACE
#include <control_cmd_publisher/cmd_to_ros2.hpp>
#endif
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

    // ── 4b. Control (optional, off by default) ───────────────────────────────
    Planner planner;
    LongitudinalController lon_ctrl;
    LateralController lat_ctrl;
#ifdef ENABLE_ROS2_INTERFACE
    std::unique_ptr<control_cmd_publisher::ControlCmdPublisher> control_pub;
    if (cfg.control.enabled) {
        control_pub = std::make_unique<control_cmd_publisher::ControlCmdPublisher>(
            cfg.control.topic, cfg.control.frame_id);
    }
#endif

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
            vd::annotate_frame(warped, vd::debug_view_from(
                *r, label, cfg.wheel_dir, cfg.homography_path));

            if (cfg.control.enabled && r->lateral.valid) {
                // cte/epsi/kappa come straight from lateral fusion (the signed model-view
                // world frame the MPC expects). velocity_ms is the closing rate (negative
                // = approaching), so the lead's absolute speed is ego_v + velocity_ms.
                // ego_v is a placeholder until vehicle-state input (Phase 5).
                const double ego_v = cfg.control.ego_speed_mps;
                const bool has_cipo = r->cipo.valid;
                const double cipo_v = has_cipo ? ego_v + r->cipo.velocity_ms : ego_v;
                const double cipo_distance = has_cipo ? r->cipo.distance_m : 9999.0;

                const ControlCommand cmd = compute_control_command(
                    planner, lon_ctrl, lat_ctrl, r->lateral.cte_m, r->lateral.yaw_rad,
                    r->lateral.curvature, ego_v, has_cipo, cipo_v, cipo_distance,
                    cfg.control.dt_s);
                VP_INFO("[Control] steer=%.4f rad  speed=%.2f m/s  accel=%.2f m/s2",
                        cmd.steering_angle_rad, cmd.speed_mps, cmd.acceleration_mps2);
#ifdef ENABLE_ROS2_INTERFACE
                if (control_pub) control_pub->publish(cmd);
#endif
            }
        }

        if (show_window) visualization::render_frame(warped, "VisionPilot", {});
#ifdef ENABLE_WEBRTC
        if (webrtc) webrtc->push_frame(warped);
#endif
    }

    visualization::close_windows();
    return 0;
}
