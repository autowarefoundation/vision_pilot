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

#include <algorithm>
#include <chrono>
#include <memory>
#include <thread>

namespace ve = visionpilot::engine;
namespace vm = visionpilot::models;
namespace vd = visionpilot::debug;

int main(int argc, char **argv) {
    // ── 1. Config ─────────────────────────────────────────────────────────────
    const std::string cfg_path = resolve_vision_pilot_config_path(argc, argv);
    if (cfg_path.empty()) {
        VP_ERROR("No config — cp config/vision_pilot.conf.example config/vision_pilot.conf");
        return 1;
    }

    VisionPilotConfig cfg;
    try { cfg = load_vision_pilot_config(cfg_path); } catch (const std::exception &e) {
        VP_ERROR("Config: %s", e.what());
        return 1;
    }

    // ── 2. Pipeline (preprocess + ONNX + inference/fusion) ────────────────────
    ImagePreprocessor preprocessor;
    ve::OnnxEngine engine(cfg.engine);
    // vm::InferencePipeline pipeline(engine, {cfg.inference.precision, cfg.fusion_debug,});
    vm::InferencePipeline pipeline(engine, cfg.inference);

    Planner planner(cfg.speed_limit, cfg.Lf);

    vd::init_wheel_assets(cfg.wheel_dir);
    vd::init_homography();

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
    double planned_speed_mps = cfg.speed_limit;
    auto last_plan_ts = std::chrono::steady_clock::now();

    // ── 5. Main loop ────────────────────────────────────────────────────────
    while (true) {
        auto [ok, frame] = source->get_latest_frame();
        if (!ok || frame.empty()) {
            if (cfg.source.mode == SourceMode::Video && !cfg.source.video_loop) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        preprocessor.preprocess(frame, warped, resized, net_size);

        cv::Mat display_frame = resized.clone();

        if (const auto r = pipeline.process(resized)) {

            pipeline.latency().print();
            
            if (debug_ui) {
                // If developer mode is toggled, generate the teammate's HUD annotations
                vd::annotate_frame(display_frame, vd::debug_view_from(
                    *r, label, cfg.wheel_dir));
            } else {

                // ====================== VISUALIZATION CALL ======================

                // 1. AutoSpeed detections => YOLO bbox mapping

                std::vector<visualization::YoloBoundingBox> bboxes;
                for (const auto& det : r->auto_speed.detections) {

                    visualization::YoloBoundingBox bbox;
                    bbox.class_id = det.class_id;

                    // Convert absolute pixel coords back to normalized
                    bbox.center_x = (det.x1 + det.x2) / (2.0f * visualization::kFrameWidth);
                    bbox.center_y = (det.y1 + det.y2) / (2.0f * visualization::kFrameHeight);
                    bbox.width    = (det.x2 - det.x1) / visualization::kFrameWidth;
                    bbox.height   = (det.y2 - det.y1) / visualization::kFrameHeight;

                    bboxes.push_back(bbox);

                }

                // 2. Map lane shape, filter tracking params

                visualization::LaneShapeVisualization lane_shape;
                lane_shape.has_cipo_object = r->cipo.valid;
                if (r->cipo.valid) {
                    lane_shape.distance_to_cipo = r->cipo.distance_m;
                    lane_shape.relative_cipo_velocity = r->cipo.velocity_ms * 3.6f; // m/s to km/h
                }

                // Inject lateral fusion tracking polynomial params for BEV rendering
                lane_shape.path_a = r->lateral.path_a;
                lane_shape.path_b = r->lateral.path_b;
                lane_shape.path_c = r->lateral.path_c;

                // Extract perspective image centerline points from AutoSteer flattened array
                for (int i = 0; i < 64; ++i) {
                    float h_left = r->auto_steer.h_vector[i];
                    float h_right = r->auto_steer.h_vector[64 + i];

                    if (h_left >= 0.5f && h_right >= 0.5f) {
                        float left_x = r->auto_steer.xp[i] * visualization::kFrameWidth;
                        float right_x = r->auto_steer.xp[64 + i] * visualization::kFrameWidth;
                        float center_x = (left_x + right_x) / 2.0f;
                        float v = i * (511.0f / 63.0f); // Map 64 intervals to 512px height

                        lane_shape.tracked_waypoints.emplace_back(center_x, v);
                    }
                }

                // 3. Map Desired Control Parameters

                visualization::DesiredControlVisualization desired_control;

                // 3.1 Pull control from planning module (not mock constants)
                const auto now = std::chrono::steady_clock::now();
                const double dt_s = std::clamp(
                    std::chrono::duration<double>(now - last_plan_ts).count(),
                    0.01,
                    0.2
                );
                last_plan_ts = now;

                const double cte = r->lateral.cte_m;
                const double epsi = r->lateral.yaw_rad;
                const double kappa = r->lateral.curvature;
                const bool has_cipo = r->cipo.valid;
                const double cipo_distance = has_cipo ? r->cipo.distance_m : 9999.0;
                const double lead_speed_mps = has_cipo
                    ? std::max(0.0, planned_speed_mps + static_cast<double>(r->cipo.velocity_ms))
                    : cfg.speed_limit;

                const auto [acceleration_cmd, steering_plan] = planner.compute_plan(
                    cte,
                    epsi,
                    kappa,
                    planned_speed_mps,
                    has_cipo,
                    lead_speed_mps,
                    cipo_distance
                );

                planned_speed_mps = std::clamp(planned_speed_mps + acceleration_cmd * dt_s, 0.0, cfg.speed_limit);

                const double steering_rad = steering_plan.empty() ? 0.0 : steering_plan.front();
                desired_control.steering_angle = static_cast<float>(steering_rad * 180.0 / M_PI);
                desired_control.velocity = static_cast<float>(planned_speed_mps * 3.6);
                desired_control.acceleration = static_cast<float>(acceleration_cmd);

                // 4. Full UI vis render call
                display_frame = visualization::visualize_frame(
                    resized, 
                    bboxes, 
                    lane_shape, 
                    desired_control
                );

            }

        }

        if (show_window) visualization::render_frame(
            display_frame, 
            "VisionPilot", 
            {}
        );

#ifdef ENABLE_WEBRTC
        if (webrtc) webrtc->push_frame(warped);
#endif

    }

    visualization::close_windows();
    return 0;
}
