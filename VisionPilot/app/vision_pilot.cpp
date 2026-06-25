// VisionPilot — preprocess → inference → fusion → display
#include <config/vision_pilot_config.hpp>
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
#include <vehicle_interface/vehicle_interface.hpp>
#include <vehicle_interface/can_interface.hpp>

#ifdef ENABLE_ROS2_INTERFACE
#include <rclcpp/rclcpp.hpp>
#include <vehicle_ros2_interface/vehicle_ros2_interface.hpp>
#endif

namespace ve = visionpilot::engine;
namespace vm = visionpilot::models;

int main(int argc, char** argv)
{
    const std::string cfg_path = resolve_vision_pilot_config_path(argc, argv);
    if (cfg_path.empty())
    {
        VP_ERROR("No config — cp config/vision_pilot.conf.example config/vision_pilot.conf");
        return 1;
    }

    std::shared_ptr<VehicleInterface> vehicle_interface;
#ifdef ENABLE_ROS2_INTERFACE
    rclcpp::init(argc, argv);
    vehicle_interface = std::make_shared<VehicleRos2Interface>();
#else
    vehicle_interface = std::make_shared<CanInterface>();
#endif

    VisionPilotConfig cfg;
    try { cfg = load_vision_pilot_config(cfg_path); }
    catch (const std::exception& e)
    {
        VP_ERROR("Config: %s", e.what());
        return 1;
    }

    ImagePreprocessor preprocessor;
    ve::OnnxEngine engine(cfg.engine);
    vm::InferencePipeline pipeline(engine, cfg.inference);
    Planner planner(cfg.speed_limit, cfg.Lf);

    visualization::init_production_assets();

    bool show_window = true;
#ifdef ENABLE_WEBRTC
    std::unique_ptr<visualization::WebRTCStreamer> webrtc;
    for (int i = 1; i < argc; ++i)
    {
        if (std::string(argv[i]) == "--webrtc") show_window = false;
        if (std::string(argv[i]) == "--webrtc-port" && i + 1 < argc)
        {
            webrtc = std::make_unique<visualization::WebRTCStreamer>();
            if (!webrtc->init(static_cast<uint16_t>(std::stoi(argv[++i])))) return 1;
        }
    }
#endif

    auto source = camera_interface::open_frame_source(cfg.source);
    if (!source || !source->is_device_open())
    {
        VP_ERROR("Cannot open frame source");
        return 1;
    }

    const cv::Size net_size(vm::AutoDrive::NET_W, vm::AutoDrive::NET_H);
    cv::Mat frame, warped, resized;

    while (true)
    {
        auto [ok, frame] = source->get_latest_frame();
        if (!ok || frame.empty())
        {
            if (cfg.source.mode == SourceMode::Video && !cfg.source.video_loop) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        preprocessor.preprocess(frame, warped, resized, net_size);

        if (const auto r = pipeline.process(warped))
        {
            pipeline.latency().print();

            const double ego_v     = vehicle_interface->read();
            const double cte       = r->lateral.cte_m;
            const double epsi      = r->lateral.yaw_rad;
            const double kappa     = r->lateral.curvature;
            const bool   has_cipo  = r->cipo.cipo_raw_found;
            const double cipo_v    = has_cipo ? r->cipo.velocity_ms : cfg.speed_limit;
            const double cipo_dist = r->cipo.distance_m;

            const Plan plan = planner.compute_plan(
                cte, epsi, kappa, ego_v, has_cipo, cipo_v, cipo_dist);

            VP_INFO("plan: tyre=%.4f rad  accel=%.3f m/s²",
                    plan.steering.empty() ? 0.0 : plan.steering[0],
                    plan.acceleration);

            vehicle_interface->write(
                plan.steering.empty() ? 0.0 : plan.steering[0],
                plan.acceleration);

            if (show_window)
                visualization::ProductionView::visualize(warped, *r, plan, ego_v);
        }
        else if (show_window)
        {
            visualization::show_frame(warped);
        }

#ifdef ENABLE_WEBRTC
        if (webrtc) webrtc->push_frame(warped);
#endif
    }

    visualization::close_windows();
    return 0;
}
