// VisionPilot — preprocess → inference → fusion → display
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <config/vision_pilot_config.hpp>
#include <common/utils.hpp>
#include <engine/onnx_engine.hpp>
#include <vehicle_interface/vehicle_interface.hpp>
#include <vehicle_interface/can_interface.hpp>
#include <logging/logger.hpp>
#include <models/inference.hpp>
#include <pilot/pilot.hpp>
#include <visualization/visualization.hpp>
#include <debug/debug_draw.hpp>

#include "camera_interface/v4l2_camera_interface.hpp"
#include "camera_interface/file_interface.hpp"
#include "vehicle_interface/file_interface.hpp"

#if ENABLE_ROS2_INTERFACE
#include <rclcpp/rclcpp.hpp>
#include <camera_ros2_interface/camera_ros2_interface.hpp>
#include <vehicle_ros2_interface/vehicle_ros2_interface.hpp>
#endif

namespace vm = visionpilot::models;
namespace vp = visionpilot::pilot;
namespace vd = visionpilot::debug;

int main(int argc, char** argv)
{
    Config cfg;
    try { cfg = load_vision_pilot_config(); }
    catch (const std::exception& e)
    {
        VP_ERROR("Config: %s", e.what());
        return 1;
    }

    // ── CLI flags ─────────────────────────────────────────────────────────────
    bool show_window = true;
    bool debug_viz = false;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg(argv[i]);
        if (arg == "--debug-viz") debug_viz = true;
        else if (arg == "--no-window") show_window = false;
    }

    std::shared_ptr<CameraInterface> camera_interface;
    std::shared_ptr<VehicleInterface> vehicle_interface;
#if ENABLE_ROS2_INTERFACE
    rclcpp::init(argc, argv);
    camera_interface = std::make_unique<CameraRos2Interface>(cfg.source.input_camera_topic);
    vehicle_interface = std::make_shared<VehicleRos2Interface>(cfg.vehicle_speed_topic,
                                                               cfg.vehicle_steering_topic,
                                                               cfg.vehicle_acceleration_topic);
#else
    if (cfg.source.mode == SourceMode::Video)
    {
        camera_interface = std::make_unique<camera_interface::FileInterface>(
            cfg.source.input_video, cfg.source.video_loop, cfg.source.video_realtime);
        vehicle_interface = std::make_shared<vehicle_interface::FileInterface>(
            cfg.source.input_vehicle_speed, cfg.source.video_loop);
    }
    else
    {
        camera_interface = std::make_unique<camera_interface::V4L2CameraInterface>(
            cfg.source.v4l2_device, static_cast<uint32_t>(cfg.source.v4l2_fps));
        vehicle_interface = std::make_shared<CanInterface>();
    }
#endif

    // preprocess → inference → fusion → planning; the same step the Python
    // bindings drive in closed-loop simulation.
    vp::Config pilot_cfg;
    pilot_cfg.engine = cfg.engine;
    pilot_cfg.inference = cfg.inference;
    pilot_cfg.speed_limit = cfg.speed_limit;
    pilot_cfg.L = cfg.L;
    vp::Pilot pilot(pilot_cfg, load_matrix("H.yaml", "H"),
                    load_matrix("homography_C_matrix.yaml", "C"));
    const vm::InferencePipeline& pipeline = pilot.pipeline();
    if (cfg.rrd_on) logging::Rerun::init(cfg.rrd_log);

    // ── Init visualization assets once based on mode ──────────────────────────
    if (debug_viz)
    {
        VP_INFO("[Viz] Debug mode — annotated telemetry overlay");
        vd::init_wheel_assets(cfg.wheel_dir);
        vd::init_homography();
    }
    else
    {
        VP_INFO("[Viz] Production mode — clean HUD");
        visualization::init_production_assets();
    }

    // ── Initialize camera interface ───────────────────────────────────────────

    if (!camera_interface || !camera_interface->is_device_open())
    {
        VP_ERROR("Cannot open frame source");
        return 1;
    }

    // ── Initialize display ────────────────────────────────────────────────────
    visualization::Visualization visualization({cfg.webrtc_on, cfg.webrtc_port, show_window});

    while (true)
    {
        auto [ok, frame] = camera_interface->get_latest_frame();
        if (!ok || frame.empty())
        {
            if (cfg.source.mode == SourceMode::Video && !cfg.source.video_loop) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        // Read once per frame, so a recorded speed file stays aligned with the video.
        const double ego_v = vehicle_interface->read();
        vp::StepResult step = pilot.step(frame, ego_v);
        const cv::Mat& warped = step.warped;
        cv::Mat& resized = step.resized;

        // ── Default frame no inference ────────────────────────────────────────────
        cv::Mat display_frame = resized;

        if (const auto& r = step.perception)
        {
            // pipeline.latency().print();

            const double cte = r->lateral.cte_m;
            const double epsi = r->lateral.yaw_rad;
            const double kappa = r->lateral.curvature;
            const bool has_cipo = step.has_cipo;
            const double cipo_dist = r->cipo.distance_m;

            const double raw_cte = r->lateral.path_valid
                                       ? static_cast<double>(r->lateral.raw_cte_m)
                                       : cte;
            const Plan& plan = *step.plan;

            VP_INFO(
                "plan: tyre=%.4f rad  accel=%.3f m/s²  |  cte=%.2fm(raw=%.2fm) cte_dot=%+.2fm/s  epsi=%.3f epsi_dot=%+.3frad/s  kappa=%.4f  |  cipo=%s  dist=%.1f m  vel=%+.2f m/s",
                plan.steering.empty() ? 0.0 : plan.steering[1],
                plan.acceleration,
                cte,
                raw_cte,
                r->lateral.cte_rate_mps,
                epsi,
                r->lateral.yaw_rate_rps,
                kappa,
                has_cipo ? "true" : "false",
                cipo_dist,
                r->cipo.velocity_ms);

            vehicle_interface->write(step.command->steering_tyre_rad,
                                     step.command->acceleration_mps2);
            cv::Mat viz;  // output visualization image (empty when viz is off)
            if (cfg.visualization_on)
            {
                if (debug_viz)
                {
                    // annotate_frame() draws inplace
                    viz = cfg.rrd_on ? resized.clone() : resized;
                    vd::visualize(viz, *r, source_label(cfg.source), cfg.wheel_dir, pipeline.H_world2resized());
                    display_frame = viz;
                }
                else
                {
                    display_frame = visualization.build_frame(resized, *r, plan, ego_v, pipeline.H_resized(), cfg.speed_limit);
                    viz = display_frame;
                }
            }

            // Submit all required logging params to single logger func
            if (cfg.rrd_on)
                logging::Rerun::log_frame(r->frame_id, frame, warped, resized, *r, plan, ego_v, viz);
        }
        if (cfg.visualization_on)
        {
            visualization.render_frame(display_frame);
        }
    }

    if (cfg.rrd_on) logging::Rerun::shutdown();  // flush & close .rrd

    // stop() returns true on a clean shutdown; translate that to a 0 exit code
    // so VisionPilot can be supervised as a batch/oneshot job (a successful run
    // must not exit non-zero).
    return visualization.stop() ? 0 : 1;
}
