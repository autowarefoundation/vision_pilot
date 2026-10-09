// VisionPilot — preprocess → inference → fusion → display
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <config/vision_pilot_config.hpp>
#include <common/utils.hpp>
#include <engine/onnx_engine.hpp>
#include <camera_interface/v4l2_camera_interface.hpp>
#include <camera_interface/file_interface.hpp>
#include <vehicle_interface/vehicle_interface.hpp>
#include <vehicle_interface/file_interface.hpp>
#include <vehicle_interface/can_interface.hpp>
#include <image_preprocessing/image_preprocessor.hpp>
#include <logging/logger.hpp>
#include <models/inference.hpp>
#include <planning/planning.hpp>
#include <visualization/visualization.hpp>

#if ENABLE_RADAR_INTERFACE
#include <radar_interface/file_interface.hpp>
#endif

#if ENABLE_ROS2_INTERFACE
#include <rclcpp/rclcpp.hpp>
#include <camera_ros2_interface/camera_ros2_interface.hpp>
#include <vehicle_ros2_interface/vehicle_ros2_interface.hpp>
#endif

#if BUILD_TESTING
#include <debug/debug_draw.hpp>
#endif


namespace ve = visionpilot::engine;
namespace vm = visionpilot::models;
#if BUILD_TESTING
namespace vd = visionpilot::debug;
#endif

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
#if BUILD_TESTING
    bool debug_viz = false;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg(argv[i]);
        if (arg == "--debug-viz") debug_viz = true;
    }
#endif

    std::shared_ptr<CameraInterface> camera_interface;
    std::shared_ptr<VehicleInterface> vehicle_interface;
#if ENABLE_RADAR_INTERFACE
    std::shared_ptr<RadarInterface> radar_interface;
#endif


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
#if ENABLE_RADAR_INTERFACE
        radar_interface = std::make_shared<radar_interface::FileInterface>(cfg.source.input_radar_file, cfg.source.video_loop);
#endif
    }
    else
    {
        camera_interface = std::make_unique<camera_interface::V4L2CameraInterface>(
            cfg.source.v4l2_device, static_cast<uint32_t>(cfg.source.v4l2_fps));
        vehicle_interface = std::make_shared<CanInterface>();
    }
#endif

    ImagePreprocessor preprocessor;
    ve::OnnxEngine engine(cfg.engine);
#if ENABLE_RADAR_INTERFACE
    cfg.inference.long_fusion.radar_enabled = cfg.radar_on;
    cfg.inference.long_fusion.radar_hfov_deg = cfg.radar_hfov_deg;
#endif
    vm::InferencePipeline pipeline(engine, cfg.inference);
    Planner planner(cfg.speed_limit, cfg.L);
    if (cfg.rrd_on) logging::Rerun::init(cfg.rrd_log);

    // ── Init visualization assets once based on mode ──────────────────────────
#if BUILD_TESTING
    if (debug_viz)
    {
        VP_INFO("[Viz] Debug mode — annotated telemetry overlay");
        vd::init_wheel_assets(cfg.wheel_dir);
        vd::init_homography();
    }
    else
#endif
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
    visualization::Visualization visualization({cfg.webrtc_on, cfg.webrtc_port, cfg.visualization_on});

    const cv::Size net_size(vm::AutoDrive::NET_W, vm::AutoDrive::NET_H);
    cv::Mat frame, warped, resized;
    bool h_resized_set = false;
    cv::Mat H = load_matrix("H.yaml", "H");
    while (true)
    {
        auto [ok, frame] = camera_interface->get_latest_frame();
        if (!ok || frame.empty())
        {
            if (cfg.source.mode == SourceMode::Video && !cfg.source.video_loop) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        preprocessor.preprocess(frame, warped, resized, net_size);
        cv::Size frame_size = frame.size();
        // One-time: tell the pipeline how to project AutoSteer/AutoSpeed outputs
        // back to world when those networks run on the plain-resized image.
        if (!h_resized_set)
        {
            pipeline.set_H_resized(H, frame_size);
            h_resized_set = true;
        }

        // ── Default frame no inference ────────────────────────────────────────────
        cv::Mat display_frame = resized;

        const double ego_v = vehicle_interface->read();
        VP_INFO("ego_speed=%.2f m/s", ego_v);
#if ENABLE_RADAR_INTERFACE
        if (cfg.radar_on)
        {
            std::vector<RadarPoint> points = radar_interface->read_points();
            pipeline.set_radar_points(points);
        }
#endif
        if (const auto r = pipeline.process(warped, resized,
                                            static_cast<float>(ego_v), true))
        {
            const double cte = r->lateral.cte_m;
            const double epsi = r->lateral.yaw_rad;
            const double kappa = r->lateral.curvature;

            // has_cipo: tracker-based — true only when filter tracks a target
            // closer than D_MAX. cipo_raw_found alone must not gate the planner.
            static constexpr double D_MAX = 150.0;
            const bool has_cipo = r->cipo.valid && r->cipo.distance_m < D_MAX;
            const double cipo_v = has_cipo ? r->cipo.velocity_ms : cfg.speed_limit;
            const double cipo_dist = r->cipo.distance_m;

            const double raw_cte = r->lateral.path_valid
                                       ? static_cast<double>(r->lateral.raw_cte_m)
                                       : cte;
            const Plan plan = planner.compute_plan(
                cte, epsi, kappa, ego_v, has_cipo, cipo_v, cipo_dist);

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

            vehicle_interface->write(
                plan.steering.empty() ? 0.0 : plan.steering[1],
                plan.acceleration);
            cv::Mat viz; // output visualization image (empty when viz is off)
            if (cfg.visualization_on)
            {
#if BUILD_TESTING
                if (debug_viz)
                {
                    // annotate_frame() draws inplace
                    viz = cfg.rrd_on ? resized.clone() : resized;
                    vd::visualize(viz, *r, source_label(cfg.source), cfg.wheel_dir,
                                  pipeline.H_world2resized(),
                                  static_cast<float>(ego_v));
                    display_frame = viz;
                }
                else
#endif
                {
                    display_frame = visualization.build_frame(resized, *r, plan, ego_v, pipeline.H_resized(),
                                                              cfg.speed_limit);
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

    if (cfg.rrd_on) logging::Rerun::shutdown(); // flush & close .rrd

    // stop() returns true on a clean shutdown; translate that to a 0 exit code
    // so VisionPilot can be supervised as a batch/oneshot job (a successful run
    // must not exit non-zero).
    return visualization.stop() ? 0 : 1;
}
