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
#include <image_preprocessing/image_preprocessor.hpp>
#include <logging/logger.hpp>
#include <models/inference.hpp>
#include <planning/planning.hpp>
#include <visualization/visualization.hpp>
#include <debug/debug_draw.hpp>

#include "camera_interface/v4l2_camera_interface.hpp"
#include "camera_interface/file_interface.hpp"
#include "vehicle_interface/file_interface.hpp"
#include <fstream>

#if ENABLE_ROS2_INTERFACE
#include <rclcpp/rclcpp.hpp>
#include <vehicle_ros2_interface/vehicle_ros2_interface.hpp>
#endif

namespace ve = visionpilot::engine;
namespace vm = visionpilot::models;
namespace vd = visionpilot::debug;

std::vector<double> readSpeeds(const std::string& filename)
{
    std::vector<double> speeds;
    std::ifstream file(filename);

    if (!file.is_open())
    {
        std::cerr << "Error: could not open file " << filename << std::endl;
        return speeds;
    }

    std::string line;
    while (std::getline(file, line))
    {
        // skip empty lines
        if (line.find_first_not_of(" \t\r\n") == std::string::npos)
        {
            continue;
        }
        try
        {
            double value = std::stod(line);
            speeds.push_back(value);
        }
        catch (const std::exception& e)
        {
            std::cerr << "Warning: skipping invalid line: \"" << line
                << "\" (" << e.what() << ")" << std::endl;
        }
    }

    file.close();
    return speeds;
}

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
    }

    std::shared_ptr<CameraInterface> camera_interface;
    std::shared_ptr<VehicleInterface> vehicle_interface;
#ifdef ENABLE_ROS2_INTERFACE
    rclcpp::init(argc, argv);
    camera_interface = std::make_unique<ROS2ImageSubscriber>(cfg.input_camera_topic);
    vehicle_interface = std::make_shared<VehicleRos2Interface>(cfg.vehicle_speed_topic,
                                                               cfg.vehicle_steering_topic,
                                                               cfg.vehicle_acceleration_topic);
#else
    if (cfg.source.mode == SourceMode::Video)
    {
        camera_interface = std::make_unique<camera_interface::FileInterface>(
            cfg.source.input_video, cfg.source.video_loop, cfg.source.video_realtime);
        vehicle_interface = std::make_shared<FileInterface>(cfg.source.input_vehicle_speed);
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
    vm::InferencePipeline pipeline(engine, cfg.inference);
    Planner planner(cfg.speed_limit, cfg.Lf);

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
    visualization::Visualization visualization({cfg.webrtc_on, cfg.webrtc_port});

    const cv::Size net_size(vm::AutoDrive::NET_W, vm::AutoDrive::NET_H);
    cv::Mat frame, warped, resized;
    bool h_resized_set = false;
    cv::Mat H = load_matrix("H.yaml", "H");
    int frame_number = 0;
    std::vector<double> speeds = readSpeeds("<INPUT_VEHICLE_SPEED_FILE_PATH>");
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

        if (const auto r = pipeline.process(warped, resized))
        {
            pipeline.latency().print();

            const double ego_v = speeds[frame_number++];
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
                "plan: tyre=%.4f rad  accel=%.3f m/s²  |  cte=%.2fm(raw=%.2fm)  |  cipo=%s  dist=%.1f m  vel=%+.2f m/s",
                plan.steering.empty() ? 0.0 : plan.steering[0],
                plan.acceleration,
                cte,
                raw_cte,
                has_cipo ? "true" : "false",
                cipo_dist,
                r->cipo.velocity_ms);

            vehicle_interface->write(
                plan.steering.empty() ? 0.0 : plan.steering[0],
                plan.acceleration);

            if (cfg.visualization_on)
            {
                if (debug_viz)
                    vd::visualize(resized, *r, source_label(cfg.source), cfg.wheel_dir, pipeline.H_world2resized());
                else
                    display_frame = visualization.build_frame(resized, *r, plan, ego_v, pipeline.H_resized(), cfg.speed_limit);
            }
        }
        if (cfg.visualization_on)
        {
            visualization.render_frame(display_frame);
        }
    }

    return 0;
}
