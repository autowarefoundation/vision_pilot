#pragma once

#include <string>
#include <engine/onnx_engine.hpp>
#include <models/inference.hpp>

namespace vpe = visionpilot::engine;
namespace vpm = visionpilot::models;

enum class SourceMode { Ros2 = 0, V4l2 = 1, Video = 2 };

struct SourceConfig {
    SourceMode  mode         = SourceMode::Video;
    std::string video_path;
    bool        video_realtime = true;
    bool        video_loop     = false;
    std::string input_camera_topic   = "/camera/image";
    std::string v4l2_device  = "/dev/video0";
    int         v4l2_fps     = 10;
};

// struct PipelineConfig {
//     bool initial_inference_check = true;
// };

struct Config {
    vpe::Config engine;
    vpm::Config inference;
    SourceConfig      source;
    // PipelineConfig    pipeline;
    // Print per-frame fusion debug logs
    bool              fusion_debug   = false;
    // Directory with wheel_white.png / wheel_green.png for steering HUD
    std::string       wheel_dir;

    std::string vehicle_speed_topic;
    std::string vehicle_steering_topic;
    std::string vehicle_acceleration_topic;

    double speed_limit;
    double Lf;
};

// Load from key=value .conf file. Expands ~ to $HOME.
// Throws std::runtime_error on missing or invalid config.
Config load_vision_pilot_config();

// Resolve config path from --config <path>, VISIONPILOT_CONFIG env var,
// or default candidates. Returns empty string if nothing found.
std::string resolve_vision_pilot_config_path(int argc, char** argv);

SourceMode parse_source_mode(const std::string& value);

// Short label for debug overlay (video / device path / ROS topic).
std::string source_label(const SourceConfig& source);
