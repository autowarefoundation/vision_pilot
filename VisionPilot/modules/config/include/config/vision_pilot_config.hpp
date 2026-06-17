#pragma once

#include <engine/onnx_engine.hpp>
#include <string>

namespace vpe = visionpilot::engine;

enum class SourceMode { Ros2 = 0, V4l2 = 1, Video = 2 };

struct SourceConfig {
    SourceMode  mode         = SourceMode::Video;
    std::string video_path;
    bool        video_realtime = true;
    bool        video_loop     = false;
    std::string ros2_topic   = "/camera/image";
    std::string v4l2_device  = "/dev/video0";
    int         v4l2_fps     = 10;
};

struct PipelineConfig {
    bool initial_inference_check = true;
};

// Optional perception -> planner -> control drive loop. Off by default (pure perception +
// display). ego_speed_mps is a placeholder until vehicle-state input (Phase 5); dt_s is
// the control period used for steering/accel shaping.
struct ControlConfig {
    bool        enabled       = false;
    double      ego_speed_mps = 10.0;
    double      dt_s          = 0.10;
    // ROS2 actuation output (ENABLE_ROS2_INTERFACE only): the ControlCommand is published
    // as ackermann_msgs/AckermannDriveStamped on `topic`, stamped with `frame_id`.
    std::string topic         = "/control/ackermann_cmd";
    std::string frame_id      = "base_link";
};

struct VisionPilotConfig {
    std::string       autodrive_model;
    std::string       autosteer_model;
    std::string       autospeed_model;
    vpe::EngineConfig engine_cfg;
    SourceConfig      source;
    PipelineConfig    pipeline;
    // Path to homography YAML — enables ObjectFinder tracker when non-empty.
    std::string       homography_path;
    // Print per-frame fusion debug logs
    bool              fusion_debug   = false;
    // Directory with wheel_white.png / wheel_green.png for steering HUD
    std::string       wheel_dir;
    ControlConfig     control;
};

// Load from key=value .conf file. Expands ~ to $HOME.
// Throws std::runtime_error on missing or invalid config.
VisionPilotConfig load_vision_pilot_config(const std::string& config_path);

// Resolve config path from --config <path>, VISIONPILOT_CONFIG env var,
// or default candidates. Returns empty string if nothing found.
std::string resolve_vision_pilot_config_path(int argc, char** argv);

SourceMode parse_source_mode(const std::string& value);

// Short label for debug overlay (video / device path / ROS topic).
std::string source_label(const SourceConfig& source);
