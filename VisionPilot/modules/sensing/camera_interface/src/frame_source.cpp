#include "camera_interface/frame_source.hpp"

#include "camera_interface/video_file_interface.hpp"
#include "camera_interface/v4l2_camera_interface.hpp"
#include <logging/logger.hpp>

#ifdef ENABLE_ROS2_INTERFACE
#include <camera_subscriber/ros2_to_opencv.hpp>
#endif

namespace camera_interface {

std::unique_ptr<CameraInterface> open_frame_source(const SourceConfig& cfg)
{
    switch (cfg.mode) {
    case SourceMode::Video:
        return std::make_unique<VideoFileInterface>(
            cfg.video_path, cfg.video_loop, cfg.video_realtime);

    case SourceMode::V4l2:
        return std::make_unique<V4L2CameraInterface>(
            cfg.v4l2_device, static_cast<uint32_t>(cfg.v4l2_fps));

    case SourceMode::Ros2:
#ifdef ENABLE_ROS2_INTERFACE
        return std::make_unique<ROS2ImageSubscriber>(cfg.input_camera_topic);
#else
        VP_ERROR("ROS2 requested but ENABLE_ROS2_INTERFACE=OFF");
        return nullptr;
#endif

    default:
        VP_ERROR("Unknown source mode");
        return nullptr;
    }
}

}  // namespace camera_interface
