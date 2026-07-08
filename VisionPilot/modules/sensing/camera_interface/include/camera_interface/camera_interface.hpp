#ifndef VISIONPILOT_CAMERA_INTERFACE_HPP
#define VISIONPILOT_CAMERA_INTERFACE_HPP
#include <string>
#include <tuple>
#include <vector>
#include <opencv2/core/mat.hpp>

class CameraInterface
{
public:
    virtual ~CameraInterface() = default;

    struct CaptureStats
    {
        uint64_t frames_captured = 0; // Total frames successfully captured
        uint64_t capture_errors = 0; // Failed frame captures (empty frames)
        uint32_t current_width = 0; // Current frame width
        uint32_t current_height = 0; // Current frame height
        double current_fps = 0.0; // Current FPS setting
        std::string device_path; // Device path being used
        std::string node_name;
        uint64_t frames_received = 0;
        uint64_t frames_dropped = 0;
        std::string last_encoding;
        uint64_t conversion_errors = 0;
    };

    virtual bool is_device_open() const = 0;
    // {false, {}} = no frame yet (live) or stream ended (video, no loop).
    virtual std::tuple<bool, cv::Mat> get_latest_frame() = 0;
    virtual std::vector<std::string> get_overlay() const = 0;
};

#endif //VISIONPILOT_CAMERA_INTERFACE_HPP
