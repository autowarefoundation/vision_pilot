#ifndef VISIONPILOT_CAMERA_INTERFACE_HPP
#define VISIONPILOT_CAMERA_INTERFACE_HPP
#include <string>
#include <tuple>
#include <vector>
#include <opencv2/core/mat.hpp>
#include <common/types.hpp>

namespace camera_interface
{
    class CameraInterface
    {
    public:
        virtual ~CameraInterface() = default;

        virtual bool is_device_open() const = 0;
        // {false, {}} = no frame yet (live) or stream ended (video, no loop).
        virtual cv::Mat get_latest_frame() = 0;
        virtual CameraStats get_camera_stats() const = 0;

    protected:
        CameraInterface() = default;
        CameraInterface(const CameraInterface& other) = default;
        CameraInterface(CameraInterface&& other) noexcept = default;
        CameraInterface& operator=(const CameraInterface& other) & = default;
        CameraInterface& operator=(CameraInterface&& other) & noexcept = default;
    };
} // namespace camera_interface

#endif //VISIONPILOT_CAMERA_INTERFACE_HPP
