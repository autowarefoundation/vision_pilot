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
        virtual std::tuple<bool, cv::Mat> get_latest_frame() = 0;
        virtual CameraStats get_camera_stats() const = 0;

    protected:
        CameraInterface() = default;
        CameraInterface(const CameraInterface& other);
        CameraInterface(CameraInterface&& other) noexcept;
        CameraInterface& operator=(const CameraInterface& other);
        CameraInterface& operator=(CameraInterface&& other) noexcept;
    };

    // CameraInterface has no data members, so copy/move have nothing to transfer.
    // If data members are added, these bodies must be updated to copy/move them.
    inline CameraInterface::CameraInterface(const CameraInterface&) {}
    inline CameraInterface::CameraInterface(CameraInterface&&) noexcept {}
    inline CameraInterface& CameraInterface::operator=(const CameraInterface&) { return *this; }
    inline CameraInterface& CameraInterface::operator=(CameraInterface&&) noexcept { return *this; }
} // namespace camera_interface

#endif //VISIONPILOT_CAMERA_INTERFACE_HPP
