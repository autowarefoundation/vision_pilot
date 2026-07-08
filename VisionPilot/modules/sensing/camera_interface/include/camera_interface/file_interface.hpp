#ifndef VISIONPILOT_VIDEO_FILE_INTERFACE_HPP
#define VISIONPILOT_VIDEO_FILE_INTERFACE_HPP

#include <camera_interface/camera_interface.hpp>
#include <chrono>
#include <opencv2/videoio.hpp>
#include <string>

namespace camera_interface {

class FileInterface : public CameraInterface {
public:
    FileInterface(const std::string& path, bool loop, bool realtime);

    bool is_device_open() const override;
    std::tuple<bool, cv::Mat> get_latest_frame() override;
    std::vector<std::string> get_overlay() const override;

private:
    std::string path_;
    bool loop_;
    bool realtime_;
    cv::VideoCapture cap_;
    std::chrono::duration<double> frame_period_{0};
};

}  // namespace camera_interface

#endif  // VISIONPILOT_VIDEO_FILE_INTERFACE_HPP
