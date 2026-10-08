#include "camera_interface/file_interface.hpp"

#include <thread>

namespace camera_interface {

FileInterface::FileInterface(const std::string& path, bool loop, bool realtime)
    : path_(path), loop_(loop), realtime_(realtime)
{
    cap_.open(path);
    if (!cap_.isOpened()) {
        return;
    }

    const double fps = cap_.get(cv::CAP_PROP_FPS);
    if (realtime_ && fps > 1.0) {
        frame_period_ = std::chrono::duration<double>(1.0 / fps);
    }
}

bool FileInterface::is_device_open() const
{
    return cap_.isOpened();
}

std::tuple<bool, cv::Mat> FileInterface::get_latest_frame()
{
    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) {
        if (!loop_) {
            return {false, {}};
        }
        cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
        if (!cap_.read(frame) || frame.empty()) {
            return {false, {}};
        }
    }

    if (frame_period_.count() > 0) {
        if (t0_.time_since_epoch().count() != 0) {          // not the first frame
            const auto elapsed = std::chrono::steady_clock::now() - t0_;
            const auto rem = frame_period_ - elapsed;
            if (rem.count() > 0) {
                std::this_thread::sleep_for(rem);
            }
        }
        t0_ = std::chrono::steady_clock::now();              // when this frame is released
    }


    return {true, frame};
}

std::vector<std::string> FileInterface::get_overlay() const
{
    return {"video: " + path_};
}

}  // namespace camera_interface
