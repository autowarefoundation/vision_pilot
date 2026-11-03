#include "camera_interface/file_interface.hpp"
#include <thread>
#include <common/types.hpp>

namespace camera_interface
{
    FileInterface::FileInterface(const std::string& path, bool loop, bool realtime)
        : CameraInterface(), path_(path), loop_(loop), realtime_(realtime)
    {
        if (cap_.open(path))
        {
            if (const double fps = cap_.get(cv::CAP_PROP_FPS); realtime_ && (fps > 1.0))
            {
                frame_period_ = std::chrono::duration<double>(1.0 / fps);
            }
        }
    }

    bool FileInterface::is_device_open() const
    {
        return cap_.isOpened();
    }

    cv::Mat FileInterface::get_latest_frame()
    {
        cv::Mat frame;
        bool ok = cap_.read(frame) && !frame.empty();

        if (!ok && loop_)
        {
            if (cap_.set(cv::CAP_PROP_POS_FRAMES, 0.0))
            {
                ok = cap_.read(frame) && !frame.empty();
            }
        }

        if (ok && (frame_period_.count() > 0.0))
        {
            if (t0_.time_since_epoch().count() != 0)
            {
                const auto elapsed = std::chrono::steady_clock::now() - t0_;
                const auto rem = frame_period_ - elapsed;
                if (rem.count() > 0.0)
                {
                    std::this_thread::sleep_for(rem);
                }
            }
            t0_ = std::chrono::steady_clock::now();
        }

        if (!ok)
        {
            frame = cv::Mat();
        }
        return frame;
    }

    CameraStats FileInterface::get_camera_stats() const
    {
        return stats;
    }
} // namespace camera_interface
