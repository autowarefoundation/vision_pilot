#include "camera_interface/v4l2_camera_interface.hpp"

#include <sstream>
#include <iomanip>
#include <chrono>
#include <thread>


namespace camera_interface
{
    V4L2CameraInterface::V4L2CameraInterface(
        const std::string& device_path,
        const uint32_t fps
    ) : CameraInterface(),
        device_path(device_path),
        fps_(static_cast<double>(fps))
    {
        log_info("Initializing V4L2 Reader");
        log_info("  Device Path: " + device_path);
        log_info("  Target FPS: " + std::to_string(fps));

        stats.current_fps = static_cast<double>(fps);

        // Init display rate throttling
        frame_period_ms = (fps > 0U) ? (1000U / fps) : 33U;
        last_display_time = std::chrono::steady_clock::now();

        // Parse device number from path (for example, "/dev/video0" => 0)
        int32_t device_number = 0;
        try
        {
            size_t last_slash = device_path.find_last_of('/');
            if (last_slash != std::string::npos)
            {
                std::string dev_num_str = device_path.substr(last_slash + 1U);
                if (dev_num_str.find("video") != std::string::npos)
                {
                    device_number = std::stoi(dev_num_str.substr(5U));
                }
            }
        }
        catch (const std::exception& e)
        {
            log_warning("Could not parse device number from path: " + device_path);
        }

        // Open camera with V4L2 "one-liner" approach (suggested by Zain)
        if (camera_capture.open(device_number, cv::CAP_V4L2))
        {
            // Config stream properties
            // Reference: https://docs.opencv.org/master/d8/dfe/classcv_1_1VideoCapture.html

            try
            {
                // Set codec to MJPEG for better compatibility and efficiency
                auto fourcc_mjpg =  static_cast<double>(cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
                if (!camera_capture.set(cv::CAP_PROP_FOURCC, fourcc_mjpg))
                {
                    camera_capture.release();
                    throw std::runtime_error("Failed to set CAP_PROP_FOURCC");
                }

                // Set FPS
                if (!camera_capture.set(cv::CAP_PROP_FPS, fps_))
                {
                    camera_capture.release();
                    throw std::runtime_error("Failed to set CAP_PROP_FPS");
                }

                // Get actual configs set by driver
                double actual_width = camera_capture.get(cv::CAP_PROP_FRAME_WIDTH);
                double actual_height = camera_capture.get(cv::CAP_PROP_FRAME_HEIGHT);
                double actual_fps = camera_capture.get(cv::CAP_PROP_FPS);

                stats.current_width = static_cast<uint32_t>(actual_width);
                stats.current_height = static_cast<uint32_t>(actual_height);
                stats.current_fps = actual_fps;

                log_info("V4L2 device configured successfully");
                log_info("  Received sesolution: " +
                    std::to_string(static_cast<int>(actual_width)) + "x" +
                    std::to_string(static_cast<int>(actual_height)));
                log_info("  Received FPS: " + std::to_string(actual_fps));
            }
            catch (const std::exception& e)
            {
                log_error(std::string("Exception during device configuration: ") + e.what());
            }
        }
        else
        {
            log_error("Failed to open V4L2 device at: " + device_path);
        }
    };

    V4L2CameraInterface::~V4L2CameraInterface()
    {
        if (camera_capture.isOpened())
        {
            camera_capture.release();
            log_info("V4L2 device closed and resources released");
        }
    };


    std::tuple<bool, cv::Mat> V4L2CameraInterface::get_latest_frame()
    {
        std::tuple<bool, cv::Mat> result{false, cv::Mat()};

        if (!camera_capture.isOpened())
        {
            {
                std::lock_guard<std::mutex> stats_lock(stats_mutex);
                stats.capture_errors++;
            }
            log_warning("Cannot capture frame: V4L2 device is not open");
        }
        else
        {
            // Core one-liner frame capture from V4L2 stream
            cv::Mat frame;
            camera_capture >> frame;

            // Declare stream has started
            is_stream_started = true;

            if (frame.empty())
            {
                {
                    std::lock_guard<std::mutex> stats_lock(stats_mutex);
                    stats.capture_errors++;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            else
            {
                // Store frame in single latest-frame slot with thread safety
                {
                    std::lock_guard<std::mutex> lock(frame_mutex);

                    // Single-slot buffering
                    if (has_latest_frame)
                    {
                        std::lock_guard<std::mutex> stats_lock(stats_mutex);
                    }

                    // Store latest frame
                    latest_frame = frame.clone();
                    has_latest_frame = true;
                }

                // Update statistics
                {
                    std::lock_guard<std::mutex> stats_lock(stats_mutex);
                    stats.frames_captured++;
                }

                // Apply display rate throttling
                {
                    std::lock_guard<std::mutex> lock(display_time_mutex);
                    const auto now = std::chrono::steady_clock::now();
                    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_display_time).count();
                    const auto period_ms = static_cast<decltype(elapsed_ms)>(frame_period_ms);

                    if (elapsed_ms >= period_ms)
                    {
                        last_display_time = now;
                    }
                    else
                    {
                        // Sleep until it's time to display the next frame
                        std::this_thread::sleep_for(std::chrono::milliseconds(period_ms - elapsed_ms));
                        last_display_time = std::chrono::steady_clock::now();
                    }
                    result = std::make_tuple(true, frame);
                }
            }
        }

        return result;
    }


    bool V4L2CameraInterface::has_frames() const
    {
        std::lock_guard<std::mutex> lock(frame_mutex);
        return has_latest_frame;
    }


    bool V4L2CameraInterface::is_stream_active() const
    {
        return is_stream_started;
    }


    bool V4L2CameraInterface::is_device_open() const
    {
        return camera_capture.isOpened();
    }


    void V4L2CameraInterface::clear_frame_buffer()
    {
        std::lock_guard<std::mutex> lock(frame_mutex);
        latest_frame.release();
        has_latest_frame = false;
        log_info("Frame buffer cleared");
    }


    // Statistics handling


    CameraStats V4L2CameraInterface::get_camera_stats() const
    {
        std::lock_guard<std::mutex> lock(stats_mutex);

        return stats;
    }

    void V4L2CameraInterface::reset_stats()
    {
        std::lock_guard<std::mutex> lock(stats_mutex);
        stats.frames_captured = 0U;
        stats.capture_errors = 0U;
        log_info("Statistics reset");
    }


    // Logging helpers

    void V4L2CameraInterface::log_info(const std::string& message) const
    {
        std::cout << "[V4L2CameraInterface INFO] " << message << std::endl;
    }


    void V4L2CameraInterface::log_warning(const std::string& message) const
    {
        std::cerr << "[V4L2CameraInterface WARNING] " << message << std::endl;
    }


    void V4L2CameraInterface::log_error(const std::string& message) const
    {
        std::cerr << "[V4L2CameraInterface ERROR] " << message << std::endl;
    }
}
