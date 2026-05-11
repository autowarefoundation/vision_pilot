#include <v4l2_interface/v4l2_reader.hpp>

#include <sstream>
#include <iomanip>
#include <chrono>
#include <thread>


namespace v4l2_interface {

    V4L2Reader::V4L2Reader(
        const std::string& device_path,
        uint32_t fps
    ) : device_path(device_path),
        target_fps(fps),
        is_stream_started(false),
        has_latest_frame(false)
    {
        
        log_info("Initializing V4L2 Reader");
        log_info("  Device Path: " + device_path);
        log_info("  Target FPS: " + std::to_string(fps));

        stats.device_path = device_path;
        stats.current_fps = fps;

        // Init display rate throttling
        display_frame_period_ms = (fps > 0) ? (1000 / fps) : 33;
        last_display_time = std::chrono::steady_clock::now();

        // Parse device number from path (for example, "/dev/video0" => 0)
        int device_number = 0;
        try {
            size_t last_slash = device_path.find_last_of('/');
            if (last_slash != std::string::npos) {
                std::string dev_num_str = device_path.substr(last_slash + 1);
                if (dev_num_str.find("video") != std::string::npos) {
                    device_number = std::stoi(dev_num_str.substr(5));
                }
            }
        } catch (const std::exception& e) {
            log_warning("Could not parse device number from path: " + device_path);
        }

        // Open camera with V4L2 "one-liner" approach (suggested by Zain)
        camera_capture.open(device_number, cv::CAP_V4L2);

        if (!camera_capture.isOpened()) {
            log_error("Failed to open V4L2 device at: " + device_path);
            return;
        }

        // Config stream properties
        // Reference: https://docs.opencv.org/master/d8/dfe/classcv_1_1VideoCapture.html
        
        try {
            // Set codec to MJPEG for better compatibility and efficiency
            camera_capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
            
            // Set FPS
            camera_capture.set(cv::CAP_PROP_FPS, fps);

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

        } catch (const std::exception& e) {
            log_error(std::string("Exception during device configuration: ") + e.what());
        }

    };


    V4L2Reader::~V4L2Reader() {
        if (camera_capture.isOpened()) {
            camera_capture.release();
            log_info("V4L2 device closed and resources released");
        }
    };


    std::tuple<bool, cv::Mat> V4L2Reader::get_latest_frame() {
        
        if (!camera_capture.isOpened()) {
            {
                std::lock_guard<std::mutex> stats_lock(stats_mutex);
                stats.capture_errors++;
            }
            log_warning("Cannot capture frame: V4L2 device is not open");
            return std::make_tuple(false, cv::Mat());
        }

        // Core one-liner frame capture from V4L2 stream
        cv::Mat frame;
        camera_capture >> frame;

        // Declare stream has started
        is_stream_started = true;

        if (frame.empty()) {
            {
                std::lock_guard<std::mutex> stats_lock(stats_mutex);
                stats.capture_errors++;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            return std::make_tuple(false, cv::Mat());
        }

        if (processed_frame.empty()) {
            {
                std::lock_guard<std::mutex> stats_lock(stats_mutex);
                stats.capture_errors++;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            return std::make_tuple(false, cv::Mat());
        }

        // Store frame in single latest-frame slot with thread safety
        {
            std::lock_guard<std::mutex> lock(frame_mutex);

            // Single-slot buffering
            if (has_latest_frame) {
                std::lock_guard<std::mutex> stats_lock(stats_mutex);
            }

            // Store latest frame
            latest_frame = processed_frame.clone();
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
            auto now = std::chrono::steady_clock::now();
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_display_time).count();
            
            if (elapsed_ms >= display_frame_period_ms) {
                last_display_time = now;
                return std::make_tuple(true, processed_frame);
            } else {
                // Sleep until it's time to display the next frame
                std::this_thread::sleep_for(std::chrono::milliseconds(display_frame_period_ms - elapsed_ms));
                last_display_time = std::chrono::steady_clock::now();
                return std::make_tuple(true, processed_frame);
            }
        }
    };


    bool V4L2Reader::has_frames() const {
        std::lock_guard<std::mutex> lock(frame_mutex);
        return has_latest_frame;
    }


    bool V4L2Reader::is_stream_active() const {
        return is_stream_started;
    }


    bool V4L2Reader::is_device_open() const {
        return camera_capture.isOpened();
    }


    void V4L2Reader::clear_frame_buffer() {
        std::lock_guard<std::mutex> lock(frame_mutex);
        latest_frame.release();
        has_latest_frame = false;
        log_info("Frame buffer cleared");
    }


}