#ifndef VISIONPILOT_V4L2_TO_OPENCV_HPP
#define VISIONPILOT_V4L2_TO_OPENCV_HPP

#include <opencv2/opencv.hpp>
#include <cstdint>
#include <mutex>
#include <tuple>
#include <string>
#include <memory>
#include <iostream>

namespace v4l2_interface {

    class V4L2Reader {

        public:


        /**
        * @brief Constructor for V4L2Reader
        *
        * @param device_path Mounting path to V4L2 device (e.g., "/dev/video0")
        @ @param fps Desired FPS for outputing frames. Current default is 10.
        * 
        * Inits the V4L2 camera device with specified parameters and start capture/vis thread.
        * Also logs these details and subscribtion info.
        */    
        explicit V4L2Reader(
            const std::string& device_path,
            uint32_t fps = 10
        );


        /**
        * @brief Destructor for V4L2Reader
        *
        * Properly cleans up camera resources and closes video capture threads.
        */
        ~V4L2Reader();


        // FRAME HANDLINGS


        /**
        * @brief Get latest captured frame from V4L2 device
        *
        * Attempts to read one from the V4L2 stream and applies target FPS throttling to
        * match target FPS. Handles ann timing and sleeping internally.
        *
        * @return A tuple containing:
        *     - bool: true if frame is valid, false otherwise
        *     - cv::Mat: latest captured frame (empty if invalid)
        *
        * Thread-safe with the frame cloned and removed from buffer after retrieval.
        */
        std::tuple<bool, cv::Mat> get_latest_frame();


        /**
        * @brief Check if latest frame is currently available
        * 
        * @return true if a frame is available in the buffer, false otherwise
        */
        bool has_frames() const;


        /**
        * @brief Reset frame buffer (clear latest frame slot)
        * 
        * Kinda useful for resetting state or handling error conditions
        */
        void clear_frame_buffer();


        /**
        * @brief Check if V4L2 camera stream is active
        * 
        * @return true if stream has started capturing frames, false otherwise
        */
        bool is_stream_active() const;


        /**
        * @brief Check if V4L2 device is properly opened
        * 
        * @return true if VideoCapture is successfully opened, false otherwise
        */
        bool is_device_open() const;


        // STATISTICS


        /**
        * @brief Statistics structure for V4L2 capture operations
        */
        struct CaptureStats {
            uint64_t frames_captured = 0;      // Total frames successfully captured
            uint64_t capture_errors = 0;       // Failed frame captures (empty frames)
            uint32_t current_width = 0;        // Current frame width
            uint32_t current_height = 0;       // Current frame height
            double current_fps = 0.0;          // Current FPS setting
            std::string device_path;           // Device path being used
        };

        /**
        * @brief Get statistics about V4L2 capture operations
        * 
        * @return A CaptureStats struct containing:
        *         - frames_captured: total frames captured
        *         - capture_errors: failed frame captures
        *         - current_width/height: frame dimensions
        *         - current_fps: frames per second
        *         - device_path: V4L2 device being used
        */
        CaptureStats get_stats() const;

        /**
        * @brief Reset statistics counters
        */
        void reset_stats();


    private:


        // OpenCV VideoCapture with V4L2 backend
        cv::VideoCapture camera_capture;


        // Single latest-frame slot with thread safety
        mutable std::mutex frame_mutex;
        cv::Mat latest_frame;
        bool has_latest_frame = false;


        // Flag for started stream
        bool is_stream_started = false;


        // Device configuration
        std::string device_path;
        uint32_t target_fps;


        // Display rate throttling
        mutable std::mutex display_time_mutex;
        std::chrono::steady_clock::time_point last_display_time;
        uint32_t display_frame_period_ms;


        // Statistics tracking
        mutable std::mutex stats_mutex;
        CaptureStats stats;


        // Logging helper
        void log_info(const std::string& message) const;
        void log_warning(const std::string& message) const;
        void log_error(const std::string& message) const;
        

    };

};  // namespace v4l2_interface

#endif //VISIONPILOT_V4L2_TO_OPENCV_HPP
