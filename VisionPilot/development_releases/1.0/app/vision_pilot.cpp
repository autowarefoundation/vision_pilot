#include <camera_subscriber/ros2_to_opencv.hpp>
#include <v4l2_interface/v4l2_reader.hpp>

#include <chrono>
#include <iostream>
#include <unordered_map>
#include <string>
#include <vector>
#include <thread>

#include <visualization/visualization.hpp>
#include <visualization/visualization_to_webrtc.hpp>

int main(int argc, char** argv) {

    // For now, first argument is either 0 (for ROS2) or 1 (for V4L2):
    // - If 0: second argument is ROS2 topic name (default: "/camera/image")
    // - If 1:
    //      - Second argument is V4L2 device path (default: "/dev/video0")
    //      - Third argument is FPS (default: 10)
    //
    // Then the following arguments are for the WebRTC streamer config:
    // - Next argument is whether to start WebRTC streamer (0: False or 1: True, default: 0)
    // - Next argument is browser port (default: 8080)

    std::unique_ptr<camera_subscriber::ROS2ImageSubscriber> ros2_subscriber;
    std::unique_ptr<v4l2_interface::V4L2Reader> v4l2_reader;
    
    if (argc < 2) {

        std::cout << "Usage: " << argv[0] << " [mode] [args...]\n";
        std::cout << "  mode: 0 for ROS2, 1 for V4L2\n";
        std::cout << "  For ROS2 mode: [topic_name]\n";
        std::cout << "  For V4L2 mode: [device_path] [fps]\n";
        return 1;

    } else {

        int mode = std::stoi(argv[1]);
        std::unordered_map<std::string, std::string> args_info;


        // ==================================== INPUT MODES ====================================


        // 1. ROS2

        if (mode == 0) {

            std::string topic = "/camera/image";
            if (argc > 2) {
                topic = argv[2];
            };
            std::cout << "Starting in ROS2 mode with topic: " << topic << "\n";

            args_info["topic"] = topic;

            // Init reader instance
            ros2_subscriber = std::make_unique<camera_subscriber::ROS2ImageSubscriber>(topic);

        // 2. V4L2

        } else if (mode == 1) {

            std::string device_path = "/dev/video0";
            uint32_t target_fps = 10;
            if (argc > 2) {
                device_path = argv[2];
            };
            if (argc > 3) {
                target_fps = static_cast<uint32_t>(std::stoi(argv[3]));
            };
            std::cout << "Starting in V4L2 mode with device: " << device_path 
                      << " and FPS: " << target_fps << "\n";

            args_info["device_path"] = device_path;
            args_info["target_fps"] = std::to_string(target_fps);

            // Init reader instance
            v4l2_reader = std::make_unique<v4l2_interface::V4L2Reader>(device_path, target_fps);
            if (!v4l2_reader->is_device_open()) {
                std::cerr << "Failed to open V4L2 device: " << device_path << std::endl;
                return 1;
            };

        // 3. Invalid input mode

        } else {
            std::cout << "Invalid mode. Use 0 for ROS2 or 1 for V4L2.\n";
            return 1;
        };


        // =================================== WEBRTC INIT ===================================


        bool start_webrtc = false;
        uint16_t webrtc_port = 8080;

        if (argc > 4) {
            start_webrtc = (std::stoi(argv[4]) != 0);
        };
        if (argc > 5) {
            webrtc_port = static_cast<uint16_t>(std::stoi(argv[5]));
        };

        std::unique_ptr<visualization::WebRTCStreamer> webrtc_streamer;
        // Disable local preview if WebRTC is enabled to avoid X11/xcb threading issues
        const bool show_local_preview = !start_webrtc;

        if (start_webrtc) {

            std::cout << "Starting WebRTC streamer on port: " << webrtc_port << "\n";

            visualization::WebRTCStreamer::Config webrtc_config;
            webrtc_config.port = webrtc_port;
            webrtc_streamer = std::make_unique<visualization::WebRTCStreamer>(webrtc_config);
            if (!webrtc_streamer->start()) {
                std::cerr << "Failed to start WebRTC streamer." << std::endl;
                return 1;
            }

            std::cout << "Open browser at: " << webrtc_streamer->browser_url() << "\n";
            std::cout << "Local OpenCV preview is disabled while WebRTC is enabled.\n";
        
        } else {
            std::cout << "WebRTC streamer disabled.\n";
        };


        // ==================================== MAIN LOOP ====================================
        

        while (true) {

            // Get latest frame and flag
            bool has_frame = false;
            cv::Mat frame;
            if (mode == 0) {
                auto frame_result = ros2_subscriber->get_latest_frame();
                has_frame = std::get<0>(frame_result);
                frame = std::get<1>(frame_result);
            } else {
                auto frame_result = v4l2_reader->get_latest_frame();
                has_frame = std::get<0>(frame_result);
                frame = std::get<1>(frame_result);
            }

            if (has_frame && !frame.empty()) {

                // Overlap stats strings
                std::vector<std::string> overlay_strs;

                if (mode == 0) {
                    auto stats = ros2_subscriber->get_stats();
                    overlay_strs = {
                        "topic: " + args_info["topic"],
                        "frames received: " + std::to_string(stats.frames_received),
                        "frames dropped: " + std::to_string(stats.frames_dropped),
                        "conversion errors: " + std::to_string(stats.conversion_errors)
                    };
                } else {
                    auto stats = v4l2_reader->get_stats();
                    overlay_strs = {
                        "device: " + args_info["device_path"],
                        "frames captured: " + std::to_string(stats.frames_captured),
                        "capture errors: " + std::to_string(stats.capture_errors),
                        "resolution: " + std::to_string(stats.current_width) + "x" +
                            std::to_string(stats.current_height),
                        "fps: " + std::to_string(static_cast<int>(stats.current_fps))
                    };
                };

                // Render out frame ONLY WHEN not WebRTC streaming to avoid X11/xcb threading issues
                if (show_local_preview) {
                    visualization::render_frame(
                        frame,
                        "VisionPilot",
                        overlay_strs
                    );
                }

                // Push frame to WebRTC streamer if enabled
                if (webrtc_streamer != nullptr) {
                    webrtc_streamer->push_frame(frame);
                };

            };

            std::this_thread::sleep_for(std::chrono::milliseconds(33));
        };

        visualization::close_windows();

    }

    return 0;
}
