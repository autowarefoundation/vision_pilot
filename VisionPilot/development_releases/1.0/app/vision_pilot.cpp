#include <camera_subscriber/ros2_to_opencv.hpp>

#include <chrono>
#include <iostream>
#include <string>
#include <vector>
#include <thread>

#include <visualization/visualization.hpp>

int main(int argc, char** argv) {
    std::string topic = "/camera/image";
    if (argc > 1) {
        topic = argv[1];
    }

    std::cout << "Hello and welcome to VisionPilot!\n";

    camera_subscriber::ROS2ImageSubscriber camera_subscriber(topic);

    while (true) {
        auto [has_frame, frame] = camera_subscriber.get_latest_frame();

        if (has_frame && !frame.empty()) {
            auto stats = camera_subscriber.get_stats();
            std::vector<std::string> overlay_strs = {
                "topic: " + topic,
                "frames received: " + std::to_string(stats.frames_received),
                "frames dropped: " + std::to_string(stats.frames_dropped),
                "conversion errors: " + std::to_string(stats.conversion_errors)
            };

            visualization::render_frame(
                frame, 
                "VisionPilot", 
                overlay_strs
            );
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }

    visualization::close_windows();

    return 0;
}
