#include <camera_subscriber/ros2_to_opencv.hpp>
#include <v4l2_interface/v4l2_reader.hpp>

#include <chrono>
#include <iostream>
#include <string>
#include <vector>
#include <thread>

#include <visualization/visualization.hpp>

// Drive by wire
#include "sensing/can_interface/include/can_config.hpp"
#include "sensing/can_interface/include/can_reader.hpp"
#include "sensing/can_interface/include/can_writer.hpp"


int main(int argc, char** argv) {

    // For now, first argument is either 0 (for ROS2) or 1 (for V4L2):
    // - If 0: second argument is ROS2 topic name (default: "/camera/image")
    // - If 1:
    //      - Second argument is V4L2 device path (default: "/dev/video0")
    //      - Third argument is FPS (default: 10)

    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " [mode] [args...]\n";
        std::cout << "  mode: 0 for ROS2, 1 for V4L2\n";
        std::cout << "  For ROS2 mode: [topic_name] [car_model] [dbc_path]\n";
        std::cout << "  For V4L2 mode: [device_path] [fps] [car_model] [dbc_path]\n";
        return 1;
    } else {

        int mode = std::stoi(argv[1]);

        // ==================================== ROS2 MODE ====================================

        if (mode == 0) {

            std::string topic = "/camera/image";
            if (argc > 2) {
                topic = argv[2];
            }
            if (argc > 3) {
                car_model = argv[3];
            }
            if (argc > 4) {
                dbc_path = argv[4];
            }
            std::cout << "Starting in ROS2 mode with topic: " << topic
                      << " and CAN model: " << car_model
                      << " DBC: " << dbc_path << "\n";

            camera_subscriber::ROS2ImageSubscriber ros2_subscriber(topic);
            
            // Drive By Wire Init 
            can_reader::CanReader reader(car_model);
            can_writer::CanWriter writer(car_model);

            // Main loop
            while (true) {
                auto [has_frame, frame] = ros2_subscriber.get_latest_frame();

                if (has_frame && !frame.empty()) {
                    auto stats = ros2_subscriber.get_stats();
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
                };
                
                // Read CAN Bus
                auto state = reader.read_once();
                if (!state.valid) {
                    std::cout << "State invalid, issuing safe stop." << std::endl;
                    if (!writer.send_safe_stop()) {
                        std::cerr << "Failed to send safe stop" << std::endl;
                        return 1;
                    }
                } else {
                    // Send CAN Bus
                    if (!writer.send_command(state.steering_angle,
                                             state.accelerator_pos,
                                             state.brake,
                                             state.vehicle_speed,
                                             state.avg_wheel_speed,
                                            )) {
                        std::cerr << "Failed to send command" << std::endl;
                        return 1;
                    }
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(33));
            };

            visualization::close_windows();

        // ==================================== V4L2 MODE ====================================

        } else if (mode == 1) {

            std::string device_path = "/dev/video0";
            uint32_t target_fps = 10;
            std::string car_model = DBWConfig::DEFAULT_MODEL;
            std::string dbc_path = DBWConfig::DEFAULT_DBC_PATH;
            if (argc > 2) {
                device_path = argv[2];
            };
            if (argc > 3) {
                target_fps = static_cast<uint32_t>(std::stoi(argv[3]));
            };
            if (argc > 4) {
                car_model = argv[4];
            }
            if (argc > 5) {
                dbc_path = argv[5];
            }
            std::cout << "Starting in V4L2 mode with device: " << device_path
                      << " and FPS: " << target_fps
                      << ", CAN model: " << car_model
                      << " DBC: " << dbc_path << "\n";

            // Init reader instance
            v4l2_interface::V4L2Reader v4l2_reader(device_path, target_fps);
            if (!v4l2_reader.is_device_open()) {
                std::cerr << "Failed to open V4L2 device: " << device_path << std::endl;
                return 1;
            }

            // Main Loop
            can_reader::CanReader reader(car_model, dbc_path);
            can_writer::CanWriter writer(car_model, dbc_path);

            while (true) {
                auto [has_frame, frame] = v4l2_reader.get_latest_frame();

                if (has_frame && !frame.empty()) {
                    auto stats = v4l2_reader.get_stats();
                    std::vector<std::string> overlay_strs = {
                        "device: " + device_path,
                        "frames captured: " + std::to_string(stats.frames_captured),
                        "capture errors: " + std::to_string(stats.capture_errors),
                        "resolution: " + std::to_string(stats.current_width) + "x" +
                            std::to_string(stats.current_height),
                        "fps: " + std::to_string(static_cast<int>(stats.current_fps))
                    };

                    visualization::render_frame(frame, "VisionPilot", overlay_strs);
                }
                
                // ==================== Drive By Wire Process ===========================
                // Read CAN Bus
                auto state = reader.read_once();
                if (!state.valid) {
                    std::cout << "State invalid, issuing safe stop." << std::endl;
                    if (!writer.send_safe_stop(command_message_id)) {
                        std::cerr << "Failed to send safe stop" << std::endl;
                        return 1;
                    }
                } else {
                    // Send Can Bus
                    if (!writer.send_command(state.steering_angle,
                                             state.accelerator_pos,
                                             state.brake,
                                             state.vehicle_speed,
                                             state.avg_wheel_speed,)) 
                    {
                        std::cerr << "Failed to send command" << std::endl;
                        return 1;
                    }
                };
            };

            visualization::close_windows();

        // ==================================== INVALID MODE INPUT ====================================

        } else {
            std::cout << "Invalid mode. Use 0 for ROS2 or 1 for V4L2.\n";
            return 1;
        }

    }

    return 0;
}
