#include <iostream>
#include <string>
#include <thread>

#include <sensing/can_interface/include/can_config.hpp>
#include <sensing/can_interface/include/can_reader.hpp>
#include <sensing/can_interface/include/can_writer.hpp>

int main() {
    try {
        // Init Can Read and Write
        can_reader::CanReader reader(DBWConfig::DEFAULT_MODEL);
        can_writer::CanWriter writer(DBWConfig::DEFAULT_MODEL);
        
        std::cout << "System running... Press Ctrl+C to stop.\n";

        while (true) {
            // Read CAN Bus data
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
                                            state.avg_wheel_speed
                                        )) {
                    std::cerr << "Failed to send command" << std::endl;
                    return 1;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(33));
        }
    } 
    catch (const std::exception& e) {
        std::cerr << "\nSystem Exception: " << e.what() << std::endl;
        return -1;
    }
    return 0;
}
