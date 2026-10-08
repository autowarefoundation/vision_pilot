#ifndef VISIONPILOT_SPEED_FILE_INTERFACE_HPP
#define VISIONPILOT_SPEED_FILE_INTERFACE_HPP

#include <string>
#include <vector>
#include <vehicle_interface/vehicle_interface.hpp>

namespace vehicle_interface
{
    class FileInterface : public VehicleInterface
    {
    public:
        FileInterface(const std::string& filename, bool loop);
        ~FileInterface() override = default;

        // Read vehicle speed via CAN frame
        double read() override;

        // Send steering and acceleration via CAN frame
        void write(double steering, double acceleration) override;

    private:
        bool loop_;
        std::vector<double> speeds_;
        int frame_cnt_ = 0;
    };
} // namespace vehicle_interface

#endif //VISIONPILOT_SPEED_FILE_INTERFACE_HPP
