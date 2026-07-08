#ifndef VISIONPILOT_FILE_INTERFACE_HPP
#define VISIONPILOT_FILE_INTERFACE_HPP

#include <string>
#include <vector>
#include <vehicle_interface/vehicle_interface.hpp>

class FileInterface : public VehicleInterface
{
public:
    FileInterface(const std::string& filename);
    ~FileInterface() override = default;

    // Read vehicle speed via CAN frame
    double read() override;

    // Send steering and acceleration via CAN frame
    void write(double steering, double acceleration) override;

private:
    std::vector<double> speeds_;
    int frame_cnt_ = 0;
};

#endif //VISIONPILOT_FILE_INTERFACE_HPP
