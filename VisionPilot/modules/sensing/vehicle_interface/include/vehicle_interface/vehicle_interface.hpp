#ifndef VISIONPILOT_VEHICLE_INTERFACE_HPP
#define VISIONPILOT_VEHICLE_INTERFACE_HPP


class VehicleInterface
{
public:
    virtual ~VehicleInterface() = default;

    // Read vehicle speed via CAN frame
    virtual double read() = 0;

    // Send steering and acceleration via CAN frame
    virtual void write(double steering, double acceleration) = 0;

protected:
    VehicleInterface() = default;
    VehicleInterface(const VehicleInterface& other) = default;
    VehicleInterface(VehicleInterface&& other) noexcept = default;
    VehicleInterface& operator=(const VehicleInterface& other) & = default;
    VehicleInterface& operator=(VehicleInterface&& other) & noexcept = default;
};


#endif //VISIONPILOT_VEHICLE_INTERFACE_HPP
