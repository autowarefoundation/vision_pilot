#ifndef VISIONPILOT_VEHICLE_INTERFACE_HPP
#define VISIONPILOT_VEHICLE_INTERFACE_HPP


class VehicleInterface
{
public:
    VehicleInterface() = default;
    virtual ~VehicleInterface() = default;

    // Read vehicle speed via CAN frame
    virtual double read() = 0;

    // Send steering and acceleration via CAN frame
    virtual void write(double steering, double acceleration) = 0;

    // Publish the fused lane-center geometry (middleware-specific; default: none).
    // Args mirror LateralFusionEstimate: cte_m +ve = ego right of path,
    // yaw_rad +ve = path heading left, curvature +ve = left turn;
    // stamp_sec = source-image time (negative = unknown).
    virtual void publish_lane_path(double /*stamp_sec*/, bool /*valid*/,
                                   double /*cte_m*/, double /*yaw_rad*/,
                                   double /*curvature*/, bool /*path_valid*/,
                                   double /*path_x_max*/) {}
};


#endif //VISIONPILOT_VEHICLE_INTERFACE_HPP
