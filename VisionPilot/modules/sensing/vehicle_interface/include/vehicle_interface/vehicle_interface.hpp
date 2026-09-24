#ifndef VISIONPILOT_VEHICLE_INTERFACE_HPP
#define VISIONPILOT_VEHICLE_INTERFACE_HPP

#include <cstdint>
#include <vector>

// ── Vehicle-independent output descriptions ───────────────────────────────────
//
// A driving command is one compound request from a single camera cycle:
// steering, VP-selected target speed and signed acceleration, plus the
// identity of the source capture and the VP session/cycle. The Safety
// Island integration consumes it through the ROS2 backend; CAN/file
// backends ignore it. Defaults describe an invalid, unsourced sample so a
// partially filled struct can never look like a valid command.

struct DrivingCommandData
{
    bool     has_source_stamp = false;
    int32_t  source_stamp_sec = 0;
    uint32_t source_stamp_nanosec = 0;

    // VP process session (changes on restart) and strictly increasing cycle.
    uint32_t session = 0;
    uint64_t cycle = 0;

    bool   valid = false;
    double steering_tire_angle_rad = 0.0;
    double target_speed_mps = 0.0;
    double acceleration_mps2 = 0.0;
};

// A reference is VP's native path + speed/stop intent from one camera
// cycle. Path and speed horizon belong together; consumers must not pair
// them across cycles.
struct DrivingReferenceData
{
    bool     has_source_stamp = false;
    int32_t  source_stamp_sec = 0;
    uint32_t source_stamp_nanosec = 0;

    uint32_t session = 0;
    uint64_t cycle = 0;

    bool valid = false;

    // Fused lane-center polynomial y = a x² + b x + c in base_link
    // (x forward [m], y left [m]); invalid when path_valid is false.
    bool  path_valid = false;
    float path_a = 0.0f;
    float path_b = 0.0f;
    float path_c = 0.0f;
    float path_x_max_m = 0.0f;

    // Planned speed schedule: v[i] m/s at t = i * horizon_dt_s.
    double horizon_dt_s = 0.0;
    std::vector<double> speed_horizon_mps;
};

class VehicleInterface
{
public:
    VehicleInterface() = default;
    virtual ~VehicleInterface() = default;

    // Read vehicle speed via CAN frame
    virtual double read() = 0;

    // Send steering and acceleration via CAN frame
    virtual void write(double steering, double acceleration) = 0;

    // Publish one compound driving command. Default no-op so CAN/file
    // backends are unchanged.
    virtual void publish_driving_command(const DrivingCommandData& command)
    {
        (void)command;
    }

    // Publish one compound path + speed reference. Default no-op.
    virtual void publish_driving_reference(const DrivingReferenceData& reference)
    {
        (void)reference;
    }
};


#endif //VISIONPILOT_VEHICLE_INTERFACE_HPP