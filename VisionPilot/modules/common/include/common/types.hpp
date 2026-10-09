#ifndef VISIONPILOT_TYPES_HPP
#define VISIONPILOT_TYPES_HPP

#include <cstdint>
#include <vector>

enum class Warning : uint8_t {
    None        = 0,
    FCW         = 1, // Forward collision warning
    AEB         = 2, // Automatic Emergency Braking
    LLDW        = 3, // Left lane departure warning
    RLDW        = 4  // Right lane departure warning
};

struct Plan {
    double                acceleration;
    std::vector<double>   steering;
    std::vector<Warning>  warnings;
};

struct RadarPoint {
    float range_m     = 0.f;
    float azimuth_rad = 0.f;
    float range_rate  = 0.f;  // relative; negative = closing
};

#endif //VISIONPILOT_TYPES_HPP
