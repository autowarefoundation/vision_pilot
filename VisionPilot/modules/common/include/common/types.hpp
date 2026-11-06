#ifndef VISIONPILOT_TYPES_HPP
#define VISIONPILOT_TYPES_HPP

#include <cstdint>
#include <vector>

struct CameraInfo
{
    std::array<char, 32> device_path; // Device path being used
    std::array<char, 32> node_name;
};

struct CameraStats
{
    uint64_t frames_captured; // Total frames successfully captured
    uint64_t capture_errors; // Failed frame captures (empty frames)
    uint32_t current_width; // Current frame width
    uint32_t current_height; // Current frame height
    double current_fps; // Current FPS setting
    uint64_t frames_received;
    uint64_t frames_dropped;
    uint64_t conversion_errors;
};

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

#endif //VISIONPILOT_TYPES_HPP
