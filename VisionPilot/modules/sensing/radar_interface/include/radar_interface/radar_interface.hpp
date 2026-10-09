#ifndef VISIONPILOT_RADAR_INTERFACE_HPP
#define VISIONPILOT_RADAR_INTERFACE_HPP
#include <vector>
#include <common/types.hpp>

class RadarInterface
{
public:
    RadarInterface() = default;
    virtual ~RadarInterface() = default;

    // Read radar points
    virtual std::vector<RadarPoint> read_points() = 0;
};

#endif //VISIONPILOT_RADAR_INTERFACE_HPP
