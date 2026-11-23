#ifndef VISIONPILOT_PLANNING_CONFIG_HPP
#define VISIONPILOT_PLANNING_CONFIG_HPP

#include <cstddef>

namespace planning
{
    // Spatial step size matching the spatial Lateral MPC horizon step (meters)
    inline constexpr double DS = 0.5;

    // 50Hz control period — used by planning.cpp
    inline constexpr double dt = 0.05;

    inline constexpr std::size_t N{20};
} // namespace visionpilot::models

#endif //VISIONPILOT_PLANNING_CONFIG_HPP
