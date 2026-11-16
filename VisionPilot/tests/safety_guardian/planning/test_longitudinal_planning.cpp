#include <cstdio>
#include <iostream>
#include <vector>
#include <string>
#include <planning/config.hpp>
#include <planning/longitudinal_planning.hpp>

using namespace planning;

struct Scenario {
    std::string name;
    double ego_velocity;    // m/s
    double ego_position;    // m
    double lead_velocity;   // m/s
    double lead_position;   // m
    double total_time;      // seconds
};

void run_scenario(const Scenario& s, const LongitudinalPlanner::Config& config) {
    LongitudinalPlanner idm(config);

    double ego_pos  = s.ego_position;
    double ego_vel  = s.ego_velocity;
    double lead_pos = s.lead_position;
    constexpr double dt = 0.1;

    std::cout << "\n=== Scenario: " << s.name << " ===\n";
    std::cout << "Time(s) | Ego Pos(m) | Ego Vel(m/s) | Gap(m) | Accel(m/s^2)\n";
    std::cout << "---------------------------------------------------------\n";

    for (double t = 0.0; t <= s.total_time; t += dt) {
        const double gap   = lead_pos - ego_pos;
        const double accel = idm.compute_acceleration(ego_vel, s.lead_velocity, gap);

        std::printf("%7.1f | %10.2f | %12.2f | %6.2f | %12.2f\n",
                    t, ego_pos, ego_vel, gap, accel);

        ego_vel  += accel * dt;
        ego_pos  += ego_vel * dt;
        lead_pos += s.lead_velocity * dt;
    }
}

int main() {
    const LongitudinalPlanner::Config config;

    const std::vector<Scenario> scenarios = {
        { "Closing in on slow leader",   20.0,  0.0, 15.0,  40.0, 20.0 },
        { "Free road, no leader",         5.0,  0.0,  0.0, 999.0, 15.0 },
        { "Already at desired speed",    30.0,  0.0, 30.0, 100.0, 10.0 },
        { "Emergency close gap",         25.0,  0.0,  0.0,  10.0,  5.0 },
        { "Start from standstill",        0.0,  0.0, 15.0,  50.0, 20.0 },
    };

    for (const auto& scenario : scenarios) {
        run_scenario(scenario, config);
    }

    return 0;
}