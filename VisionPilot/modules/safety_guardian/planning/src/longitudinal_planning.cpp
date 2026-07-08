#include <planning/longitudinal_planning.hpp>
#include <cmath>
#include <algorithm>

LongitudinalPlanner::LongitudinalPlanner(const Config& config)
    : config_(config) {}

double LongitudinalPlanner::compute_acceleration(double kappa, double ego_v, bool has_cipo, double cipo_v, double cipo_distance) {

    // Limit max speed based on road curvature
    double curv_v_max = std::sqrt(config_.mu * config_.g / std::abs(kappa));   // inf when kappa ~ 0, fine
    double speed_limit = std::min(config_.speed_limit, curv_v_max);

    // Closing speed — negative when ego is slower than lead (gap opening)
    double delta_v = cipo_v; // cipo_v relative CIPO vehicle sppeed

    // Wrap the dynamic term in max(0, …).
    // Without this, when v < lead_v, delta_v < 0 making dynamic_term
    // negative, which shrinks s_star below s0 and collapses the interaction
    // term — IDM would then output full free-road acceleration and overshoot.
    double dynamic_term = (ego_v * delta_v) / (2.0 * std::sqrt(config_.a * config_.b));
    double s_star = config_.s0 + std::max(0.0, ego_v * config_.T + dynamic_term);

    // Raise floor to 0.5 m — matches the gap floor in mpc_test.cpp;
    // prevents (s_star / s)² from becoming catastrophically large.
    double s = std::max(0.5, cipo_distance);

    // Free-road term: positive, drives ego toward speed_limit
    double free_road_term = std::pow(ego_v / speed_limit, config_.delta);

    // Interaction term: only active when a real lead vehicle is present.
    double interaction_term  = has_cipo ? std::pow(s_star / s, 2.0) : 0.0;

    return config_.a * (1.0 - free_road_term - interaction_term);
}