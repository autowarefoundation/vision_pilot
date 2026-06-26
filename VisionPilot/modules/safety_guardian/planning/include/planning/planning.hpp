#ifndef VISIONPILOT_PLANNING_HPP
#define VISIONPILOT_PLANNING_HPP

#include <utility>
#include <vector>
#include <common/types.hpp>
#include <planning/longitudinal_planning.hpp>
#include <planning/lateral_planning.hpp>

class Planner {
public:
    Planner(double speed_limit, double Lf);

    // Unified longitudinal + lateral plan.
    //
    //   cte, epsi      : tracking errors (m, rad)
    //   kappa          : SIGNED (CCW-positive) road curvature at the ego now
    //                    (1/m).  This is the only road information required.
    //                    Sign matters: it must match the MPC's epsi convention
    //                    (see buildSinePath — the value is -x''/(1+x'^2)^1.5,
    //                    not +). Pass a value sampled at the ego's true
    //                    position (interpolated, not snapped to a sample) to
    //                    keep the prediction smooth.
    //   ego_v          : current ego speed (m/s)
    //   cipo_v         : lead speed (m/s); speed_limit when free road
    //   cipo_distance  : gap (m); 9999.0 when free road
    //
    // The planner remembers the previous kappa internally and extrapolates a
    // filtered, clamped curvature rate over the horizon, so it gets a little
    // look-ahead from current + previous samples alone.  Set KAPPA_RATE_GAIN to
    // 0 in planning.cpp for pure constant-curvature behaviour.
    //
    // Returns { acceleration, [delta_0, delta_0, delta_1, ..., delta_{N-2}] }
    Plan compute_plan(
        double cte,
        double epsi,
        double kappa,
        double ego_v,
        bool   has_cipo,
        double cipo_v,
        double cipo_distance);

private:
    double Lf_;            // Front-axle to CoG (m)

    // LongitudinalPlanner::Config cfg_;
    LongitudinalPlanner         longitudinal_planner;
    LateralPlanner              lateral_planner;

    // Curvature-rate estimation (current + previous kappa)
    double prev_kappa_     = 0.0;
    double dkappa_filt_    = 0.0;   // EMA-filtered d(kappa)/ds
    bool   has_prev_kappa_ = false;
};

#endif //VISIONPILOT_PLANNING_HPP
