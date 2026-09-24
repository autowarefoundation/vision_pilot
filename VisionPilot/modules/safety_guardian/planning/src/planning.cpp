#include <cmath>
#include <algorithm>
#include <planning/planning.hpp>

// Spatial step size matching the spatial Lateral MPC horizon step (meters)
constexpr double DS = 0.5;

Planner::Planner(const double speed_limit, const double Lf)
    : L_(Lf)
      , longitudinal_planner([&]
      {
          LongitudinalPlanner::Config c;
          c.speed_limit = speed_limit;
          return c;
      }())
{
}

Plan Planner::compute_plan(
    const double cte,
    const double epsi,
    const double kappa,
    const double ego_v,
    const bool has_cipo,
    const double cipo_v,
    const double cipo_distance)
{
    // KAPPA_RATE_GAIN : 1.0 = use estimated curvature rate d(kappa)/ds
    //                   0.0 = constant curvature approximation
    constexpr double KAPPA_RATE_GAIN = 1.0;
    constexpr double RATE_ALPHA = 0.6; // EMA smoothing factor for dkappa_ds
    constexpr double RATE_MAX = 0.15;   // Clamp on |d(kappa)/ds| (1/m^2)

    const double KAPPA_MAX = std::tan(0.436332) / L_; // Maximum achievable curvature (~0.175 /m)

    // Compute spatial derivative d(kappa)/ds using vehicle displacement between loop cycles
    double dkappa_ds = 0.0;
    if (has_prev_kappa_ && KAPPA_RATE_GAIN > 0.0)
    {
        // Cycle displacement along path: ds_cycle = v * dt
        const double ds_cycle = std::max(1e-3, ego_v * dt);
        double raw_dkappa_ds = (kappa - prev_kappa_) / ds_cycle;
        raw_dkappa_ds = std::max(-RATE_MAX, std::min(RATE_MAX, raw_dkappa_ds));

        dkappa_filt_ = RATE_ALPHA * raw_dkappa_ds + (1.0 - RATE_ALPHA) * dkappa_filt_;
        dkappa_ds = KAPPA_RATE_GAIN * dkappa_filt_;
    }
    prev_kappa_ = kappa;
    has_prev_kappa_ = true;

    // Longitudinal Planner
    double acceleration = longitudinal_planner.compute_acceleration(kappa, ego_v, has_cipo, cipo_v, cipo_distance);

    Eigen::VectorXd v_schedule(N);
    {
        double v_plan = ego_v;
        double cipo_plan = cipo_distance;
        for (int i = 0; i < (int)N; i++)
        {
            v_schedule[i] = v_plan;
            double a_plan = longitudinal_planner.compute_acceleration(kappa, v_plan, has_cipo, cipo_v, cipo_plan);
            if (has_cipo)
                cipo_plan = std::max(0.5, cipo_plan + (cipo_v - v_plan) * dt);
            v_plan = std::max(0.0, v_plan + a_plan * dt);
        }
    }

    // ── Spatial Curvature Schedule (Polynomial Reference Curve) ───────────────
    // Fit y(x) = c0 + c1·x + c2·x² + c3·x³ in the path-tangent spatial frame:
    //   c1 = tan(epsi)     (heading offset)
    //   c2 = kappa / 2     (ego initial curvature)
    //   c3 = dkappa_ds / 6 (spatial rate of change of curvature)
    //
    // Sample signed curvature k = y'' / (1 + y'²)^1.5 at fixed spatial steps (DS).
    Eigen::VectorXd kappa_schedule(N);
    {
        const double c1 = std::tan(epsi);
        const double c2 = 0.5 * kappa;
        const double c3 = dkappa_ds / 6.0;
        double x = 0.0; // Spatial distance along path-tangent (meters)

        for (int i = 0; i < (int)N; i++)
        {
            double yp  = c1 + 2.0 * c2 * x + 3.0 * c3 * x * x; // Spatial slope y'(x)
            double ypp = 2.0 * c2 + 6.0 * c3 * x;             // Spatial curvature y''(x)

            double k = ypp / std::pow(1.0 + yp * yp, 1.5);
            kappa_schedule[i] = std::max(-KAPPA_MAX, std::min(KAPPA_MAX, k));

            x += DS; // Advance by fixed spatial increment DS (meters)
        }
    }

    // Lateral Planner (Spatial MPC Execution)
    Eigen::VectorXd state(3);
    state << cte, epsi, kappa;

    auto steering = lateral_planner.compute_steering(L_, state, v_schedule, kappa_schedule);

    // Safety Warnings
    std::vector<Warning> warnings;

    // LLDW / RLDW
    if (cte < -0.5)
    {
        warnings.push_back(Warning::LLDW);
    }
    else if (cte > 0.5)
    {
        warnings.push_back(Warning::RLDW);
    }

    // FCW / AEB
    if (-5.0 <= acceleration && acceleration <= -3.0)
    {
        warnings.push_back(Warning::FCW);
    }
    else if (acceleration < -5.0)
    {
        warnings.push_back(Warning::AEB);
    }

    // Export the predicted speed schedule as the native longitudinal intent.
    std::vector<double> speed_horizon(v_schedule.data(), v_schedule.data() + v_schedule.size());
    return {acceleration, steering, speed_horizon, warnings};
}