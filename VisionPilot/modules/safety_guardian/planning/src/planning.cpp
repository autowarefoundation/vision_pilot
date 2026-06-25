#include <cmath>
#include <algorithm>
#include <planning/planning.hpp>

Planner::Planner(const double speed_limit, const double Lf)
    : Lf_(Lf)
      , cfg_(load_config(speed_limit))
      , longitudinal_planner(cfg_)
{
}

LongitudinalPlanner::Config Planner::load_config(const double speed_limit)
{
    LongitudinalPlanner::Config cfg;
    cfg.speed_limit = speed_limit;
    cfg.a = 1.5;
    cfg.b = 3.0;
    cfg.T = 1.5;
    cfg.s0 = 2.0;
    cfg.delta = 4.0;
    return cfg;
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
    // KAPPA_RATE_GAIN : 1.0 = use the estimated curvature rate (cubic curve,
    //                         a little look-ahead from current + previous kappa)
    //                   0.0 = no rate (quadratic curve ≈ constant curvature;
    //                         simplest, best steady-state CTE).
    constexpr double KAPPA_RATE_GAIN = 1.0;
    constexpr double RATE_ALPHA = 0.3; // EMA gain for the rate (lower = smoother)
    constexpr double RATE_MAX = 0.08; // clamp on |d(kappa)/ds| (1/m^2)

    const double KAPPA_MAX = std::tan(0.436332) / Lf_; // steering-achievable, ≈0.175 /m

    // Curvature rate from current + previous kappa (filtered)
    double dkappa_ds = 0.0;
    if (has_prev_kappa_ && KAPPA_RATE_GAIN > 0.0)
    {
        const double ds = std::max(1e-6, ego_v * dt);
        double raw = (kappa - prev_kappa_) / ds;
        raw = std::max(-RATE_MAX, std::min(RATE_MAX, raw));
        dkappa_filt_ = RATE_ALPHA * raw + (1.0 - RATE_ALPHA) * dkappa_filt_;
        dkappa_ds = KAPPA_RATE_GAIN * dkappa_filt_;
    }
    prev_kappa_ = kappa;
    has_prev_kappa_ = true;

    // Longitudinal planner
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

    // ── Curvature schedule from a local polynomial reference curve ─────────────
    // Fit  y(x) = c0 + c1·x + c2·x² + c3·x³  in the path-tangent frame, where
    //   c1 = tan(epsi)       (heading / slope)
    //   c2 = kappa / 2       (curvature at the ego, small-angle)
    //   c3 = dkappa_ds / 6   (curvature-rate term; 0 => quadratic ≈ constant)
    // then sample its signed curvature  k = y'' / (1 + y'²)^1.5  at each horizon
    // arc-length.  c0 = cte is a pure lateral offset and does not affect
    // curvature, so it is unused.  x advances by v_schedule[i]·dt, so the curve
    // is sampled along the predicted arc length.  The clamp keeps the demanded
    // curvature within what the steering can physically produce.
    Eigen::VectorXd kappa_schedule(N);
    {
        const double c1 = std::tan(epsi);
        const double c2 = 0.5 * kappa;
        const double c3 = dkappa_ds / 6.0;
        double x = 0.0; // longitudinal coord ≈ arc length
        for (int i = 0; i < (int)N; i++)
        {
            double yp = c1 + 2.0 * c2 * x + 3.0 * c3 * x * x; // y'(x)
            double ypp = 2.0 * c2 + 6.0 * c3 * x; // y''(x)
            double k = ypp / std::pow(1.0 + yp * yp, 1.5); // signed curvature
            kappa_schedule[i] = std::max(-KAPPA_MAX, std::min(KAPPA_MAX, k));
            x += v_schedule[i] * dt; // advance by this step's arc length
        }
    }

    // Lateral planner
    Eigen::VectorXd state(3);
    state << cte, epsi, kappa;

    auto steering = lateral_planner.compute_steering(Lf_, state, v_schedule, kappa_schedule);

    // WARNINGS
    std::vector<Warning> warnings;
    // LLDW
    if (cte < -1.0)
    {
        warnings.push_back(Warning::LLDW);
    }

    // RLDW
    if (cte > 1.0)
    {
        warnings.push_back(Warning::RLDW);
    }

    // FCW
    if (-5.0 <= acceleration && acceleration <= -3.0)
    {
        warnings.push_back(Warning::FCW);
    }

    // AEB
    if (acceleration < -5.0) // deccelerate more than 5.0 m/s
    {
        warnings.push_back(Warning::AEB);
    }

    return {acceleration, steering, warnings};
}
