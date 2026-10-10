#include <planning/lateral_planning.hpp>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <string>
#include <vector>
#include <cmath>
#include <Eigen/Core>

using CppAD::AD;
using Eigen::VectorXd;

size_t N = 20;

size_t cte_start        = 0;
size_t epsi_start       = cte_start + N;
size_t kappa_road_start = epsi_start + N;
size_t delta_start      = kappa_road_start + N;

class FG_eval
{
public:
    double L_;
    double ds_;
    VectorXd v_schedule;
    VectorXd kappa_schedule;

    FG_eval(double L, double ds, VectorXd v_sched, VectorXd kappa_sched)
        : L_(L), ds_(ds), v_schedule(std::move(v_sched)), kappa_schedule(std::move(kappa_sched))
    {
    }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    void operator()(ADvector& fg, const ADvector& vars)
    {
        fg[0] = 0;

        double v_avg = std::max(v_schedule[0], 0.5);
        double v2    = v_avg * v_avg;

        // UNDERSTEER GRADIENT (K_us):
        // Accounts for tire slip angle under centripetal lateral acceleration (a_y = v^2 * kappa).
        // Typical passenger car K_us is around 0.001 - 0.003 rad/(m/s^2).
        constexpr double K_us = 0.0015;

        // 1. CURVATURE-AWARE STATE WEIGHTS:
        // On heavy curves, boost CTE tracking weight so staying centered outweighs steering smoothness.
        double max_kappa = 0.0;
        for (int s = 0; s < (int)N; ++s) {
            max_kappa = std::max(max_kappa, std::abs(kappa_schedule[s]));
        }

        // Scale CTE weight aggressively when previewing sharp curves (> 0.05 /m)
        const double curve_factor = 1.0 + 100.0 * max_kappa;
        // const double cte_weight   = 40.0 * curve_factor;
        const double cte_weight   = 10.0 * curve_factor;
        const double epsi_weight  = 30.0 * curve_factor;

        // 2. REASONABLE SLEW PENALTY:
        // Capped slew penalty so the solver isn't penalized too heavily for fast turn-in on curve entry.
        const double delta_weight  = 45000.0;
        const double ddelta_weight = (15000.0 * std::min(v2, 25.0)); // Cap multiplier at 25x (~5 m/s or 18 km/h base)

        // State Tracking Cost
        for (int s = 0; s < (int)N; s++)
        {
            AD<double> cte_err = vars[cte_start + s];
            fg[0] += cte_weight * CppAD::pow(cte_err, 2);

            // Progressive penalty for drifting wide (> 0.3m from center)
            fg[0] += cte_weight * 5.0 * CppAD::pow(cte_err, 4);

            fg[0] += epsi_weight * CppAD::pow(vars[epsi_start + s], 2);
        }

        // Steering Effort Cost with Understeer Gradient Compensation
        for (int s = 0; s < (int)N - 1; s++)
        {
            double k = kappa_schedule[s];
            // Feedforward steering = Ackermann angle + Understeer slip angle compensation
            double delta_ff = std::atan(L_ * k) + (K_us * v2 * k);

            fg[0] += delta_weight * CppAD::pow(vars[delta_start + s] - delta_ff, 2);
        }

        // Steering Rate / Slew Cost
        for (int s = 0; s < (int)N - 2; s++)
        {
            fg[0] += ddelta_weight * CppAD::pow(
                vars[delta_start + s + 1] - vars[delta_start + s], 2);
        }

        // Initial State Constraints
        fg[1 + cte_start]        = vars[cte_start];
        fg[1 + epsi_start]       = vars[epsi_start];
        fg[1 + kappa_road_start] = vars[kappa_road_start];

        // Kinematic Spatial Model Constraints
        for (int s = 1; s < (int)N; s++)
        {
            AD<double> cte0   = vars[cte_start + s - 1];
            AD<double> epsi0  = vars[epsi_start + s - 1];
            AD<double> delta0 = vars[delta_start + s - 1];

            AD<double> kappa_road0 = kappa_schedule[s - 1];
            AD<double> kappa_cmd   = CppAD::tan(delta0) / L_;

            fg[1 + cte_start + s] = vars[cte_start + s]
                - (cte0 + CppAD::sin(epsi0) * ds_);

            fg[1 + epsi_start + s] = vars[epsi_start + s]
                - (epsi0 + (kappa_cmd - kappa_road0) * ds_);

            fg[1 + kappa_road_start + s] = vars[kappa_road_start + s]
                - kappa_schedule[s];
        }
    }
};

LateralPlanner::LateralPlanner(const double max_cpu_time_s)
    : max_cpu_time_s_(max_cpu_time_s)
{
}
LateralPlanner::~LateralPlanner() = default;

std::vector<double> LateralPlanner::compute_steering(const double L,
                                                     const VectorXd& state,
                                                     const VectorXd& v_schedule,
                                                     const VectorXd& kappa_schedule)
{
    const double v_curr = v_schedule[0];

    if (std::abs(v_curr) < 0.2)
    {
        return std::vector<double>(N - 1, 0.0);
    }

    // Dynamic spatial step: Ensures lookahead preview horizon (~1.0 sec)
    const double ds = std::max(0.30, (v_curr * 1.0) / static_cast<double>(N));

    typedef CPPAD_TESTVECTOR(double) Dvector;

    const double cte_raw    = state[0];
    const double epsi_raw   = state[1];
    const double kappa_road = state[2];

    // FIX LATENCY COMPENSATION:
    // When turning into a curve (kappa_road > 0), heading relative to road tangent
    // INCREASES by (kappa_cmd - kappa_road)*ds. Using raw feedforward angle delta_ff_0:
    const double delta_ff_0 = std::atan(L * kappa_road);
    const double kappa_cmd_0 = std::tan(delta_ff_0) / L;

    const double cte  = cte_raw + std::sin(epsi_raw) * ds;
    const double epsi = epsi_raw + (kappa_cmd_0 - kappa_road) * ds;

    const size_t n_vars        = N * 3 + (N - 1);
    const size_t n_constraints = N * 3;

    Dvector vars(n_vars);

    // Warm-start spatial trajectory
    double current_cte  = cte;
    double current_epsi = epsi;
    for (size_t i = 0; i < N; ++i)
    {
        vars[cte_start + i]        = current_cte;
        vars[epsi_start + i]       = current_epsi;
        vars[kappa_road_start + i] = kappa_schedule[i];

        current_cte += std::sin(current_epsi) * ds;
    }

    // Warm-start steering at feedforward angles + understeer offset
    constexpr double K_us = 0.0015;
    const double v2 = v_curr * v_curr;
    for (size_t i = 0; i < N - 1; ++i)
    {
        double k = kappa_schedule[i];
        double delta_ff = std::atan(L * k) + (K_us * v2 * k);
        vars[delta_start + i] = delta_ff;
    }

    Dvector vars_lb(n_vars), vars_ub(n_vars);
    for (int i = 0; i < (int)delta_start; i++)
    {
        vars_lb[i] = -1.0e19;
        vars_ub[i] =  1.0e19;
    }

    // Physical steering limits (rad)
    for (int i = delta_start; i < (int)n_vars; i++)
    {
        vars_lb[i] = -0.442; // ~23 degrees wheel angle
        vars_ub[i] =  0.442;
    }

    Dvector con_lb(n_constraints), con_ub(n_constraints);
    for (int i = 0; i < (int)n_constraints; i++) con_lb[i] = con_ub[i] = 0.0;
    con_lb[cte_start]        = con_ub[cte_start]        = cte;
    con_lb[epsi_start]       = con_ub[epsi_start]       = epsi;
    con_lb[kappa_road_start] = con_ub[kappa_road_start] = kappa_road;

    FG_eval fg_eval(L, ds, v_schedule, kappa_schedule);

    std::string options;
    options += "Integer print_level 0\n";
    options += "Sparse true forward\n";
    options += "Sparse true reverse\n";
    options += "Numeric max_cpu_time " + std::to_string(max_cpu_time_s_) + "\n"; // 15 ms on the vehicle

    CppAD::ipopt::solve_result<Dvector> solution;
    CppAD::ipopt::solve<Dvector, FG_eval>(
        options, vars,
        vars_lb, vars_ub,
        con_lb, con_ub,
        fg_eval, solution);

    std::vector<double> result;
    for (int i = 0; i < (int)N - 1; i++)
    {
        result.push_back(solution.x[delta_start + i]);
    }

    return result;
}