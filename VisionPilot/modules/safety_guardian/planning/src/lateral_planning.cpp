#include <planning/lateral_planning.hpp>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Core>

using CppAD::AD;
using Eigen::VectorXd;

// Horizon parameters
size_t N  = 20;
double dt = 0.05;

// Variable-vector index offsets
// Layout:  cte[0..N-1] | epsi[N..2N-1] | kappa_road[2N..3N-1] | delta[3N..4N-2]
size_t cte_start        = 0;
size_t epsi_start       = cte_start        + N;
size_t kappa_road_start = epsi_start       + N;
size_t delta_start      = kappa_road_start + N;

// Cost / constraint functor
class FG_eval {
public:
    VectorXd v_schedule;
    VectorXd kappa_schedule;   // precomputed, clamped curvature preview

    FG_eval(double Lf, VectorXd v_schedule, VectorXd kappa_schedule)
        : Lf_(Lf)
        , v_schedule(std::move(v_schedule))
        , kappa_schedule(std::move(kappa_schedule)) {}

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    void operator()(ADvector& fg, const ADvector& vars) {
        fg[0] = 0;

        // Tracking weights
        const double cte_weight   = 2000.0;
        const double epsi_weight  = 1000.0;
        // Steering effort vs. smoothness.
        // delta_weight   penalises absolute steering magnitude.
        // ddelta_weight  penalises steering RATE (jerk).  Raised well above the
        //                magnitude weight to suppress the step-to-step steering
        //                oscillation; tracking stays tight because cte_weight
        //                still dominates.  Tune ddelta_weight up for smoother /
        //                down for snappier steering.
        const double delta_weight  = 100.0;
        const double ddelta_weight = 600.0;

        for (int t = 0; t < (int)N; t++) {
            fg[0] += cte_weight  * CppAD::pow(vars[cte_start  + t], 2);
            fg[0] += epsi_weight * CppAD::pow(vars[epsi_start + t], 2);
        }

        for (int t = 0; t < (int)N - 1; t++)
            fg[0] += delta_weight * CppAD::pow(vars[delta_start + t], 2);

        for (int t = 0; t < (int)N - 2; t++)
            fg[0] += ddelta_weight * CppAD::pow(
                vars[delta_start + t + 1] - vars[delta_start + t], 2);

        // Initial conditions (t = 0)
        fg[1 + cte_start]        = vars[cte_start];
        fg[1 + epsi_start]       = vars[epsi_start];
        fg[1 + kappa_road_start] = vars[kappa_road_start];

        // Dynamics (t = 1 … N-1)
        for (int t = 1; t < (int)N; t++) {
            AD<double> cte0   = vars[cte_start  + t - 1];
            AD<double> epsi0  = vars[epsi_start + t - 1];
            AD<double> delta0 = vars[delta_start + t - 1];

            // Road curvature at the PREVIOUS step is taken straight from the
            // precomputed schedule — not from an internal recurrence — so the
            // value is already clamped to a physically achievable curvature.
            AD<double> kappa_road0 = kappa_schedule[t - 1];

            AD<double> v0        = v_schedule[t - 1];
            AD<double> ds        = v0 * dt;
            AD<double> kappa_cmd = CppAD::tan(delta0) / Lf_;

            // Cross-track error
            fg[1 + cte_start + t] = vars[cte_start + t]
                                   - (cte0 + v0 * CppAD::sin(epsi0) * dt);

            // Heading error
            fg[1 + epsi_start + t] = vars[epsi_start + t]
                                    - (epsi0 + (kappa_cmd - kappa_road0) * ds);

            // Pin the kappa_road state to the schedule (smooth: RHS is a
            // constant, so the constraint Jacobian is trivial for IPOPT).
            fg[1 + kappa_road_start + t] = vars[kappa_road_start + t]
                                         - kappa_schedule[t];
        }
    }
private:
    double Lf_;
};

// LateralPlanner
LateralPlanner::LateralPlanner() = default;
LateralPlanner::~LateralPlanner() = default;

std::vector<double> LateralPlanner::compute_steering(const double Lf,
                                                    const VectorXd& state,
                                                    const VectorXd& v_schedule,
                                                    const VectorXd& kappa_schedule) {
    bool ok = true;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    const double cte        = state[0];
    const double epsi       = state[1];
    const double kappa_road = state[2];

    const size_t n_vars        = N * 3 + (N - 1);
    const size_t n_constraints = N * 3;

    Dvector vars(n_vars);
    for (int i = 0; i < (int)n_vars; i++) vars[i] = 0.0;
    vars[cte_start]        = cte;
    vars[epsi_start]       = epsi;
    vars[kappa_road_start] = kappa_road;

    Dvector vars_lb(n_vars), vars_ub(n_vars);
    for (int i = 0; i < (int)delta_start; i++) {   // states unconstrained
        vars_lb[i] = -1.0e19;
        vars_ub[i] =  1.0e19;
    }
    for (int i = delta_start; i < (int)n_vars; i++) {   // steering ±25 deg
        vars_lb[i] = -0.436332;
        vars_ub[i] =  0.436332;
    }

    Dvector con_lb(n_constraints), con_ub(n_constraints);
    for (int i = 0; i < (int)n_constraints; i++) con_lb[i] = con_ub[i] = 0.0;
    con_lb[cte_start]        = con_ub[cte_start]        = cte;
    con_lb[epsi_start]       = con_ub[epsi_start]       = epsi;
    con_lb[kappa_road_start] = con_ub[kappa_road_start] = kappa_road;

    FG_eval fg_eval(Lf, v_schedule, kappa_schedule);

    std::string options;
    options += "Integer print_level 0\n";
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    options += "Numeric max_cpu_time 0.5\n";

    CppAD::ipopt::solve_result<Dvector> solution;
    CppAD::ipopt::solve<Dvector, FG_eval>(
        options, vars,
        vars_lb, vars_ub,
        con_lb,  con_ub,
        fg_eval, solution);

    // ok &= (solution.status == CppAD::ipopt::solve_result<Dvector>::success);
    // std::cout << "MPC cost: " << solution.obj_value
    //           << (ok ? "" : "  [SOLVER FAILED]") << "\n";

    std::vector<double> result;
    result.push_back(solution.x[delta_start]);
    for (int i = 0; i < (int)N - 1; i++)
        result.push_back(solution.x[delta_start + i]);

    return result;
}