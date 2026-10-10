#ifndef VISIONPILOT_LATERAL_HPP
#define VISIONPILOT_LATERAL_HPP

#include <vector>
#include <Eigen/Core>

// ── Horizon parameters ───────────────────────────────────────────────────────
extern size_t N;

class LateralPlanner {
public:
    // IPOPT's CPU-time budget per solve on the vehicle. A solve that hits it
    // returns its current iterate, so the result depends on machine speed.
    static constexpr double kRealTimeBudgetS = 0.015;

    // max_cpu_time_s: per-solve budget. An offline simulation can raise it so
    // that every solve converges and runs are reproducible.
    explicit LateralPlanner(double max_cpu_time_s = kRealTimeBudgetS);
    ~LateralPlanner();

    // Solve the MPC for the steering sequence.
    //
    //   state          = [cte, epsi, kappa_road]   (initial conditions)
    //   v_schedule     = predicted speed at each horizon step      (N elements)
    //   kappa_schedule = predicted road curvature at each horizon  (N elements)
    //                    step.  Built by the caller from the current curvature
    //                    plus a *clamped* linear preview, so it can never
    //                    exceed the steering-achievable curvature.  This is
    //                    passed in (like v_schedule) rather than reconstructed
    //                    inside the MPC, which keeps the optimiser's
    //                    constraints smooth.
    //
    // Returns [delta_0, delta_0, delta_1, ..., delta_{N-2}]
    std::vector<double> compute_steering(double L,
                                        const Eigen::VectorXd& state,
                                        const Eigen::VectorXd& v_schedule,
                                        const Eigen::VectorXd& kappa_schedule);

private:
    double max_cpu_time_s_;
};

#endif //VISIONPILOT_LATERAL_HPP
