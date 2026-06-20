#ifndef VISIONPILOT_LATERAL_HPP
#define VISIONPILOT_LATERAL_HPP

#include <vector>
#include <Eigen/Core>

// ── Horizon parameters ───────────────────────────────────────────────────────
extern size_t N;
extern double dt;

class LateralPlanner {
public:
    LateralPlanner();
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
    std::vector<double> compute_steering(double Lf,
                                        const Eigen::VectorXd& state,
                                        const Eigen::VectorXd& v_schedule,
                                        const Eigen::VectorXd& kappa_schedule);
};

#endif //VISIONPILOT_LATERAL_HPP
