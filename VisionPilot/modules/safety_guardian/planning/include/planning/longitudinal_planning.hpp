#ifndef VISIONPILOT_LONGITUDIANL_PLANNING_HPP
#define VISIONPILOT_LONGITUDIANL_PLANNING_HPP

class LongitudinalPlanner {
public:
    struct Config {
        double speed_limit;  // m/s
        double a     = 1.5;    // max comfortable acceleration  (m/s²)
        double b     = 3.0;    // comfortable deceleration      (m/s²)
        double T     = 1.5;    // desired time-headway          (s)
        double s0    = 2.0;    // minimum gap at standstill     (m)
        double delta = 4.0;    // free-road acceleration exponent
        double mu = 0.5;
        double g = 9.81;
    };

    explicit LongitudinalPlanner(const Config& config);

    // Returns the edo acceleration for the current step.
    //
    //   ego_v         : ego speed (m/s)
    //   has_cipo      : CIPO in front
    //   cipo_v        : lead-vehicle speed (m/s); set to speed_limit for free road
    //   cipo_distance : bumper-to-bumper gap (m); use 9999.0 for free road
    double compute_acceleration(double kappa, double ego_v, bool has_cipo, double cipo_v, double cipo_distance);

private:
    Config config_;
};

#endif //VISIONPILOT_LONGITUDIANL_PLANNING_HPP
