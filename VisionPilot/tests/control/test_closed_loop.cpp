// Phase 6 — deterministic closed-loop SIL validation harness.
//
// Closes the loop around the REAL control stack (Planner/MPC + the longitudinal/lateral
// shaping controllers, composed exactly as app/control_bridge.cpp does) against an
// independent kinematic plant. No weights, no sim, no ROS2 — a plain executable like
// test_control / test_planning.
//
// The plant integrates the FULL nonlinear Frenet (road-relative) kinematics:
//     cte_dot  = v * sin(epsi)
//     epsi_dot = v * ( tan(delta)/Lf  -  kappa_road * cos(epsi) / (1 - cte*kappa_road) )
// The MPC optimizes against the simplified model (no 1/(1-cte*kappa) or cos(epsi) terms,
// see lateral_planning.cpp), so the plant is genuinely independent of the controller's
// internal model while sharing its exact cte/epsi/kappa sign convention (CCW-positive
// kappa; cte_dot = v sin(epsi); steering left-positive). Lf is bound to the planner's own
// wheelbase via external linkage so the two can never drift apart.
//
// GUARDRAIL (findings §6.7): thresholds are loose physical regression bounds, NOT tuned to
// make a target pass. If a bound (esp. the curve) cannot be met, that is a Phase-1 control
// defect to capture with this test red — do not loosen the bound or tune in-harness.
#include <control/control_command.hpp>
#include <control/lateral_control.hpp>
#include <control/longitudinal_control.hpp>
#include <planning/planning.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

// Defined with external linkage in modules/safety_guardian/planning/src/lateral_planning.cpp.
// Binding it here keeps the plant wheelbase identical to the planner's.
extern const double Lf;

namespace
{

int g_failures = 0;

void check(const std::string & name, bool ok)
{
  std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", name.c_str());
  if (!ok) ++g_failures;
}

bool approx(double a, double b, double tol)
{
  return std::fabs(a - b) <= tol;
}

// Compose the real control stack exactly like app/control_bridge.cpp:11-15.
ControlCommand step_control(
  Planner & planner, LongitudinalController & lon, LateralController & lat, double cte, double epsi,
  double kappa, double ego_v, bool has_cipo, double cipo_v, double cipo_distance, double dt)
{
  auto [accel, steer_seq] =
    planner.compute_plan(cte, epsi, kappa, ego_v, has_cipo, cipo_v, cipo_distance);
  const auto [speed_set, accel_set] = lon.compute(accel, ego_v, dt);
  const double steer = steer_seq.empty() ? 0.0 : lat.compute(steer_seq.front(), dt);
  return {steer, speed_set, accel_set};
}

// Full nonlinear Frenet plant state (road-relative). v held by the caller per scenario.
struct Frenet
{
  double cte = 0.0;
  double epsi = 0.0;

  void advance(double delta, double v, double kappa_road, double dt)
  {
    const double kappa_cmd = std::tan(delta) / Lf;
    double denom = 1.0 - cte * kappa_road;
    if (std::fabs(denom) < 1e-3) denom = (denom < 0.0 ? -1e-3 : 1e-3);  // guard near 1/kappa
    const double epsi_dot = v * (kappa_cmd - kappa_road * std::cos(epsi) / denom);
    cte += v * std::sin(epsi) * dt;
    epsi += epsi_dot * dt;
  }
};

constexpr double kDt = 0.05;            // sim/control period (s)
constexpr double kCruise = 10.0;        // held ego speed for the lateral scenarios (m/s)
const double kSpeedLimit = 60.0 / 3.6;  // free-road IDM target (planning.cpp load_config)

// ---- 1. Sign sanity (open loop, one step) -----------------------------------
// Pins the whole sign convention against the real stack before any closed-loop run.
// Positive cte (vehicle left of path) must command negative (rightward) steering.
void test_sign_convention()
{
  {
    Planner p;
    LongitudinalController lon;
    LateralController lat;
    const ControlCommand c =
      step_control(p, lon, lat, 1.0, 0.0, 0.0, kCruise, false, kCruise, 9999.0, kDt);
    check("sign: +cte commands rightward (negative) steer", c.steering_angle_rad < 0.0);
  }
  {
    Planner p;
    LongitudinalController lon;
    LateralController lat;
    const ControlCommand c =
      step_control(p, lon, lat, -1.0, 0.0, 0.0, kCruise, false, kCruise, 9999.0, kDt);
    check("sign: -cte commands leftward (positive) steer", c.steering_angle_rad > 0.0);
  }
}

// ---- 2. Straight-line recovery ----------------------------------------------
void test_straight_recovery()
{
  Planner p;
  LongitudinalController lon;
  LateralController lat;
  Frenet s;
  s.cte = 1.0;  // 1 m lateral offset, aligned heading
  s.epsi = 0.0;

  double max_abs_cte = std::fabs(s.cte);
  const int steps = 400;  // 400 * 0.05 s * 10 m/s = 200 m
  for (int i = 0; i < steps; ++i) {
    const ControlCommand c =
      step_control(p, lon, lat, s.cte, s.epsi, 0.0, kCruise, false, kCruise, 9999.0, kDt);
    s.advance(c.steering_angle_rad, kCruise, 0.0, kDt);
    max_abs_cte = std::max(max_abs_cte, std::fabs(s.cte));
  }
  check("straight: cte recovers below 0.1 m", std::fabs(s.cte) < 0.1);
  check("straight: no overshoot blow-up (<5%)", max_abs_cte <= 1.05);
}

// ---- 3. Constant-radius curve -----------------------------------------------
void test_constant_curve()
{
  Planner p;
  LongitudinalController lon;
  LateralController lat;
  Frenet s;  // start centered on a straight, then enter the arc

  const double kappa_road = 1.0 / 50.0;  // R = 50 m, left turn (CCW-positive)
  const int warmup = 60;                 // straight first, lets the loop settle
  const int curve_steps = 500;           // ~250 m of arc
  double steady_max_cte = 0.0;
  double overall_max_cte = 0.0;

  for (int i = 0; i < warmup + curve_steps; ++i) {
    const double k = (i < warmup) ? 0.0 : kappa_road;
    const ControlCommand c =
      step_control(p, lon, lat, s.cte, s.epsi, k, kCruise, false, kCruise, 9999.0, kDt);
    s.advance(c.steering_angle_rad, kCruise, k, kDt);
    overall_max_cte = std::max(overall_max_cte, std::fabs(s.cte));
    if (i >= warmup + curve_steps - 100)
      steady_max_cte = std::max(steady_max_cte, std::fabs(s.cte));
  }
  check("curve: steady-state |cte| < 0.5 m", steady_max_cte < 0.5);
  check("curve: no divergence (|cte| stays < 1.0 m)", overall_max_cte < 1.0);
}

// ---- 4. Longitudinal tracking -----------------------------------------------
void test_longitudinal_tracking()
{
  {  // free road from standstill -> converge to the speed limit, accel/jerk bounded
    Planner p;
    LongitudinalController lon;
    LateralController lat;
    const LongitudinalController::Config lc;  // defaults: a_max, jerk_max
    double v = 0.0;
    double prev_a = 0.0;
    bool accel_ok = true;
    bool jerk_ok = true;
    const int steps = 800;  // 40 s
    for (int i = 0; i < steps; ++i) {
      const ControlCommand c = step_control(p, lon, lat, 0.0, 0.0, 0.0, v, false, v, 9999.0, kDt);
      if (c.acceleration_mps2 > lc.a_max + 1e-6 || c.acceleration_mps2 < lc.a_min - 1e-6)
        accel_ok = false;
      if (std::fabs(c.acceleration_mps2 - prev_a) > lc.jerk_max * kDt + 1e-6) jerk_ok = false;
      prev_a = c.acceleration_mps2;
      v = c.speed_mps;
    }
    check("long: converges to speed limit", approx(v, kSpeedLimit, 0.3));
    check("long: acceleration within [a_min, a_max]", accel_ok);
    check("long: jerk within budget", jerk_ok);
  }
  {  // closing on a slower lead -> speed must come down from cruise
    Planner p;
    LongitudinalController lon;
    LateralController lat;
    double v = kSpeedLimit;
    const double lead_v = 5.0;
    double gap = 20.0;
    for (int i = 0; i < 400; ++i) {
      const ControlCommand c = step_control(p, lon, lat, 0.0, 0.0, 0.0, v, true, lead_v, gap, kDt);
      v = c.speed_mps;
      gap = std::max(0.5, gap + (lead_v - v) * kDt);
    }
    check("long: slows for a closer/slower lead", v < kSpeedLimit - 1.0);
  }
}

}  // namespace

int main()
{
  test_sign_convention();
  test_straight_recovery();
  test_constant_curve();
  test_longitudinal_tracking();
  std::printf(
    "\n%s (%d failure%s)\n", g_failures ? "FAILED" : "ALL PASS", g_failures,
    g_failures == 1 ? "" : "s");
  return g_failures == 0 ? 0 : 1;
}
