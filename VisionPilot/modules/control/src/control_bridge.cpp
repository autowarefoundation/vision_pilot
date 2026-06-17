#include <control/control_bridge.hpp>

#include <algorithm>

ControlCommand ControlBridge::compute(
  double cte, double epsi, double kappa, bool has_cipo, double cipo_closing_mps,
  double cipo_distance_m, double ego_v_mps, double dt)
{
  // Lead absolute speed = ego + closing rate; clamp to >= 0 so estimator noise near a
  // stopped lead cannot feed the IDM term a non-physical negative speed. Far sentinel
  // distance when no lead.
  const double cipo_v = has_cipo ? std::max(0.0, ego_v_mps + cipo_closing_mps) : ego_v_mps;
  const double cipo_distance = has_cipo ? cipo_distance_m : 9999.0;

  // Planner (safety_guardian MPC) owns the control law; the Controller shapes its
  // steering-angle + acceleration intent for actuation.
  auto [accel, steer_seq] =
    planner_.compute_plan(cte, epsi, kappa, ego_v_mps, has_cipo, cipo_v, cipo_distance);
  const double planner_steer = steer_seq.empty() ? 0.0 : steer_seq.front();
  // Logging is the caller's decision (this is a reusable per-frame hot-path library): the app
  // logs the returned command at whatever cadence it wants.
  return controller_.compute(planner_steer, accel, ego_v_mps, dt);
}
