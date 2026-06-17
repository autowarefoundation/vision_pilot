# Control module

Middleware- and vehicle-agnostic layer that turns the Planner's physical intent
(acceleration in m/s², steering angle in rad) into one `ControlCommand`
`{steering_angle_rad, speed_mps, acceleration_mps2}`.

## Contract (upstream issues)

- **#287** — steering angle + vehicle speed are the canonical output, serialized to CAN.
- **#288** — CAN is the primary path; ROS2/Ackermann is an adapter.
- **#269** — VisionPilot emits physical setpoints; the vehicle ECU/DBW owns pedals/torque.

## Components

- **`ControlCommand`** (`control_command.hpp`) — the agnostic output struct.
- **`LongitudinalController`** — integrates the planner's acceleration into a target
  speed, jerk-limited and clamped to a physical envelope. No throttle/brake and no PI
  loop: the ECU/sim closes the actuator loop (#269).
- **`LateralController`** — shapes the MPC steering angle (clamp → slew → low-pass). A
  shaping/actuation layer, **not** a second path-tracker: the MPC owns the steering law,
  and `max_steer_rad` is the vehicle's physical limit, not a control gain.
- **`IDbwSink` + `DbwAdapter`** (`dbw_sink.hpp`, `dbw_adapter.{hpp,cpp}`) — the drive-by-wire
  seam. `IDbwSink` is the abstract transport; `DbwAdapter` owns **all** unit/pedal conversion
  (rad→deg, m/s→km/h, acceleration→accelerator-pos + brake) and the DBW clamps, so the
  controllers stay SI/agnostic. A concrete sink (e.g. comma.ai `CanWriter`, or a fake for
  tests) plugs in behind `IDbwSink`. The accel→pedal map is a documented placeholder.

## Actuation outputs (where the `ControlCommand` goes)

The same `ControlCommand` feeds any of these — all middleware-agnostic:

- **CAN / DBW** — via `DbwAdapter` → `IDbwSink` (#287).
- **ROS2 Ackermann** — `modules/middleware_interfaces/ros2_interface/control_cmd_publisher` (#288).
- Ego speed for the longitudinal loop comes back via `.../vehicle_state_subscriber` (odometry).

## Config (in `vision_pilot.conf`)

`control.enabled` (off by default → pure perception + display), `control.ego_speed_mps` (fallback
until live odometry), `control.dt_s` (control period), plus the ROS2 keys documented in the
publisher/subscriber module READMEs.

## Tests (deterministic, no weights, no simulator)

Build the target and run the binary; each prints `PASS`/`FAIL` and exits non-zero on failure.

- `test_control` — unit + closed-loop kinematic tests for the two controllers.
- `test_dbw_adapter` — `DbwAdapter` conversions/clamps against a fake `IDbwSink`.
- `test_closed_loop` — **closed-loop SIL harness**: the real stack
  (`compute_plan`→longitudinal→lateral) driven around an independent kinematic plant on a
  straight + an R=50 m curve; asserts a sign-convention check, bounded CTE, and speed tracking.
