#!/usr/bin/env python3
"""Relay VisionPilot Float64 actuation -> carla_msgs/CarlaEgoVehicleControl.

VisionPilot (ENABLE_ROS2_INTERFACE=ON) publishes two std_msgs/Float64 commands:
  /vehicle/steering_cmd   tyre angle [rad]
  /vehicle/throttle_cmd   longitudinal acceleration [m/s^2]
and the ego speed arrives on std_msgs/Float64 /vehicle/speed [m/s] (published by
ros_carla_config.py) — the SAME topic VisionPilot's longitudinal planner consumes, so the
bridge watchdog and VP share one speed source of truth.

CARLA has no ros2_ackermann_control, so the ego only accepts carla_msgs/CarlaEgoVehicleControl.
This node caches the latest commands and publishes a CarlaEgoVehicleControl at a fixed rate.

Steering is a direct proportional map of the VisionPilot tyre angle, normalized by the ego's
real front-wheel limit: ros_carla_config.py publishes it (latched) on
/carla_bridge/max_steer_angle after spawn; the `max_steer` parameter is only the fallback.
Longitudinal is a pure FEED-FORWARD map of VisionPilot's acceleration command to
throttle/brake — the bridge runs NO speed controller, because VisionPilot closes the speed
loop. A hard decel below the AEB threshold still forces full brake. Watchdog: brake and hold
until FRESH VisionPilot commands *and* a fresh speed arrive, so the car never drives during
VisionPilot start-up or if either input stalls. The per-tick decision (watchdog + steering
map + accel map + AEB) lives in the pure, unit-tested control_mapping.decide_control.
"""
import rclpy
from carla_msgs.msg import CarlaEgoVehicleControl
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64
from visionpilot_carla_bridge.control_mapping import (
    AccelMap,
    ControlParams,
    decide_control,
)


class CarlaControlRelay(Node):
    def __init__(self):
        super().__init__("carla_control_relay")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("steering_topic", "/vehicle/steering_cmd")
        self.declare_parameter("throttle_topic", "/vehicle/throttle_cmd")
        # Ego speed — same topic VisionPilot consumes; one speed source of truth.
        self.declare_parameter("speed_topic", "/vehicle/speed")
        self.declare_parameter("control_topic", "/carla/hero/vehicle_control_cmd")
        # Latched by ros_carla_config.py with the spawned ego's real front-wheel limit [rad].
        self.declare_parameter("max_steer_topic", "/carla_bridge/max_steer_angle")
        # 100 Hz: the relay re-emits the latest VP command promptly. At the old 20 Hz the 50 ms
        # hold added loop delay that fed the lateral limit cycle.
        self.declare_parameter("control_rate_hz", 100.0)
        # Fallback front-wheel limit if the latched max_steer_topic never arrives.
        self.declare_parameter("max_steer", ControlParams.max_steer)
        self.declare_parameter("steer_gain", ControlParams.steer_gain)
        # Longitudinal: FEED-FORWARD accel -> throttle/brake (NO bridge PID; VP closes the loop).
        self.declare_parameter("throttle_ref_accel", AccelMap.throttle_ref_accel)
        self.declare_parameter("brake_ref_accel", AccelMap.brake_ref_accel)
        self.declare_parameter("accel_throttle_max", AccelMap.throttle_max)
        self.declare_parameter("accel_brake_max", AccelMap.brake_max)
        self.declare_parameter("accel_deadband", AccelMap.accel_deadband)
        # AEB: brake hard when the planner requests decel below this [m/s^2].
        self.declare_parameter("aeb_accel_threshold", -2.0)
        # Watchdog: hold the ego if no VisionPilot command / no speed within this many seconds.
        self.declare_parameter("vp_cmd_timeout", 1.0)
        self.declare_parameter("speed_timeout", 1.0)

        g = self.get_parameter
        steering_topic = g("steering_topic").value
        throttle_topic = g("throttle_topic").value
        speed_topic = g("speed_topic").value
        control_topic = g("control_topic").value
        max_steer_topic = g("max_steer_topic").value
        rate = float(g("control_rate_hz").value)
        self.dt = 1.0 / rate
        self.params = ControlParams(
            max_steer=float(g("max_steer").value),
            steer_gain=float(g("steer_gain").value),
        )
        self.aeb_accel_threshold = float(g("aeb_accel_threshold").value)
        self.vp_cmd_timeout = float(g("vp_cmd_timeout").value)
        self.speed_timeout = float(g("speed_timeout").value)
        self.accel_map = AccelMap(
            throttle_ref_accel=float(g("throttle_ref_accel").value),
            brake_ref_accel=float(g("brake_ref_accel").value),
            throttle_max=float(g("accel_throttle_max").value),
            brake_max=float(g("accel_brake_max").value),
            accel_deadband=float(g("accel_deadband").value),
        )

        # ── State (latest input, held between updates) ────────────────────────
        self.steer_rad = 0.0
        self.accel_mps2 = 0.0
        self.speed = 0.0
        self.last_vp_t = None  # last VisionPilot command time
        self.last_speed_t = None  # last /vehicle/speed time

        self.create_subscription(Float64, steering_topic, self._on_steer, 10)
        self.create_subscription(Float64, throttle_topic, self._on_accel, 10)
        self.create_subscription(Float64, speed_topic, self._on_speed, 10)
        # Latched (transient_local) so the value published at spawn is seen even if the
        # relay starts later.
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Float64, max_steer_topic, self._on_max_steer, latched)
        self.pub = self.create_publisher(CarlaEgoVehicleControl, control_topic, 10)
        self.timer = self.create_timer(self.dt, self._publish)

        self.get_logger().info(
            "relay %s + %s + %s -> %s @ %.0f Hz | steer: gain=%.2f max=%.4f rad (fallback; "
            "latched %s overrides) | accel feed-forward (thr_ref=%.2f brk_ref=%.2f db=%.2f) "
            "-- VP closes speed loop | aeb<%.1f | hold if no VP cmd >%.1fs or no speed >%.1fs"
            % (
                steering_topic,
                throttle_topic,
                speed_topic,
                control_topic,
                rate,
                self.params.steer_gain,
                self.params.max_steer,
                max_steer_topic,
                self.accel_map.throttle_ref_accel,
                self.accel_map.brake_ref_accel,
                self.accel_map.accel_deadband,
                self.aeb_accel_threshold,
                self.vp_cmd_timeout,
                self.speed_timeout,
            )
        )

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_steer(self, msg: Float64):
        self.steer_rad = float(msg.data)
        self.last_vp_t = self._now()

    def _on_accel(self, msg: Float64):
        self.accel_mps2 = float(msg.data)
        self.last_vp_t = self._now()

    def _on_speed(self, msg: Float64):
        self.speed = float(msg.data)
        self.last_speed_t = self._now()

    def _on_max_steer(self, msg: Float64):
        value = float(msg.data)
        if value <= 0.0:
            self.get_logger().warn("ignoring non-positive max_steer_angle %.4f" % value)
            return
        if abs(value - self.params.max_steer) > 1e-6:
            self.get_logger().info(
                "max_steer %.4f -> %.4f rad (from spawned ego)" % (self.params.max_steer, value)
            )
        self.params.max_steer = value

    def _publish(self):
        steer, throttle, brake = decide_control(
            self.steer_rad,
            self.accel_mps2,
            self._now(),
            self.last_vp_t,
            self.last_speed_t,
            self.params,
            self.accel_map,
            aeb_accel_threshold=self.aeb_accel_threshold,
            vp_cmd_timeout=self.vp_cmd_timeout,
            speed_timeout=self.speed_timeout,
        )
        out = CarlaEgoVehicleControl()
        out.header.stamp = self.get_clock().now().to_msg()
        out.throttle = float(throttle)
        out.steer = float(steer)
        out.brake = float(brake)
        out.hand_brake = False
        out.reverse = False
        out.gear = 1
        out.manual_gear_shift = False
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CarlaControlRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
