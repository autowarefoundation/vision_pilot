#!/usr/bin/env python3

import argparse
import json
import logging
import os
import signal
import time
import carla

import math

def _setup_vehicle(world, config):
    logging.debug("Spawning vehicle: {}".format(config.get("type")))

    bp_library = world.get_blueprint_library()
    map_ = world.get_map()

    bp = bp_library.filter(config.get("type"))[0]
    bp.set_attribute("role_name", config.get("id"))
    bp.set_attribute("ros_name", config.get("id"))

    # Apply any extra vehicle blueprint attributes from the config. This is how the ego
    # opts into CARLA's native-ROS2 Ackermann control subscriber via
    # "ros2_ackermann_control": true. Without it CARLA only exposes
    # /carla/<ros_name>/vehicle_control_cmd (CarlaEgoVehicleControl) and silently ignores
    # Ackermann messages, so the vehicle never moves.
    for key, value in config.get("attributes", {}).items():
        if not bp.has_attribute(str(key)):
            logging.warning("Vehicle blueprint has no attribute '%s', skipping", key)
            continue
        bp.set_attribute(str(key), str(value).lower() if isinstance(value, bool) else str(value))

    print(map_.name)
    spawn_points = map_.get_spawn_points()
    for i in range(len(spawn_points)):
        waypt = map_.get_waypoint(spawn_points[i].location)
        print ("Spawn Point {}: road {} lane {} section {}".format(i, waypt.road_id, waypt.lane_id, waypt.section_id))
    
    _idx = os.environ.get("CARLA_SPAWN_INDEX")
    if _idx is not None:
        spawn_pt = spawn_points[int(_idx)]
        logging.info("using CARLA_SPAWN_INDEX=%s -> loc=(%.1f,%.1f)",
                     _idx, spawn_pt.location.x, spawn_pt.location.y)
    elif 'Town06' in map_.name:
        spawn_pt = spawn_points[102]
    else:
        # Auto-select a CURVED LANE so the steering can actually be evaluated - a straight
        # lane tells us nothing. Score each spawn by SMOOTH cumulative heading change over
        # ~50 m along a SINGLE lane, EXCLUDING junctions (intersections fake huge heading
        # jumps) and per-step jumps > 15 deg (branch/lane-change artifacts). On Town10HD
        # this deterministically lands on the verified curved-lane spot (spawn idx 63,
        # ~72 deg smooth bend) where EgoLanes detects clean lane lines.
        spawn_pt = spawn_points[5]
        best_score = -1.0
        for sp in spawn_points:
            wp = map_.get_waypoint(sp.location, project_to_road=True)
            if wp is None or wp.is_junction:
                continue
            cur, n, total_turn, prev_yaw, bad = wp, 0, 0.0, None, False
            for _ in range(100):  # ~50 m at 0.5 m step
                nxt = cur.next(0.5)
                if not nxt:
                    break
                cur = nxt[0]
                if cur.is_junction:   # stay on an open road, not a crossing
                    break
                n += 1
                yaw = cur.transform.rotation.yaw
                if prev_yaw is not None:
                    d = abs((yaw - prev_yaw + 180.0) % 360.0 - 180.0)
                    if d > 15.0:      # branch/lane-change artifact - reject this spawn
                        bad = True
                        break
                    total_turn += d
                prev_yaw = yaw
            score = total_turn if (n >= 60 and not bad) else -1.0   # >= 30 m continuous curve
            if score > best_score:
                best_score, spawn_pt = score, sp
        logging.info("auto-selected curved-lane spawn (%.0f deg smooth curvature ahead)", best_score)

    # NOTE: CARLA UE5 0.10 Vehicle has no enable_for_ros() (sensors only), so the
    # ego publishes no native odometry — VisionPilot reads ego_v=0. Odometry is
    # supplied separately (see odom bridge) so the MPC gets cruise speed + lateral
    # (cte/epsi) correction authority, not just curvature feed-forward.
    return world.spawn_actor(
        bp,
        spawn_pt,
        attach_to=None)


def _setup_sensors(world, vehicle, sensors_config):
    bp_library = world.get_blueprint_library()

    sensors = []
    for sensor in sensors_config:
        logging.debug("Spawning sensor: {}".format(sensor))

        bp = bp_library.filter(sensor.get("type"))[0]
        bp.set_attribute("ros_name", sensor.get("id")) 
        bp.set_attribute("role_name", sensor.get("id")) 
        for key, value in sensor.get("attributes", {}).items():
            bp.set_attribute(str(key), str(value))

        wp = carla.Transform(
            location=carla.Location(x=sensor["spawn_point"]["x"], y=-sensor["spawn_point"]["y"], z=sensor["spawn_point"]["z"]),
            rotation=carla.Rotation(roll=sensor["spawn_point"]["roll"], pitch=-sensor["spawn_point"]["pitch"], yaw=-sensor["spawn_point"]["yaw"])
        )

        sensors.append(
            world.spawn_actor(
                bp,
                wp,
                attach_to=vehicle
            )
        )

        sensors[-1].enable_for_ros()

    return sensors

def _follow_vehicle(world, vehicle, spectator):
    vehicle_transform = vehicle.get_transform()
    location = vehicle_transform.location
    rotation = vehicle_transform.rotation

    # Compute offset behind the vehicle in its local frame
    offset_distance = 6.0  # meters behind the vehicle
    height = 2.5  # meters above

    yaw_rad = math.radians(rotation.yaw)

    dx = -offset_distance * math.cos(yaw_rad)
    dy = -offset_distance * math.sin(yaw_rad)

    offset_location = carla.Location(
        x=location.x + dx,
        y=location.y + dy,
        z=location.z + height
    )

    spectator.set_transform(carla.Transform(offset_location, rotation))


def main(args):

    world = None
    vehicle = None
    sensors = []
    original_settings = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(60.0)
        print(client.get_available_maps())
                
        if args.map and 'Town06' not in client.get_world().get_map().name:
            logging.info("Loading Town06 map")
            client.load_world('Town06')

        world = client.get_world()

        original_settings = world.get_settings()
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.02
        world.apply_settings(settings)

        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)

        with open(args.file) as f:
            config = json.load(f)

        vehicle = _setup_vehicle(world, config)
        sensors = _setup_sensors(world, vehicle, config.get("sensors", []))

        _ = world.tick()

        if (args.autopilot):
            vehicle.set_autopilot(True)
        
        spectator = world.get_spectator()

        # Hold the ego stationary until VisionPilot signals control is live. VisionPilot
        # takes ~1 min to load its models / JIT the CUDA kernels; without this the
        # uncommanded ego free-rolls during that window and VisionPilot inherits a
        # mispositioned, moving car (drifts toward the road edge before the first
        # command). We brake + hand_brake every tick until the readiness sentinel file
        # (written by VisionPilot on its first control command, control.ready_sentinel_path)
        # appears, then release. Set CARLA_VP_READY_SENTINEL="" to disable the hold.
        ready_sentinel = os.environ.get("CARLA_VP_READY_SENTINEL", "/tmp/vp_ipc/vp_control_active")
        try:
            if ready_sentinel and os.path.exists(ready_sentinel):
                os.remove(ready_sentinel)  # clear a stale signal from a previous run
        except OSError:
            pass
        ego_released = not ready_sentinel  # empty path => no hold
        if not ego_released:
            logging.info("Holding ego (brake+hand_brake) until VisionPilot ready: %s", ready_sentinel)

        logging.info("Running...")

        # Pace the synchronous loop to real time (fixed_delta_seconds). Without this the
        # loop busy-ticks the server flat out, rendering every sensor on every tick with
        # zero idle, which pins the GPU and can crash the render thread. sensor_tick on
        # the cameras additionally caps their render rate independently of the tick rate.
        dt = world.get_settings().fixed_delta_seconds or 0.05
        last_log = 0.0
        while True:
            loop_start = time.time()
            _follow_vehicle(world, vehicle, spectator)
            # Startup hold: keep the ego still until VisionPilot's first command.
            if not ego_released:
                if os.path.exists(ready_sentinel):
                    ego_released = True
                    # Clear the hold so the native Ackermann controller takes over.
                    vehicle.apply_control(carla.VehicleControl(hand_brake=False))
                    logging.info("VisionPilot control active -> releasing ego hold")
                else:
                    vehicle.apply_control(
                        carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True))
            _ = world.tick()
            if loop_start - last_log > 1.0:
                last_log = loop_start
                v = vehicle.get_velocity()
                spd = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
                ctl = vehicle.get_control()
                loc = vehicle.get_location()
                logging.info(
                    "ego loc=(%.1f,%.1f,%.1f) speed=%.2f throttle=%.2f brake=%.2f steer=%.3f hb=%s",
                    loc.x, loc.y, loc.z, spd, ctl.throttle, ctl.brake, ctl.steer, ctl.hand_brake)
            elapsed = time.time() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')

    finally:
        # Block further KeyboardInterrupts during cleanup
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        
        try:
            if original_settings:
                logging.info("Restoring original settings")
                world.apply_settings(original_settings)

            for sensor in sensors:
                if sensor.is_alive:
                    logging.debug("Destroying sensor: {}".format(sensor.type_id))
                sensor.destroy()

            if vehicle:
                if vehicle.is_alive:
                    logging.debug("Destroying vehicle: {}".format(vehicle.type_id))
                vehicle.destroy()

        finally:
            # Re-enable KeyboardInterrupt handling
            signal.signal(signal.SIGINT, signal.default_int_handler)


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description='CARLA ROS2 native')
    argparser.add_argument('--host', metavar='H', default='localhost', help='IP of the host CARLA Simulator (default: localhost)')
    argparser.add_argument('--port', metavar='P', default=2000, type=int, help='TCP port of CARLA Simulator (default: 2000)')
    argparser.add_argument('-f', '--file', default='', required=True, help='File to be executed')
    argparser.add_argument('-v', '--verbose', action='store_true', dest='debug', help='print debug information')
    argparser.add_argument('-a', '--autopilot', action='store_true', dest='autopilot', help='turn on autopilot for the vehicle')
    argparser.add_argument('-m', '--map', action='store_true', dest='map', help='load Town06 map')

    args = argparser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('Listening to server %s:%s', args.host, args.port)

    main(args)
