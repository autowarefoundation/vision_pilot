#!/usr/bin/env python3
"""Spawn the VisionPilot ego + sensors in CARLA and follow it with the spectator.

Pure CARLA PythonAPI — no ROS. Needs only the `carla` wheel matching this python
(drive.sh stages it from $CARLA_ROOT). Ego telemetry (speed, max_steer) is published
as ROS 2 topics by the `ego_telemetry` node inside the bridge container, so ROS never
crosses the host/container boundary (host<->container DDS delivery proved unreliable).

Sensors are enabled for CARLA's native --ros2 (camera published by the server itself).
The spawn point comes from the rig JSON ("spawn_index"); SPAWN_INDEX env overrides.
"""

import argparse
import json
import logging
import math
import os
import signal
import time

import carla


def _check_versions(client):
    client_ver = client.get_client_version()
    server_ver = client.get_server_version()
    if client_ver.split("-")[0] != server_ver.split("-")[0]:
        logging.warning(
            "CARLA PythonAPI %s != server %s — API calls may segfault; stage the matching "
            "wheel from $CARLA_ROOT/PythonAPI/carla/dist (see drive.sh)",
            client_ver,
            server_ver,
        )
    else:
        logging.info("CARLA client/server version %s", server_ver)


def _setup_vehicle(world, config):
    logging.debug("Spawning vehicle: {}".format(config.get("type")))

    bp_library = world.get_blueprint_library()
    map_ = world.get_map()

    bp = bp_library.filter(config.get("type"))[0]
    bp.set_attribute("role_name", config.get("id"))
    bp.set_attribute("ros_name", config.get("id"))

    wp_cfg = config.get("spawn_waypoint")
    if wp_cfg:
        wp = map_.get_waypoint_xodr(wp_cfg["road_id"], wp_cfg["lane_id"], wp_cfg["s"])
        spawn_pt = wp.transform
        spawn_pt.location.z += 0.5
        return world.spawn_actor(bp, spawn_pt, attach_to=None)

    spawn_points = map_.get_spawn_points()
    for i in range(len(spawn_points)):
        waypt = map_.get_waypoint(spawn_points[i].location)
        logging.debug(
            "Spawn Point {}: road {} lane {} section {}".format(
                i, waypt.road_id, waypt.lane_id, waypt.section_id
            )
        )

    # Priority: SPAWN_INDEX env > "spawn_index" in the rig JSON > 0.
    default_idx = int(config.get("spawn_index", 0))
    idx = int(os.environ.get("SPAWN_INDEX", default_idx))
    if not 0 <= idx < len(spawn_points):
        raise IndexError(
            "SPAWN_INDEX {} out of range: map {} has {} spawn points (valid 0..{})".format(
                idx, map_.name, len(spawn_points), len(spawn_points) - 1
            )
        )
    logging.info(
        "map %s: using spawn index %d (rig default %d; override with SPAWN_INDEX)",
        map_.name,
        idx,
        default_idx,
    )
    spawn_pt = spawn_points[idx]

    return world.spawn_actor(bp, spawn_pt, attach_to=None)


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
            location=carla.Location(
                x=sensor["spawn_point"]["x"],
                y=-sensor["spawn_point"]["y"],
                z=sensor["spawn_point"]["z"],
            ),
            rotation=carla.Rotation(
                roll=sensor["spawn_point"]["roll"],
                pitch=-sensor["spawn_point"]["pitch"],
                yaw=-sensor["spawn_point"]["yaw"],
            ),
        )

        sensors.append(world.spawn_actor(bp, wp, attach_to=vehicle))

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

    offset_location = carla.Location(x=location.x + dx, y=location.y + dy, z=location.z + height)

    spectator.set_transform(carla.Transform(offset_location, rotation))


import random

def _road_spawn_point(map_, npc, n):
    """n-th car along OpenDRIVE road "road_id", "spacing_m" apart, alternating "lane_ids"."""
    lanes = npc["lane_ids"]
    s = npc["s_start"] + n * npc["spacing_m"] / len(lanes)
    wp = map_.get_waypoint_xodr(npc["road_id"], lanes[n % len(lanes)], s)
    if wp is None:
        return None
    spawn_pt = wp.transform
    spawn_pt.location.z += 0.5
    return spawn_pt


MPH_TO_MS = 0.44704
DEFAULT_EGO_SPEED_MS = 33.3  # VisionPilot default speed_limit
DEFAULT_SPEED_LIMIT_KMH = 30.0


def _ego_speed_mph(config):
    return config.get("ego_speed_mph", DEFAULT_EGO_SPEED_MS / MPH_TO_MS)


def _npc_speed_diff_percent(actor, npc, config):
    """TM speed difference for "speed_diff_percent", or for ego_speed_mph - "npc_speed_delta_mph"."""
    if "speed_diff_percent" in npc:
        return npc["speed_diff_percent"]
    if "npc_speed_delta_mph" not in config:
        return None
    target_ms = (_ego_speed_mph(config) - config["npc_speed_delta_mph"]) * MPH_TO_MS
    limit_kmh = actor.get_speed_limit() or DEFAULT_SPEED_LIMIT_KMH
    return (1.0 - target_ms * 3.6 / limit_kmh) * 100.0


def _start_autopilot(traffic_manager, actor, npc, config):
    actor.set_autopilot(True, traffic_manager.get_port())
    diff = _npc_speed_diff_percent(actor, npc, config)
    if diff is not None:
        traffic_manager.vehicle_percentage_speed_difference(actor, diff)


def _setup_npc_traffic(world, traffic_manager, config, hero_spawn_index):
    bp_library = world.get_blueprint_library()
    map_ = world.get_map()
    spawn_points = map_.get_spawn_points()

    npc_vehicles = []
    held = []
    for npc in config.get("npc_vehicles", []):
        count = npc.get("count", 1)
        for n in range(count):
            bp_filter = npc.get("type", "vehicle.*")
            candidates = bp_library.filter(bp_filter)
            if "base_type" in npc:
                candidates = [
                    bp
                    for bp in candidates
                    if bp.get_attribute("base_type").as_str() == npc["base_type"]
                ]
            if not candidates:
                logging.warning("No blueprint matches '%s', skipping", bp_filter)
                continue
            bp = random.choice(candidates)
            if bp.has_attribute("color"):
                color = random.choice(bp.get_attribute("color").recommended_values)
                bp.set_attribute("color", color)

            idx = npc.get("spawn_index")
            if "road_id" in npc:
                spawn_pt = _road_spawn_point(map_, npc, n)
                if spawn_pt is None:
                    logging.warning("npc road position %d not on the map, skipping", n)
                    continue
            elif idx is not None:
                if not 0 <= idx < len(spawn_points):
                    logging.warning("npc spawn_index %d out of range, skipping", idx)
                    continue
                spawn_pt = spawn_points[idx]
            else:
                # avoid the hero's spawn point and any already-used ones
                free = [p for i, p in enumerate(spawn_points) if i != hero_spawn_index]
                spawn_pt = random.choice(free)

            actor = world.try_spawn_actor(bp, spawn_pt)
            if actor is None:
                logging.warning("Failed to spawn NPC (spawn point likely occupied)")
                continue

            if npc.get("autopilot", True):
                if npc.get("hold_until_ego_moves"):
                    held.append((actor, npc))
                else:
                    _start_autopilot(traffic_manager, actor, npc, config)

            npc_vehicles.append(actor)

    logging.info(
        "Spawned %d NPC vehicles; ego speed %.1f mph = VisionPilot speed_limit %.3f m/s",
        len(npc_vehicles),
        _ego_speed_mph(config),
        _ego_speed_mph(config) * MPH_TO_MS,
    )
    return npc_vehicles, held


def main(args):

    world = None
    vehicle = None
    sensors = []
    original_settings = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(60.0)
        _check_versions(client)

        # if args.map and "Town06" not in client.get_world().get_map().name:
        #     logging.info("Loading Town06 map")
        client.load_world(args.map)

        world = client.get_world()

        # Synchronous mode: this client explicitly drives the sim clock via world.tick(),
        # so each step advances by exactly fixed_delta_seconds. This removes the
        # wall-clock jitter seen in async mode (sensor_tick is measured in sim-time,
        # so async render-time variance made camera frame spacing uneven even though
        # the average rate looked close to the sensor_tick target).
        original_settings = world.get_settings()
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.1  # must match sensor_tick in the sensor JSON
        world.apply_settings(settings)

        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)

        with open(args.file) as f:
            config = json.load(f)

        vehicle = _setup_vehicle(world, config)
        sensors = _setup_sensors(world, vehicle, config.get("sensors", []))

        # Spawn additional vehicles
        npc_vehicles, held_npcs = _setup_npc_traffic(world, traffic_manager, config, config.get("spawn_index", 0))

        world.tick()  # initial tick to settle the world before autopilot/spectator setup

        if args.autopilot:
            vehicle.set_autopilot(True)

        spectator = world.get_spectator()

        logging.info("Running... (ego up; telemetry is published by the bridge container)")

        TARGET_HZ = 10.0
        TARGET_PERIOD = 1.0 / TARGET_HZ

        while True:
            loop_start = time.time()

            world.tick()  # advances sim by exactly fixed_delta_seconds; blocks until the
                           # server has finished the step (including sensor captures)
            if held_npcs and math.hypot(vehicle.get_velocity().x, vehicle.get_velocity().y) > 0.5:
                for actor, npc in held_npcs:
                    _start_autopilot(traffic_manager, actor, npc, config)
                held_npcs = []
            _follow_vehicle(world, vehicle, spectator)

            elapsed = time.time() - loop_start
            sleep_time = TARGET_PERIOD - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nCancelled by user. Bye!")

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


if __name__ == "__main__":
    argparser = argparse.ArgumentParser(description="CARLA ROS2 native")
    argparser.add_argument(
        "--host",
        metavar="H",
        default="localhost",
        help="IP of the host CARLA Simulator (default: localhost)",
    )
    argparser.add_argument(
        "--port",
        metavar="P",
        default=2000,
        type=int,
        help="TCP port of CARLA Simulator (default: 2000)",
    )
    argparser.add_argument("-f", "--file", default="", required=True, help="File to be executed")
    argparser.add_argument(
        "-v", "--verbose", action="store_true", dest="debug", help="print debug information"
    )
    argparser.add_argument(
        "-a",
        "--autopilot",
        action="store_true",
        dest="autopilot",
        help="turn on autopilot for the vehicle",
    )
    argparser.add_argument("-m", "--map", default="Town04", help="map to load (default: Town04)")

    args = argparser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format="%(levelname)s: %(message)s", level=log_level)

    logging.info("Listening to server %s:%s", args.host, args.port)

    main(args)