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

    # Extra blueprint attributes from the rig JSON. On CARLA 0.10 the ego must
    # carry ros2_ackermann_control=True, otherwise the server binds the
    # CarlaEgoVehicleControl subscriber instead of the Ackermann one and every
    # /carla/<id>/ackermann_control_cmd message is silently ignored.
    for key, value in config.get("attributes", {}).items():
        if bp.has_attribute(str(key)):
            bp.set_attribute(str(key), str(value))
            logging.info("vehicle attribute %s = %s", key, value)
        else:
            logging.warning(
                "vehicle blueprint %s has no attribute '%s'; skipping", bp.id, key
            )

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

def _setup_npc_traffic(world, traffic_manager, config, hero_spawn_index):
    bp_library = world.get_blueprint_library()
    map_ = world.get_map()
    spawn_points = map_.get_spawn_points()

    npc_vehicles = []
    for npc in config.get("npc_vehicles", []):
        count = npc.get("count", 1)
        for _ in range(count):
            bp_filter = npc.get("type", "vehicle.*")
            candidates = bp_library.filter(bp_filter)
            if not candidates:
                logging.warning("No blueprint matches '%s', skipping", bp_filter)
                continue
            bp = random.choice(candidates)
            if bp.has_attribute("color"):
                color = random.choice(bp.get_attribute("color").recommended_values)
                bp.set_attribute("color", color)

            idx = npc.get("spawn_index")
            if idx is not None:
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
                actor.set_autopilot(True, traffic_manager.get_port())

            npc_vehicles.append(actor)

    logging.info("Spawned %d NPC vehicles", len(npc_vehicles))
    return npc_vehicles


def main(args):

    world = None
    vehicle = None
    sensors = []
    original_settings = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(60.0)
        _check_versions(client)

        with open(args.file) as f:
            config = json.load(f)

        # Map comes from the rig JSON ("map"); CARLA_MAP env overrides. CARLA 0.10
        # ships the *_Opt variants (Town04_Opt), not the 0.9.16 names (Town04).
        town = os.environ.get("CARLA_MAP", config.get("map", "Town04"))
        if town not in client.get_world().get_map().name:
            logging.info("Loading map %s", town)
            client.load_world(town)
        else:
            logging.info("Map %s already loaded", town)

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

        vehicle = _setup_vehicle(world, config)
        sensors = _setup_sensors(world, vehicle, config.get("sensors", []))

        # Spawn additional vehicles
        npc_vehicles = _setup_npc_traffic(world, traffic_manager, config, config.get("spawn_index", 0))

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

    args = argparser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format="%(levelname)s: %(message)s", level=log_level)

    logging.info("Listening to server %s:%s", args.host, args.port)

    main(args)