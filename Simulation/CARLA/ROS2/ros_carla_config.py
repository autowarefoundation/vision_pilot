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


def main(args):

    world = None
    vehicle = None
    sensors = []
    original_settings = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(60.0)
        _check_versions(client)

        if args.map and "Town06" not in client.get_world().get_map().name:
            logging.info("Loading Town06 map")
            client.load_world("Town06")

        world = client.get_world()

        # Asynchronous mode: the server self-ticks in real time; the ego_telemetry node
        # (bridge container) samples speed independently over the PythonAPI.
        original_settings = world.get_settings()
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(False)

        with open(args.file) as f:
            config = json.load(f)

        vehicle = _setup_vehicle(world, config)
        sensors = _setup_sensors(world, vehicle, config.get("sensors", []))

        _ = world.wait_for_tick()

        if args.autopilot:
            vehicle.set_autopilot(True)

        spectator = world.get_spectator()

        logging.info("Running... (ego up; telemetry is published by the bridge container)")

        while True:
            _follow_vehicle(world, vehicle, spectator)
            _ = world.wait_for_tick()

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
    argparser.add_argument("-m", "--map", action="store_true", dest="map", help="load Town06 map")

    args = argparser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format="%(levelname)s: %(message)s", level=log_level)

    logging.info("Listening to server %s:%s", args.host, args.port)

    main(args)
