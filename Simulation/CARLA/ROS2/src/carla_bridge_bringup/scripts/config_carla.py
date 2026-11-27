#!/usr/bin/env python3
"""Spawn the VisionPilot ego + sensors in CARLA and follow it with the spectator.

Works with CARLA 0.9.16 and CARLA 1.0 (UE5). Pure CARLA PythonAPI — no ROS. Needs only
the `carla` wheel matching the server version (drive.sh stages it from $CARLA_ROOT).
Ego telemetry (speed, max_steer) is published as ROS 2 topics by the `ego_telemetry`
node inside the bridge container.

  * Run the server with --ros2. Sensors are enabled for CARLA's native ROS 2
    (published by the server itself).
  * --carla-version 0.9.16|1.0 picks the defaults below; without it the server version does.
  * Rig JSON: -f, otherwise config/carla_0.9.16.json for a 0.9.x server and
    config/carla_1.0.json for 1.0. Each JSON uses its own version's blueprint IDs.
  * Map: -m / CARLA_MAP, else the JSON "map" key, else Town04 (0.9.x) /
    Town04_Opt (1.0, where Town01-07 ship only as layered *_Opt maps).
  * 1.0: the ego sets ros2_ackermann_control=true so the server subscribes
    /carla/<id>/ackermann_control_cmd (the 1.0 default is false). 0.9.16 has no such
    attribute (Ackermann control is always on) and it is skipped with a warning.
The spawn point comes from the rig JSON ("spawn_index"); SPAWN_INDEX env overrides.
"""

import argparse
import json
import logging
import math
import os
import random
import signal
import time

import carla


def _find_config_dir():
    """Nearest ancestor `config/` holding the rig JSONs; works from src/ and colcon install/."""
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        cand = os.path.join(d, "config")
        if os.path.isfile(os.path.join(cand, "carla_1.0.json")):
            return cand
        parent = os.path.dirname(d)
        if parent == d:
            return None
        d = parent


CONFIG_DIR = _find_config_dir()

# Defaults per server major version: (rig JSON, map).
VERSION_DEFAULTS = {
    "0": ("carla_0.9.16.json", "Town04"),
    "1": ("carla_1.0.json", "Town04_Opt"),
}

# FRAMES counts frames per sensor when --count-frames is given (debug aid).
FRAMES = {}


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
    return server_ver


def _version_defaults(server_ver):
    """(default rig JSON path, default map) for the server's major version."""
    major = server_ver.lstrip("v").split(".")[0]
    if major not in VERSION_DEFAULTS:
        logging.warning("Unknown CARLA server version %s, using 1.0 defaults", server_ver)
        major = "1"
    rig, map_name = VERSION_DEFAULTS[major]
    if CONFIG_DIR is None:
        raise FileNotFoundError(
            "No config/{} found above {}; pass -f".format(rig, os.path.dirname(os.path.abspath(__file__)))
        )
    return os.path.join(CONFIG_DIR, rig), map_name


def _find_blueprint(bp_library, bp_id, kind):
    """filter()[0] with a readable error."""
    if not bp_id:
        raise ValueError("{} entry in the rig JSON has no \"type\"".format(kind))
    matches = bp_library.filter(bp_id)
    if not matches:
        prefix = bp_id.split(".")[0] + ".*"
        raise ValueError(
            "No {} blueprint matches '{}'. Available: {}".format(
                kind, bp_id, ", ".join(sorted(b.id for b in bp_library.filter(prefix)))
            )
        )
    return matches[0]


def _set_attr(bp, key, value):
    """set_attribute that warns instead of raising when 1.0 dropped/renamed an attribute."""
    if bp.has_attribute(key):
        bp.set_attribute(key, str(value))
    else:
        logging.warning("%s has no attribute '%s' in this CARLA version, skipping", bp.id, key)


def _setup_vehicle(world, config):
    logging.debug("Spawning vehicle: {}".format(config.get("type")))

    bp_library = world.get_blueprint_library()
    map_ = world.get_map()

    bp = _find_blueprint(bp_library, config.get("type"), "vehicle")
    _set_attr(bp, "role_name", config.get("id"))
    _set_attr(bp, "ros_name", config.get("id"))
    # 1.0: opt in to the server subscribing /carla/<id>/ackermann_control_cmd natively
    # (default false). 0.9.16 has no such attribute and always subscribes.
    _set_attr(bp, "ros2_ackermann_control", "true")

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

    return world.spawn_actor(bp, spawn_pt, attach_to=None), idx


def _setup_sensors(world, vehicle, sensors_config, count_frames=False):
    bp_library = world.get_blueprint_library()

    sensors = []
    for sensor in sensors_config:
        logging.debug("Spawning sensor: {}".format(sensor))

        bp = _find_blueprint(bp_library, sensor.get("type"), "sensor")
        _set_attr(bp, "ros_name", sensor.get("id"))
        _set_attr(bp, "role_name", sensor.get("id"))
        for key, value in sensor.get("attributes", {}).items():
            _set_attr(bp, str(key), value)

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

        actor = world.spawn_actor(bp, wp, attach_to=vehicle)
        sensors.append(actor)

        # Same as 0.9.16: the server publishes this sensor on native ROS 2.
        actor.enable_for_ros()

        # Optional debug: also stream to this client and count frames.
        if count_frames:
            actor.listen(
                lambda data, sid=sensor.get("id"): FRAMES.__setitem__(sid, FRAMES.get(sid, 0) + 1)
            )

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
                # avoid the hero's spawn point
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
    npc_vehicles = []
    traffic_manager = None
    original_settings = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(120.0)  # 1.0 map loads (World Partition / _Opt) can be slow
        server_ver = _check_versions(client)
        if args.carla_version and args.carla_version.split(".")[0] != server_ver.lstrip("v").split(".")[0]:
            logging.warning(
                "--carla-version %s but the server reports %s; using %s defaults",
                args.carla_version, server_ver, args.carla_version,
            )
        default_rig, default_map = _version_defaults(args.carla_version or server_ver)

        rig_file = args.file or default_rig
        logging.info("Rig JSON: %s", rig_file)
        with open(rig_file) as f:
            config = json.load(f)

        map_name = args.map or os.environ.get("CARLA_MAP") or config.get("map") or default_map
        current = client.get_world().get_map().name.split("/")[-1]
        if current != map_name:
            available = [m.split("/")[-1] for m in client.get_available_maps()]
            if map_name not in available:
                raise ValueError(
                    "Map '{}' not on this server. Available: {}".format(
                        map_name, ", ".join(sorted(available))
                    )
                )
            logging.info("Loading map %s (current: %s)", map_name, current)
            client.load_world(map_name)
        else:
            logging.info("Map %s already loaded", current)

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

        vehicle, hero_idx = _setup_vehicle(world, config)
        sensors = _setup_sensors(world, vehicle, config.get("sensors", []), args.count_frames)

        # Spawn additional vehicles
        npc_vehicles = _setup_npc_traffic(world, traffic_manager, config, hero_idx)

        world.tick()  # initial tick to settle the world before autopilot/spectator setup

        if args.autopilot:
            vehicle.set_autopilot(True, traffic_manager.get_port())

        spectator = world.get_spectator()

        logging.info("Running... (ego up; telemetry is published by the bridge container)")

        TARGET_HZ = 10.0
        TARGET_PERIOD = 1.0 / TARGET_HZ

        t_end = time.time() + args.duration if args.duration else None
        while t_end is None or time.time() < t_end:
            loop_start = time.time()

            world.tick()  # advances sim by exactly fixed_delta_seconds; blocks until the
            # server has finished the step (including sensor captures)
            _follow_vehicle(world, vehicle, spectator)

            elapsed = time.time() - loop_start
            sleep_time = TARGET_PERIOD - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        if t_end is not None:
            loc = vehicle.get_location()
            logging.info("frames per sensor: %s; ego at (%.1f, %.1f)", FRAMES, loc.x, loc.y)

    except KeyboardInterrupt:
        print("\nCancelled by user. Bye!")

    finally:
        # Block further KeyboardInterrupts during cleanup
        signal.signal(signal.SIGINT, signal.SIG_IGN)

        try:
            if original_settings and world is not None:
                logging.info("Restoring original settings")
                world.apply_settings(original_settings)
            if traffic_manager is not None:
                traffic_manager.set_synchronous_mode(False)

            for sensor in sensors:
                if sensor.is_alive:
                    logging.debug("Destroying sensor: {}".format(sensor.type_id))
                    if args.count_frames:
                        sensor.stop()
                    sensor.destroy()

            for npc in npc_vehicles:
                if npc.is_alive:
                    npc.destroy()

            if vehicle is not None and vehicle.is_alive:
                logging.debug("Destroying vehicle: {}".format(vehicle.type_id))
                vehicle.destroy()

        finally:
            # Re-enable KeyboardInterrupt handling
            signal.signal(signal.SIGINT, signal.default_int_handler)


if __name__ == "__main__":
    argparser = argparse.ArgumentParser(description="CARLA ROS2 native (0.9.16 and 1.0)")
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
    argparser.add_argument(
        "-f",
        "--file",
        default="",
        help="rig JSON to load (default: config/carla_0.9.16.json for a 0.9.x server, "
        "config/carla_1.0.json for 1.0)",
    )
    argparser.add_argument(
        "--carla-version",
        default="",
        choices=["", "0.9.16", "1.0"],
        help="pick the default rig JSON and map for this CARLA version "
        "(default: detect from the server)",
    )
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
    argparser.add_argument(
        "-m",
        "--map",
        default="",
        help="map to load (default: env CARLA_MAP, else the rig JSON \"map\", else Town04 "
        "on 0.9.x / Town04_Opt on 1.0)",
    )
    argparser.add_argument(
        "--duration", type=float, default=0.0, help="stop after N seconds (0 = run forever)"
    )
    argparser.add_argument(
        "--count-frames",
        action="store_true",
        help="also listen() on sensors and log frame counts (debug; adds client load)",
    )

    args = argparser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format="%(levelname)s: %(message)s", level=log_level)

    logging.info("Listening to server %s:%s", args.host, args.port)

    main(args)