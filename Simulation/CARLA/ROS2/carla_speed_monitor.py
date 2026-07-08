#!/usr/bin/env python3
"""Live ground-truth observability for a CARLA drive: ego speed + collisions.

Independent of the ROS2 /vehicle/speed pipeline — reads the hero actor's true
velocity straight from the CARLA PythonAPI and attaches a collision sensor. Prints
one line per sample (speed, location) and a COLLISION line on impact, so a runaway
or crash is visible the instant it happens (stream it via Monitor).
"""
import argparse
import math
import time

import carla


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", default=2000, type=int)
    ap.add_argument("--role-name", default="hero")
    ap.add_argument("--rate", default=5.0, type=float, help="samples per second")
    args = ap.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)
    world = client.get_world()

    ego = None
    for _ in range(60):
        hits = [a for a in world.get_actors().filter("vehicle.*")
                if a.attributes.get("role_name") == args.role_name]
        if hits:
            ego = hits[0]
            break
        time.sleep(0.5)
    if ego is None:
        print("MONITOR: ego not found", flush=True)
        return

    bp = world.get_blueprint_library().find("sensor.other.collision")
    collision = world.spawn_actor(bp, carla.Transform(), attach_to=ego)
    collision.listen(lambda e: print(
        "COLLISION with %s | impulse=%.1f"
        % (e.other_actor.type_id, math.sqrt(e.normal_impulse.x**2 +
           e.normal_impulse.y**2 + e.normal_impulse.z**2)), flush=True))

    print("MONITOR: tracking ego %d" % ego.id, flush=True)
    period = 1.0 / args.rate
    try:
        while True:
            if not ego.is_alive:
                print("MONITOR: ego destroyed", flush=True)
                break
            v = ego.get_velocity()
            loc = ego.get_transform().location
            print("speed=%.2f m/s | loc=(%.1f, %.1f)"
                  % (math.hypot(v.x, v.y), loc.x, loc.y), flush=True)
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        if collision.is_alive:
            collision.destroy()


if __name__ == "__main__":
    main()
