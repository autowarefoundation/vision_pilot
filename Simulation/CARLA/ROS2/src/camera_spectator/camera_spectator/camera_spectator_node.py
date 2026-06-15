import carla
import logging
import math
import rclpy
from rclpy.node import Node


class CameraSpectatorNode(Node):
    def __init__(self):
        super().__init__('camera_spectator_node')

        # Connect to CARLA
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        settings = self.world.get_settings()
        settings.synchronous_mode = False


        # Find ego vehicle spawned by Scenario Runner
        self.vehicle = self._find_ego_vehicle()

        # Get spectator
        self.spectator = self.world.get_spectator()

        while True:
            self._follow_vehicle(self.vehicle, self.spectator)
            _ = self.world.tick()

    def _find_ego_vehicle(self):
        """
        Find the ego vehicle by role_name (usually 'hero' in Scenario Runner)
        """
        actors = self.world.get_actors().filter('vehicle.*')
        for actor in actors:
            if actor.attributes.get('role_name') == 'hero':
                return actor
        return None

    def _follow_vehicle(self, vehicle, spectator):
        vehicle_transform = vehicle.get_transform()
        location = vehicle_transform.location
        rotation = vehicle_transform.rotation

        # Compute offset behind the vehicle in its local frame
        offset_distance = 7.0  # meters behind the vehicle
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


def main():
    logging.basicConfig(level=logging.INFO)
    rclpy.init()
    node = CameraSpectatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
