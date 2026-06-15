#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import carla
import logging
import subprocess
import threading
import queue
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler


class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')

        # -------------------
        # Initialize GStreamer before using any pipeline
        # -------------------
        Gst.init(None)
        self.get_logger().info("✅ GStreamer initialized")

        # -------------------
        # Frame queue & thread control
        # -------------------
        import queue
        self.frame_queue = queue.Queue(maxsize=2)
        self.running = True

        # Connect to CARLA
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()

        self.sensors = []
        self.bridges = []

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Find ego vehicle spawned by Scenario Runner
        self.vehicle = self._find_ego_vehicle()
        if not self.vehicle:
            self.get_logger().error("❌ Ego vehicle not found! Make sure Scenario Runner has spawned it.")
            return
        self.get_logger().info(f"✅ Found ego vehicle: {self.vehicle.type_id}")

        # Camera configuration attached to vehicle front
        self.sensors_config = [
            {
                "type": "sensor.camera.rgb",
                "id": "front_camera",
                "spawn_point": {
                    "x": 2.0,   # 1.5 m in front of vehicle center
                    "y": 0.0,   # centered
                    "z": 1.3,   # above ground
                    "roll": 0.0,
                    "pitch": 0.0,
                    "yaw": 0.0
                },
                "attributes": {
                    "image_size_x": "1280",
                    "image_size_y": "720",
                    "fov": "50",
                    "sensor_tick": "0.05"
                }
            }
        ]

        # -------------------
        # Setup GStreamer pipeline (UDP H264)
        # -------------------
        self.pipeline = Gst.parse_launch(
            "appsrc name=src is-live=true format=time "
            "caps=video/x-raw,format=BGR,width=1280,height=720,framerate=10/1 "
            "! videoconvert "
            "! videocrop top=80 "
            "! video/x-raw,width=1280,height=640,framerate=10/1 "
            "! x264enc tune=zerolatency speed-preset=superfast bitrate=500 "
            "! rtph264pay config-interval=1 pt=96 "
            "! udpsink host=127.0.0.1 port=5000 sync=false async=false"
        )

        self.appsrc = self.pipeline.get_by_name("src")
        self.pipeline.set_state(Gst.State.PLAYING)
        self.get_logger().info("✅ GStreamer pipeline initialized and playing")

        # -------------------
        # Start worker thread
        # -------------------
        self.gst_thread = threading.Thread(target=self._gstreamer_worker, daemon=True)
        self.gst_thread.start()

        self._setup_sensors()

    def _find_ego_vehicle(self):
        """
        Find the ego vehicle by role_name (usually 'hero' in Scenario Runner)
        """
        actors = self.world.get_actors().filter('vehicle.*')
        for actor in actors:
            if actor.attributes.get('role_name') == 'hero':
                return actor
        return None

    def _setup_sensors(self):
        bp_library = self.world.get_blueprint_library()
        for sensor_cfg in self.sensors_config:
            bp = bp_library.find(sensor_cfg["type"])
            for k, v in sensor_cfg.get("attributes", {}).items():
                bp.set_attribute(str(k), str(v))

            # Transform relative to vehicle
            tf = carla.Transform(
                carla.Location(
                    x=sensor_cfg["spawn_point"]["x"],
                    y=sensor_cfg["spawn_point"]["y"],
                    z=sensor_cfg["spawn_point"]["z"]
                ),
                carla.Rotation(
                    pitch=sensor_cfg["spawn_point"]["pitch"],
                    yaw=sensor_cfg["spawn_point"]["yaw"],
                    roll=sensor_cfg["spawn_point"]["roll"]
                )
            )

            # Attach camera to ego vehicle
            sensor = self.world.spawn_actor(bp, tf, attach_to=self.vehicle)

            bridge = CvBridge()
            # topic = f'/carla/camera/{sensor_cfg["id"]}/image_raw'
            topic = f'/sensors/camera/image_raw'
            publisher = self.create_publisher(Image, topic, 10)

            def make_callback(pub, br, cfg):
                return lambda image: self._camera_callback(image, pub, br, cfg)

            sensor.listen(make_callback(publisher, bridge, sensor_cfg))

            self.sensors.append(sensor)
            self.bridges.append(bridge)
            self.get_logger().info(f"Camera '{sensor_cfg['id']}' publishing to {topic}")

    # ------------------------------------------------
    # GStreamer worker thread
    # ------------------------------------------------
    def _gstreamer_worker(self):
        while self.running:
            try:
                frame = self.frame_queue.get(timeout=0.1)
                buf = Gst.Buffer.new_wrapped(frame.tobytes())
                self.appsrc.emit("push-buffer", buf)
                # self.get_logger().info(f"Frame sent to GStreamer: {frame.shape[1]}x{frame.shape[0]}x{frame.shape[2]}")
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"❌ GStreamer push error: {e}")

    def _camera_callback(self, image, publisher, bridge, cfg):
        # Convert CARLA image to ROS2 Image
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))
        cv_image = array[:, :, :3]  # BGRA -> BGR

        msg = bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = cfg["id"]
        publisher.publish(msg)

        # Push frame to GStreamer queue (drop if busy)
        try:
            self.frame_queue.put(cv_image, block=False)
            # self.get_logger().info(f"Frame queued: {cv_image.shape[1]}x{cv_image.shape[0]}x{cv_image.shape[2]}")
        except queue.Full:
            self.get_logger().warn("Frame dropped, queue full")

        # Broadcast TF from ego vehicle to camera frame
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "ego_vehicle"
        t.child_frame_id = cfg["id"]
        t.transform.translation.x = cfg["spawn_point"]["x"]
        t.transform.translation.y = cfg["spawn_point"]["y"]
        t.transform.translation.z = cfg["spawn_point"]["z"]

        q = quaternion_from_euler(
            np.deg2rad(cfg["spawn_point"]["roll"]),
            np.deg2rad(cfg["spawn_point"]["pitch"]),
            np.deg2rad(cfg["spawn_point"]["yaw"])
        )
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    def cleanup(self):
        self.running = False

        if self.gst_thread:
            self.gst_thread.join(timeout=1.0)

        if self.gst_process:
            self.gst_process.stdin.close()
            self.gst_process.terminate()

        for sensor in self.sensors:
            if sensor.is_alive:
                sensor.stop()
                sensor.destroy()
        self.get_logger().info("✅ Cleaned up CARLA camera sensors")


def main():
    logging.basicConfig(level=logging.INFO)
    rclpy.init()
    node = CameraPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
