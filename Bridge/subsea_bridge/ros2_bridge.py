"""ROS 2 (Jazzy) bridge: republish the SubseaSim stream for RViz2.

Publishes

* ``points``      sensor_msgs/PointCloud2  (x, y, z float32 + packed rgb + intensity)
* ``/tf``         map -> base_link (transforms parented to ``map_frame``)
* ``/tf_static``  base_link -> depth_cam, sonar (rigid sensor mounts, re-latched only on change)
* ``odom``        nav_msgs/Odometry for transforms whose parent is the ``map_frame`` parameter
* ``telemetry``   std_msgs/String, the raw telemetry JSON
* ``sonar/raw``   sensor_msgs/Image mono8, the polar sonar image (rows = range bins, cols = beams)
* ``sonar/image`` sensor_msgs/Image rgb8, the scan-converted fan the pilot sees

Parameters (also settable through the SUBSEA_HOST / SUBSEA_PORT environment variables):
``host`` (127.0.0.1), ``port`` (9870), ``map_frame`` (map), ``use_sim_stamps`` (false),
``sonar_fan_width`` (640 px, 0 disables the fan image).

With ``use_sim_stamps:=false`` (default) sim stamps are shifted by a constant offset so
that they line up with the ROS clock; every message of a tick keeps the same stamp, so TF
lookups at the cloud stamp resolve exactly. With ``true`` the raw simulation time is used.

    ros2 run / python3 -m subsea_bridge.ros2_bridge --ros-args -p host:=192.168.1.20

This is the only module in the package that imports ROS. The PointCloud2 packing it relies
on lives in ``subsea_bridge.pointcloud2`` and is unit tested; the node itself is exercised
live with ROS 2 Jazzy in WSL (``Scripts/wsl/run_ros2.ps1``).
"""

from __future__ import annotations

import array
import json
import os
import threading

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from .pointcloud2 import pack_pointcloud2, stamp_to_sec_nanosec
from .protocol import DEFAULT_PORT, Message, PointCloud, SonarImage, StreamClient, Telemetry, Transform
from .sonar import render_fan


class SubseaBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("subsea_bridge")
        default_host = os.environ.get("SUBSEA_HOST", "127.0.0.1")
        default_port = int(os.environ.get("SUBSEA_PORT", str(DEFAULT_PORT)))
        self._host = str(self.declare_parameter("host", default_host).value)
        self._port = int(self.declare_parameter("port", default_port).value)
        self._map_frame = str(self.declare_parameter("map_frame", "map").value)
        self._use_sim_stamps = bool(self.declare_parameter("use_sim_stamps", False).value)
        self._fan_width = int(self.declare_parameter("sonar_fan_width", 640).value)

        # Reliable, keep-last: matches RViz2's default subscription QoS.
        self._cloud_pub = self.create_publisher(PointCloud2, "points", 5)
        self._odom_pub = self.create_publisher(Odometry, "odom", 20)
        self._telemetry_pub = self.create_publisher(String, "telemetry", 20)
        self._sonar_raw_pub = self.create_publisher(Image, "sonar/raw", 5)
        self._sonar_fan_pub = self.create_publisher(Image, "sonar/image", 5)
        self._tf = TransformBroadcaster(self)
        # Sensor mounts (any transform not parented to the map frame) are rigid, so they go out on
        # /tf_static: lookups at a sensor stamp then only need the vehicle pose, which is dense.
        self._static_mounts = bool(self.declare_parameter("static_mounts", True).value)
        self._static_tf = StaticTransformBroadcaster(self)
        self._mounts: dict[str, TransformStamped] = {}

        self._offset: float | None = None
        self._last_stamp = float("-inf")
        self._client = StreamClient(self._host, self._port, reconnect=True)
        self._was_connected = False
        self._thread = threading.Thread(target=self._pump, name="subsea-stream", daemon=True)
        self._thread.start()
        self.get_logger().info(f"connecting to SubseaSim at {self._host}:{self._port}")

    # -- time --------------------------------------------------------------------

    def _stamp(self, sim_stamp: float) -> TimeMsg:
        if self._use_sim_stamps:
            t = sim_stamp
        else:
            # (Re)anchor on the first message and whenever sim time jumps backwards
            # (simulator restarted).
            if self._offset is None or sim_stamp < self._last_stamp - 1.0:
                self._offset = self.get_clock().now().nanoseconds * 1e-9 - sim_stamp
            t = sim_stamp + self._offset
        self._last_stamp = sim_stamp
        sec, nanosec = stamp_to_sec_nanosec(t)
        return TimeMsg(sec=sec, nanosec=nanosec)

    # -- stream thread -------------------------------------------------------------

    def _pump(self) -> None:
        client = self._client
        while not client.closed and rclpy.ok():
            msgs = client.poll(0.5)
            if client.connected != self._was_connected:
                self._was_connected = client.connected
                if client.connected:
                    self.get_logger().info("stream connected")
                else:
                    self.get_logger().warning(f"stream lost ({client.last_error}); reconnecting")
            for msg in msgs:
                try:
                    self._handle(msg)
                except Exception as exc:  # noqa: BLE001 - keep the bridge alive, context may be shutting down
                    if rclpy.ok():
                        self.get_logger().error(f"failed to publish: {exc!r}")
                    return

    def _handle(self, msg: Message) -> None:
        if isinstance(msg, PointCloud):
            self._publish_cloud(msg)
        elif isinstance(msg, Transform):
            self._publish_transform(msg)
        elif isinstance(msg, SonarImage):
            self._publish_sonar(msg)
        elif isinstance(msg, Telemetry):
            self._telemetry_pub.publish(String(data=json.dumps(msg.data, separators=(",", ":"))))

    @staticmethod
    def _image(stamp: TimeMsg, frame_id: str, pixels, encoding: str, channels: int) -> Image:
        out = Image()
        out.header.stamp = stamp
        out.header.frame_id = frame_id
        out.height, out.width = int(pixels.shape[0]), int(pixels.shape[1])
        out.encoding = encoding
        out.is_bigendian = 0
        out.step = out.width * channels
        out.data = array.array("B", pixels.tobytes())
        return out

    def _publish_sonar(self, msg: SonarImage) -> None:
        stamp = self._stamp(msg.header.stamp)
        frame = msg.header.frame_id
        self._sonar_raw_pub.publish(self._image(stamp, frame, msg.intensity, "mono8", 1))
        if self._fan_width > 0 and self._sonar_fan_pub.get_subscription_count() > 0:
            fan = render_fan(msg, width=self._fan_width)
            self._sonar_fan_pub.publish(self._image(stamp, frame, fan, "rgb8", 3))

    def _publish_cloud(self, msg: PointCloud) -> None:
        packed = pack_pointcloud2(msg.points, msg.flags)
        out = PointCloud2()
        out.header.stamp = self._stamp(msg.header.stamp)
        out.header.frame_id = msg.header.frame_id
        out.height = packed.height
        out.width = packed.width
        out.fields = [
            PointField(name=f.name, offset=f.offset, datatype=f.datatype, count=f.count)
            for f in packed.fields
        ]
        out.is_bigendian = packed.is_bigendian
        out.point_step = packed.point_step
        out.row_step = packed.row_step
        # array('B') takes rclpy's fast path; assigning bytes would be validated per element.
        out.data = array.array("B", packed.data)
        out.is_dense = packed.is_dense
        self._cloud_pub.publish(out)

    def _publish_transform(self, msg: Transform) -> None:
        stamp = self._stamp(msg.header.stamp)
        px, py, pz = (float(v) for v in msg.position)
        qx, qy, qz, qw = (float(v) for v in msg.orientation)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = msg.parent_frame
        tf.child_frame_id = msg.child_frame
        tf.transform.translation.x = px
        tf.transform.translation.y = py
        tf.transform.translation.z = pz
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        if msg.parent_frame != self._map_frame:
            if self._static_mounts:
                self._publish_mount(tf)
            else:
                self._tf.sendTransform(tf)
            return
        self._tf.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = msg.parent_frame
        odom.child_frame_id = msg.child_frame
        odom.pose.pose.position.x = px
        odom.pose.pose.position.y = py
        odom.pose.pose.position.z = pz
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        # The protocol's twist is expressed in the child frame, as Odometry requires.
        odom.twist.twist.linear.x = float(msg.linear_vel[0])
        odom.twist.twist.linear.y = float(msg.linear_vel[1])
        odom.twist.twist.linear.z = float(msg.linear_vel[2])
        odom.twist.twist.angular.x = float(msg.angular_vel[0])
        odom.twist.twist.angular.y = float(msg.angular_vel[1])
        odom.twist.twist.angular.z = float(msg.angular_vel[2])
        self._odom_pub.publish(odom)

    def _publish_mount(self, tf: TransformStamped) -> None:
        """Latch a rigid sensor mount on /tf_static, re-sending only when it changes."""
        old = self._mounts.get(tf.child_frame_id)
        if old is not None and old.header.frame_id == tf.header.frame_id:
            a, b = old.transform, tf.transform
            delta = max(
                abs(a.translation.x - b.translation.x), abs(a.translation.y - b.translation.y),
                abs(a.translation.z - b.translation.z), abs(a.rotation.x - b.rotation.x),
                abs(a.rotation.y - b.rotation.y), abs(a.rotation.z - b.rotation.z),
                abs(a.rotation.w - b.rotation.w),
            )
            if delta < 1e-6:
                return
        self._mounts[tf.child_frame_id] = tf
        # /tf_static is latched per publisher, so every mount goes out together each time.
        self._static_tf.sendTransform(list(self._mounts.values()))

    def shutdown(self) -> None:
        self._client.close()
        self._thread.join(timeout=2.0)


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    node = SubseaBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
