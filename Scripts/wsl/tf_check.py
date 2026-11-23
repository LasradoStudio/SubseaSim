"""Do the TF lookup RViz2 does for each point cloud (fixed frame <- cloud frame at the cloud stamp)
and report how many succeed and why the rest fail.

    source /opt/ros/jazzy/setup.bash && ROS_DOMAIN_ID=42 python3 tf_check.py [seconds]
"""

import collections
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener


def main() -> None:
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    rclpy.init()
    node = Node("subsea_tf_check")
    buf = Buffer(cache_time=Duration(seconds=10))
    TransformListener(buf, node)
    pending = collections.deque()
    ok, errors, ages = 0, collections.Counter(), []

    def on_cloud(msg: PointCloud2) -> None:
        pending.append((time.monotonic(), msg.header))

    node.create_subscription(PointCloud2, "/subsea/points", on_cloud, 10)
    end = time.monotonic() + seconds
    while time.monotonic() < end or pending:
        rclpy.spin_once(node, timeout_sec=0.02)
        # Give each cloud 0.5 s for its transforms to arrive, as RViz's message filter does.
        while pending and time.monotonic() - pending[0][0] > 0.5:
            _, header = pending.popleft()
            stamp = Time.from_msg(header.stamp)
            ages.append((node.get_clock().now() - stamp).nanoseconds * 1e-9)
            try:
                buf.lookup_transform("map", header.frame_id, stamp)
                ok += 1
            except Exception as exc:  # noqa: BLE001
                errors[type(exc).__name__ + ": " + str(exc).split(".")[0][:140]] += 1
        if time.monotonic() > end + 2:
            break

    total = ok + sum(errors.values())
    print(f"clouds checked: {total}, transform ok: {ok}")
    if ages:
        print(f"cloud stamp age vs ROS clock: min {min(ages):+.3f}s, max {max(ages):+.3f}s")
    for text, n in errors.most_common(5):
        print(f"  {n:4d} x {text}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
