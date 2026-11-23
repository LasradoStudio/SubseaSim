"""Measure what the SubseaSim ROS 2 bridge publishes: message rate and size per topic.

    source /opt/ros/jazzy/setup.bash && ROS_DOMAIN_ID=42 python3 ros2_rates.py [seconds]
"""

import json
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String

TOPICS = {
    "/subsea/points": PointCloud2,
    "/subsea/odom": Odometry,
    "/subsea/sonar/raw": Image,
    "/subsea/telemetry": String,
}


def main() -> None:
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    rclpy.init()
    node = Node("subsea_rates")
    stats = {t: {"n": 0, "bytes": 0, "first": None, "last": None, "extra": None} for t in TOPICS}

    def make_cb(topic):
        def cb(msg):
            s = stats[topic]
            now = time.monotonic()
            s["first"] = s["first"] or now
            s["last"] = now
            s["n"] += 1
            if isinstance(msg, PointCloud2):
                s["bytes"] += len(msg.data)
                s["extra"] = f"{msg.width * msg.height} points"
            elif isinstance(msg, Image):
                s["bytes"] += len(msg.data)
                s["extra"] = f"{msg.width}x{msg.height} {msg.encoding}"
            elif isinstance(msg, String):
                s["extra"] = ", ".join(sorted(json.loads(msg.data))[:6])
        return cb

    for topic, kind in TOPICS.items():
        node.create_subscription(kind, topic, make_cb(topic), 10)

    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    for topic, s in stats.items():
        if s["n"] < 2:
            print(f"{topic}: {s['n']} messages")
            continue
        span = s["last"] - s["first"]
        rate = (s["n"] - 1) / span if span > 0 else 0.0
        mbit = s["bytes"] * 8 / span / 1e6 if span > 0 else 0.0
        print(f"{topic}: {rate:.1f} Hz, {s['n']} msgs, {mbit:.1f} Mbit/s, {s['extra'] or ''}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
