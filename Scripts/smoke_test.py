"""Live smoke test against a running SubseaSim instance.

Connects to the sensor stream, checks that every message type arrives and is sane, optionally asks
the sim for screenshots, and exits non-zero on failure. Used after `-game` launches and in CI.

    Bridge\\.venv\\Scripts\\python.exe Scripts\\smoke_test.py --seconds 20 --screenshot pilot_view
"""

from __future__ import annotations

import argparse
import json
import sys
import time

import numpy as np

from subsea_bridge.protocol import PointCloud, StreamClient, Telemetry, Transform


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9870)
    parser.add_argument("--seconds", type=float, default=15.0)
    parser.add_argument("--connect-timeout", type=float, default=180.0, help="how long to wait for the sim to start listening")
    parser.add_argument("--screenshot", default="", help="base name; the sim saves <name>_N.png in Saved/Screenshots")
    parser.add_argument("--screenshot-every", type=float, default=8.0)
    parser.add_argument("--command", action="append", default=[], help="JSON command to send once connected (repeatable)")
    args = parser.parse_args()

    clouds: list[PointCloud] = []
    transforms: dict[str, list[Transform]] = {}
    telemetry: list[Telemetry] = []

    with StreamClient(args.host, args.port) as client:
        deadline = time.monotonic() + args.connect_timeout
        while not client.connected and time.monotonic() < deadline:
            client.poll(timeout=1.0)
        if not client.connected:
            print(f"FAIL: nothing listening on {args.host}:{args.port} after {args.connect_timeout:.0f} s")
            return 2
        print(f"connected to {args.host}:{args.port}")

        for raw in args.command:
            client.send_command(json.loads(raw))

        start = time.monotonic()
        next_shot = start + 3.0
        shots = 0
        while time.monotonic() - start < args.seconds:
            for message in client.poll(timeout=0.5):
                if isinstance(message, PointCloud):
                    clouds.append(message)
                elif isinstance(message, Transform):
                    transforms.setdefault(message.child_frame, []).append(message)
                elif isinstance(message, Telemetry):
                    telemetry.append(message)
            if args.screenshot and time.monotonic() >= next_shot:
                client.send_command({"cmd": "screenshot", "name": f"{args.screenshot}_{shots}"})
                shots += 1
                next_shot += args.screenshot_every

    failures: list[str] = []

    def check(condition: bool, text: str) -> None:
        print(("  ok    " if condition else "  FAIL  ") + text)
        if not condition:
            failures.append(text)

    print(f"\nreceived {len(clouds)} clouds, {sum(map(len, transforms.values()))} transforms "
          f"({', '.join(f'{k}: {len(v)}' for k, v in transforms.items())}), {len(telemetry)} telemetry")

    check(len(clouds) >= 3, "point clouds arrive")
    check("base_link" in transforms and len(transforms["base_link"]) >= 10, "map -> base_link transforms arrive")
    check("depth_cam" in transforms, "base_link -> depth_cam transform arrives")
    check(len(telemetry) >= 5, "telemetry arrives")

    if clouds:
        last = clouds[-1]
        xyz = last.xyz
        rng = np.linalg.norm(xyz, axis=1) if len(xyz) else np.zeros(0)
        print(f"\nlast cloud: {last.num_points} points, frame '{last.header.frame_id}', stamp {last.header.stamp:.2f} s")
        if len(xyz):
            print(f"  x (forward) {xyz[:, 0].min():6.2f} .. {xyz[:, 0].max():6.2f} m")
            print(f"  y (left)    {xyz[:, 1].min():6.2f} .. {xyz[:, 1].max():6.2f} m")
            print(f"  z (up)      {xyz[:, 2].min():6.2f} .. {xyz[:, 2].max():6.2f} m")
            print(f"  range       {rng.min():6.2f} .. {rng.max():6.2f} m, median {np.median(rng):.2f} m")
            print(f"  mean rgb    {last.rgb.mean(axis=0).round(1).tolist()}  mean intensity {last.intensity.mean():.1f}")
        check(last.num_points > 1000, "cloud has a useful number of points")
        check(bool(len(xyz)) and bool(np.isfinite(xyz).all()), "all points are finite")
        check(bool(len(xyz)) and float(xyz[:, 0].min()) > 0.0, "all points are in front of the sensor")
        check(bool(len(rng)) and float(rng.max()) <= 12.5, "no point beyond the sensor's maximum range")
        counts = [c.num_points for c in clouds]
        stamps = [c.header.stamp for c in clouds]
        if len(stamps) > 1:
            rate = (len(stamps) - 1) / max(stamps[-1] - stamps[0], 1e-6)
            print(f"  cloud rate  {rate:.1f} Hz (sim time), points per cloud {min(counts)} .. {max(counts)}")
            check(5.0 < rate < 30.0, "cloud rate is close to the configured 10 Hz")

    if transforms.get("base_link"):
        first, last_tf = transforms["base_link"][0], transforms["base_link"][-1]
        p0, p1 = np.asarray(first.position), np.asarray(last_tf.position)
        travelled = float(np.linalg.norm(p1 - p0))
        quat = np.asarray(last_tf.orientation)
        print(f"\nvehicle: start {p0.round(2).tolist()} -> end {p1.round(2).tolist()} m (moved {travelled:.2f} m)")
        check(abs(float(np.linalg.norm(quat)) - 1.0) < 1e-6, "orientation is a unit quaternion")
        check(p1[2] < -5.0, "vehicle is below the surface (map z negative)")

    if telemetry:
        data = telemetry[-1].data
        print("\nlast telemetry: " + json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in data.items()}))
        check(0.3 < data.get("altitude_m", -1.0) < 8.0, "altitude above seabed is plausible")
        check(len(data.get("thrusters", [])) == 8, "eight thruster loads reported")
        check(data.get("fps", 0.0) > 20.0, "simulation frame rate above 20 fps")

    print("\n" + ("SMOKE TEST PASSED" if not failures else f"SMOKE TEST FAILED ({len(failures)} checks)"))
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())
