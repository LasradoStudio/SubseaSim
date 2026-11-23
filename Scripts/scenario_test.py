"""Scripted scenario against a running SubseaSim: fault injection and water-quality sweep.

Phases, each measured from live telemetry:
  1. baseline       pipeline following, all thrusters healthy
  2. thruster fault horizontal thruster 1 fails; the allocator re-solves around it
  3. recovered      thruster restored
  4. turbid water   turbidity raised; the depth camera should lose returns, the sonar should not
  5. 60 fps / 30 fps frame rate capped; tracking must not depend on frame rate

Exits non-zero if tracking degrades beyond tolerance or the sensor does not respond to turbidity.

    Bridge\\.venv\\Scripts\\python.exe Scripts\\scenario_test.py --quit
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass, field

import numpy as np

from subsea_bridge.protocol import SonarImage, StreamClient, Telemetry


@dataclass
class Phase:
    name: str
    cross_track: list[float] = field(default_factory=list)
    speed: list[float] = field(default_factory=list)
    points: list[float] = field(default_factory=list)
    peak_thrust: list[float] = field(default_factory=list)
    fps: list[float] = field(default_factory=list)
    sonar_mean: list[float] = field(default_factory=list)

    def row(self) -> str:
        xt = np.asarray(self.cross_track)
        return (f"{self.name:<16}{np.sqrt(np.mean(xt ** 2)) * 100:>11.1f}{np.abs(xt).max() * 100:>11.1f}"
                f"{np.mean(self.speed):>9.3f}{np.mean(self.points):>10.0f}{np.max(self.peak_thrust) * 100:>9.0f}"
                f"{np.mean(self.sonar_mean) if self.sonar_mean else float('nan'):>9.1f}{np.median(self.fps):>8.0f}")


def run_phase(client: StreamClient, name: str, seconds: float, settle: float) -> Phase:
    phase = Phase(name)
    start = time.monotonic()
    while time.monotonic() - start < seconds:
        for message in client.poll(timeout=0.5):
            # Ignore the transient right after a change; report the settled behaviour.
            if isinstance(message, SonarImage) and time.monotonic() - start >= settle:
                phase.sonar_mean.append(float(np.mean(message.intensity)))
            if isinstance(message, Telemetry) and time.monotonic() - start >= settle:
                data = message.data
                phase.cross_track.append(float(data["cross_track_m"]))
                phase.speed.append(float(data["speed_mps"]))
                phase.points.append(float(data["points"]))
                phase.peak_thrust.append(max(abs(v) for v in data["thrusters"]))
                phase.fps.append(float(data["fps"]))
    return phase


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9870)
    parser.add_argument("--phase-seconds", type=float, default=18.0)
    parser.add_argument("--quit", action="store_true", help="ask the sim to exit when done")
    args = parser.parse_args()

    with StreamClient(args.host, args.port) as client:
        deadline = time.monotonic() + 180.0
        while not client.connected and time.monotonic() < deadline:
            client.poll(timeout=1.0)
        if not client.connected:
            print("FAIL: sim not reachable")
            return 2

        client.send_command({"cmd": "set_mode", "mode": "pipeline_follow"})
        baseline = run_phase(client, "baseline", args.phase_seconds, settle=6.0)

        client.send_command({"cmd": "fail_thruster", "index": 0, "failed": True})
        fault = run_phase(client, "thruster fault", args.phase_seconds, settle=0.0)

        client.send_command({"cmd": "fail_thruster", "index": 0, "failed": False})
        recovered = run_phase(client, "recovered", args.phase_seconds * 0.6, settle=3.0)

        client.send_command({"cmd": "set_turbidity", "value": 0.6})
        turbid = run_phase(client, "turbidity 0.6", args.phase_seconds * 0.6, settle=3.0)
        client.send_command({"cmd": "set_turbidity", "value": 0.15})

        client.send_command({"cmd": "max_fps", "value": 60})
        fps60 = run_phase(client, "capped 60 fps", args.phase_seconds * 0.7, settle=3.0)
        client.send_command({"cmd": "max_fps", "value": 30})
        fps30 = run_phase(client, "capped 30 fps", args.phase_seconds * 0.7, settle=3.0)
        client.send_command({"cmd": "max_fps", "value": 0})
        if args.quit:
            client.poll(timeout=0.5)
            client.send_command({"cmd": "quit"})
            time.sleep(1.0)

    phases = [baseline, fault, recovered, turbid, fps60, fps30]
    print(f"\n{'phase':<16}{'xt rms cm':>11}{'xt max cm':>11}{'speed':>9}{'points':>10}{'peak %':>9}{'sonar':>9}{'fps':>8}")
    for phase in phases:
        print(phase.row())

    failures = []
    fault_max = float(np.abs(fault.cross_track).max())
    if fault_max > 0.30:
        failures.append(f"cross-track reached {fault_max:.2f} m with a failed thruster (limit 0.30 m)")
    if abs(float(np.mean(fault.speed[len(fault.speed) // 2:])) - float(np.mean(baseline.speed))) > 0.08:
        failures.append("survey speed not held with a failed thruster")
    if np.mean(turbid.points) > 0.8 * np.mean(baseline.points):
        failures.append("depth camera did not lose returns in turbid water")

    if turbid.sonar_mean and baseline.sonar_mean and abs(np.mean(turbid.sonar_mean) - np.mean(baseline.sonar_mean)) > 0.15 * np.mean(baseline.sonar_mean):
        failures.append("sonar image changed with turbidity (it is acoustic and should not)")
    for capped in (fps60, fps30):
        if float(np.abs(capped.cross_track).max()) > 0.10:
            failures.append(f"{capped.name}: cross-track {np.abs(capped.cross_track).max():.2f} m (limit 0.10 m)")
        if abs(float(np.mean(capped.speed)) - float(np.mean(baseline.speed))) > 0.03:
            failures.append(f"{capped.name}: survey speed differs from uncapped by more than 0.03 m/s")
    for failure in failures:
        print("FAIL: " + failure)
    print("\n" + ("SCENARIO TEST PASSED" if not failures else "SCENARIO TEST FAILED"))
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())
