"""Drives a running SubseaSim through a scripted shot list over the stream's command channel.

A shot list is JSON: a list of steps, each with optional "commands" (sent in order), "wait"
(seconds of wall time to let things settle) and "until_frames" (wait for a recording to finish by
counting files). Used to capture stills and frame sequences for documentation without touching
the desktop.

    Bridge\\.venv\\Scripts\\python.exe Scripts\\direct.py Scripts\\shots\\preview.json
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from subsea_bridge.protocol import StreamClient, Telemetry

PROJECT = Path(__file__).resolve().parent.parent


def send_step(client: StreamClient, step: dict) -> None:
    """Sends a step's commands. If the step starts a recording, confirms it began and resends
    otherwise: a command written just as the server drops a slow client is silently lost."""
    record = next((c for c in step.get("commands", []) if c.get("cmd") == "record"), None)
    for attempt in range(5):
        deadline = time.monotonic() + 30.0
        while not client.connected and time.monotonic() < deadline:
            client.poll(timeout=0.2)
        for command in step.get("commands", []):
            client.send_command(command)
        if record is None:
            return
        folder = PROJECT / "Saved" / "Recordings" / record["name"]
        check = time.monotonic() + 6.0
        while time.monotonic() < check:
            client.poll(timeout=0.1)
            if folder.is_dir():
                return
        print(f"  recording {record['name']} did not start, resending (attempt {attempt + 2})", flush=True)
    raise RuntimeError(f"could not start recording {record['name']}")


def wait_for_frames(client: StreamClient, name: str, frames: int, timeout: float) -> bool:
    folder = PROJECT / "Saved" / "Recordings" / name
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        client.poll(timeout=0.05)  # drain continuously so the server never drops us as slow
        if len(list(folder.glob("frame_*.png"))) >= frames:
            time.sleep(1.0)  # let the last PNG finish writing
            return True
    return False


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("shotlist", type=Path)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9870)
    parser.add_argument("--quit", action="store_true")
    args = parser.parse_args()

    steps = json.loads(args.shotlist.read_text())
    with StreamClient(args.host, args.port) as client:
        deadline = time.monotonic() + 240.0
        while not client.connected and time.monotonic() < deadline:
            client.poll(timeout=1.0)
        if not client.connected:
            print("FAIL: sim not reachable")
            return 2

        for index, step in enumerate(steps):
            label = step.get("label", f"step {index}")
            print(f"[{time.strftime('%H:%M:%S')}] {label}", flush=True)
            send_step(client, step)
            if "until_telemetry" in step:
                key, target = step["until_telemetry"]
                limit = time.monotonic() + step.get("timeout", 600.0)
                reached = False
                while not reached and time.monotonic() < limit:
                    for message in client.poll(timeout=0.5):
                        if isinstance(message, Telemetry) and float(message.data.get(key, -1e9)) >= float(target):
                            reached = True
                if not reached:
                    print(f"  telemetry {key} never reached {target}")
                    return 1
            if "until_frames" in step:
                name, frames = step["until_frames"]
                if not wait_for_frames(client, name, frames, step.get("timeout", 1800.0)):
                    print(f"  recording {name} did not finish")
                    return 1
            end = time.monotonic() + float(step.get("wait", 0.0))
            while time.monotonic() < end:
                for message in client.poll(timeout=0.25):
                    if isinstance(message, Telemetry) and step.get("print_telemetry"):
                        print("  " + json.dumps({k: message.data[k] for k in step["print_telemetry"] if k in message.data}))

        if args.quit:
            client.send_command({"cmd": "quit"})
            time.sleep(1.0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
