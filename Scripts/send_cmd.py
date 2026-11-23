"""Send one or more JSON commands to a running SubseaSim and print the next telemetry.

    python Scripts/send_cmd.py '{"cmd":"max_fps","value":60}' '{"cmd":"camera","shot":"orbit"}'
"""

import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Bridge"))
from subsea_bridge.protocol import StreamClient, Telemetry  # noqa: E402


def main() -> int:
    cmds = [json.loads(a) for a in sys.argv[1:]]
    client = StreamClient("127.0.0.1", 9870, reconnect=False)
    deadline = time.monotonic() + 10
    while not client.connected and time.monotonic() < deadline:
        client.poll(0.2)
    if not client.connected:
        print("could not connect to the simulation", file=sys.stderr)
        return 1
    for cmd in cmds:
        print("sent" if client.send_command(cmd) else "FAILED", json.dumps(cmd))
    end = time.monotonic() + 3
    while time.monotonic() < end:
        for msg in client.poll(0.2):
            if isinstance(msg, Telemetry):
                keys = ("fps", "camera", "mode")
                print({k: msg.data.get(k) for k in keys if k in msg.data})
                client.close()
                return 0
    client.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
