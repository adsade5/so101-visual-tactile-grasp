from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from shared_protocol.mvp_tcp_client import MvpTcpClient


def main() -> int:
    parser = argparse.ArgumentParser(description="Legacy MVP TCP probe. Do not use while ROS2 bridge is connected.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8770)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--get-state", action="store_true")
    args = parser.parse_args()

    if not args.get_state:
        parser.error("Only --get-state is supported by the legacy single-client probe")

    print("TCP_PROBE_START")
    print(f"host={args.host}")
    print(f"port={args.port}")
    client = MvpTcpClient(
        args.host,
        args.port,
        connect_timeout_s=args.timeout,
        state_request_timeout_s=args.timeout,
    )
    try:
        response = client.get_state()
        print("connected=true")
        print("response=" + json.dumps(response, separators=(",", ":")))
        print("TCP_PROBE_PASS")
        return 0
    except Exception as exc:
        print("TCP_PROBE_FAIL")
        print(f"error_type={type(exc).__name__}")
        print(f"error_message={exc}")
        return 1
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
