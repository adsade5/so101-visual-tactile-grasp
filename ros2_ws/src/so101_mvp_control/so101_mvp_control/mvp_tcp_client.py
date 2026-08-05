from __future__ import annotations

import json
import socket
from typing import Any


class MvpTcpClient:
    def __init__(self, host: str = "127.0.0.1", port: int = 8770, timeout_s: float = 2.0) -> None:
        self.host = host
        self.port = int(port)
        self.timeout_s = float(timeout_s)
        self.sock: socket.socket | None = None
        self.stream: Any | None = None

    def connect(self) -> None:
        if self.sock is not None:
            return
        sock = socket.create_connection((self.host, self.port), timeout=self.timeout_s)
        sock.settimeout(self.timeout_s)
        self.sock = sock
        self.stream = sock.makefile("rwb")

    def close(self) -> None:
        if self.stream is not None:
            self.stream.close()
        if self.sock is not None:
            self.sock.close()
        self.stream = None
        self.sock = None

    def request(self, payload: dict[str, Any]) -> dict[str, Any]:
        self.connect()
        assert self.stream is not None
        data = json.dumps(payload, separators=(",", ":")).encode("utf-8") + b"\n"
        self.stream.write(data)
        self.stream.flush()
        line = self.stream.readline()
        if not line:
            raise ConnectionError("MVP TCP server closed the connection")
        value = json.loads(line.decode("utf-8"))
        if not isinstance(value, dict):
            raise ValueError("MVP TCP server returned a non-object response")
        return value

    def get_state(self) -> dict[str, Any]:
        return self.request({"command": "get_state"})

    def move_joints_sequential(
        self,
        target_rad: list[float],
        speed_rad_s: float,
        joint_order: list[int],
        *,
        confirm: str = "MVP_MOVE",
    ) -> dict[str, Any]:
        return self.request(
            {
                "command": "move_joints_sequential",
                "target_rad": target_rad,
                "speed_rad_s": speed_rad_s,
                "joint_order": joint_order,
                "confirm": confirm,
            }
        )

    def stop(self) -> dict[str, Any]:
        return self.request({"command": "stop"})
