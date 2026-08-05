from __future__ import annotations

import json
import socket
import time
import uuid
from typing import Any


HOST = "127.0.0.1"
PORT = 8765

PROTOCOL_VERSION = "1.0"
MAX_MESSAGE_BYTES = 65_536

HEARTBEAT_PERIOD_S = 0.2
TEST_DURATION_S = 2.0


def encode_message(message: dict[str, Any]) -> bytes:
    raw = json.dumps(
        message,
        ensure_ascii=False,
        separators=(",", ":"),
    ).encode("utf-8") + b"\n"

    if len(raw) > MAX_MESSAGE_BYTES:
        raise ValueError("message exceeds maximum size")

    return raw


class ProbeClient:
    def __init__(self) -> None:
        self.outgoing_seq = 0
        self.last_server_seq = 0
        self.buffer = b""

        self.heartbeat_count = 0
        self.joint_state_count = 0

        self.handshake_ok = False
        self.command_rejected = False
        self.estop_observed = False

    def send(
        self,
        connection: socket.socket,
        message_type: str,
        payload: dict[str, Any],
    ) -> None:
        self.outgoing_seq += 1

        message = {
            "protocol_version": PROTOCOL_VERSION,
            "type": message_type,
            "seq": self.outgoing_seq,
            "timestamp_monotonic": time.monotonic(),
            "payload": payload,
        }

        connection.sendall(encode_message(message))

    def receive(
        self,
        connection: socket.socket,
    ) -> list[dict[str, Any]]:
        try:
            chunk = connection.recv(4096)
        except socket.timeout:
            return []

        if chunk == b"":
            raise ConnectionError("server closed the connection")

        self.buffer += chunk
        messages: list[dict[str, Any]] = []

        while b"\n" in self.buffer:
            raw_line, self.buffer = self.buffer.split(b"\n", 1)

            if not raw_line:
                continue

            message = json.loads(raw_line.decode("utf-8"))
            messages.append(message)

        return messages

    def process_message(self, message: dict[str, Any]) -> None:
        seq = message["seq"]

        if seq <= self.last_server_seq:
            raise RuntimeError(
                f"stale server seq: {seq} <= {self.last_server_seq}"
            )

        self.last_server_seq = seq

        message_type = message["type"]
        payload = message["payload"]

        if message_type == "hello_ack":
            if payload["accepted_version"] != PROTOCOL_VERSION:
                raise RuntimeError("protocol version was not accepted")

            if payload["server_mode"] != "simulation":
                raise RuntimeError("server is not in simulation mode")

            if payload["hardware_connected"] is not False:
                raise RuntimeError(
                    "hardware_connected must be false in stage 0"
                )

            if payload["command_enabled"] is not False:
                raise RuntimeError(
                    "command_enabled must be false in stage 0"
                )

            self.handshake_ok = True

        elif message_type == "heartbeat":
            self.heartbeat_count += 1

            if payload.get("estop_active") is True:
                self.estop_observed = True

        elif message_type == "joint_state":
            names = payload["names"]
            positions = payload["position"]

            if len(names) != len(positions):
                raise RuntimeError(
                    "joint_state names/position length mismatch"
                )

            if not all(name.startswith("mock_") for name in names):
                raise RuntimeError(
                    "stage 0 joint names must start with mock_"
                )

            self.joint_state_count += 1

        elif message_type == "command_ack":
            if payload.get("accepted") is False:
                if (
                    payload.get("reason")
                    == "command_disabled_in_stage_0"
                ):
                    self.command_rejected = True

        elif message_type == "error":
            print(
                "SERVER ERROR:",
                payload.get("code"),
                payload.get("message"),
            )

    def run(self) -> None:
        with socket.create_connection(
            (HOST, PORT),
            timeout=2.0,
        ) as connection:
            connection.settimeout(0.05)
            connection.setsockopt(
                socket.IPPROTO_TCP,
                socket.TCP_NODELAY,
                1,
            )

            self.send(
                connection,
                "hello",
                {
                    "role": "ros2_client",
                    "session_id": str(uuid.uuid4()),
                    "supported_protocol_versions": [
                        PROTOCOL_VERSION
                    ],
                    "capabilities": [
                        "receive_joint_state",
                        "send_joint_command",
                        "send_estop",
                    ],
                },
            )

            test_start = time.monotonic()
            last_heartbeat_tx = 0.0

            command_sent = False
            estop_sent = False

            while time.monotonic() - test_start < TEST_DURATION_S:
                now = time.monotonic()

                if (
                    self.handshake_ok
                    and now - last_heartbeat_tx
                    >= HEARTBEAT_PERIOD_S
                ):
                    self.send(
                        connection,
                        "heartbeat",
                        {
                            "state": "ACTIVE",
                            "last_rx_seq": self.last_server_seq,
                            "hardware_connected": False,
                            "command_enabled": False,
                            "estop_active": False,
                        },
                    )
                    last_heartbeat_tx = now

                elapsed = now - test_start

                if self.handshake_ok and not command_sent:
                    self.send(
                        connection,
                        "joint_command",
                        {
                            "command_id": "stage0-probe-command",
                            "mode": "position",
                            "names": [
                                "mock_joint_1",
                                "mock_joint_2",
                                "mock_joint_3",
                                "mock_joint_4",
                                "mock_joint_5",
                                "mock_gripper",
                            ],
                            "position": [
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                            ],
                            "duration_s": 1.0,
                        },
                    )
                    command_sent = True

                if (
                    self.handshake_ok
                    and elapsed >= 1.0
                    and not estop_sent
                ):
                    self.send(
                        connection,
                        "estop",
                        {
                            "reason": "stage0_protocol_probe",
                        },
                    )
                    estop_sent = True

                for message in self.receive(connection):
                    self.process_message(message)

        failures: list[str] = []

        if not self.handshake_ok:
            failures.append("hello/hello_ack handshake failed")

        if self.heartbeat_count < 3:
            failures.append(
                f"expected >=3 heartbeats, got "
                f"{self.heartbeat_count}"
            )

        if self.joint_state_count < 10:
            failures.append(
                f"expected >=10 joint states, got "
                f"{self.joint_state_count}"
            )

        if not self.command_rejected:
            failures.append(
                "stage 0 joint command was not explicitly rejected"
            )

        if not self.estop_observed:
            failures.append(
                "estop_active was not observed in server heartbeat"
            )

        if failures:
            print("FAIL:")
            for failure in failures:
                print(f"  - {failure}")
            raise SystemExit(1)

        print("PASS: mock TCP hardware server")
        print(f"PASS: server heartbeats={self.heartbeat_count}")
        print(f"PASS: joint states={self.joint_state_count}")
        print("PASS: joint command rejected")
        print("PASS: estop state observed")
        print("PASS: no real hardware was used")


if __name__ == "__main__":
    ProbeClient().run()