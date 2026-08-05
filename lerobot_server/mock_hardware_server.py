from __future__ import annotations

import argparse
import json
import logging
import math
import socket
import time
from typing import Any


PROTOCOL_VERSION = "1.0"

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8765

MAX_MESSAGE_BYTES = 65_536
HEARTBEAT_PERIOD_S = 0.2
JOINT_STATE_PERIOD_S = 0.05
PEER_TIMEOUT_S = 1.0
HANDSHAKE_TIMEOUT_S = 2.0

MOCK_JOINT_NAMES = [
    "mock_joint_1",
    "mock_joint_2",
    "mock_joint_3",
    "mock_joint_4",
    "mock_joint_5",
    "mock_gripper",
]

ALLOWED_MESSAGE_TYPES = {
    "hello",
    "hello_ack",
    "heartbeat",
    "joint_state",
    "joint_command",
    "command_ack",
    "estop",
    "clear_estop",
    "error",
}

REQUIRED_ENVELOPE_FIELDS = {
    "protocol_version",
    "type",
    "seq",
    "timestamp_monotonic",
    "payload",
}


class ProtocolError(ValueError):
    """Raised when a received TCP message violates protocol v1."""


class PeerDisconnected(ConnectionError):
    """Raised when the remote peer closes the TCP connection."""


def validate_envelope(message: Any) -> dict[str, Any]:
    if not isinstance(message, dict):
        raise ProtocolError("message must be a JSON object")

    fields = set(message)
    missing = REQUIRED_ENVELOPE_FIELDS - fields
    extra = fields - REQUIRED_ENVELOPE_FIELDS

    if missing:
        raise ProtocolError(f"missing fields: {sorted(missing)}")

    if extra:
        raise ProtocolError(f"unexpected fields: {sorted(extra)}")

    if message["protocol_version"] != PROTOCOL_VERSION:
        raise ProtocolError(
            f"unsupported protocol version: "
            f"{message['protocol_version']!r}"
        )

    message_type = message["type"]
    if message_type not in ALLOWED_MESSAGE_TYPES:
        raise ProtocolError(
            f"unsupported message type: {message_type!r}"
        )

    seq = message["seq"]
    if isinstance(seq, bool) or not isinstance(seq, int) or seq < 1:
        raise ProtocolError("seq must be a positive integer")

    timestamp = message["timestamp_monotonic"]
    if (
        isinstance(timestamp, bool)
        or not isinstance(timestamp, (int, float))
        or not math.isfinite(float(timestamp))
        or float(timestamp) < 0
    ):
        raise ProtocolError(
            "timestamp_monotonic must be a finite non-negative number"
        )

    if not isinstance(message["payload"], dict):
        raise ProtocolError("payload must be a JSON object")

    return message


def encode_message(message: dict[str, Any]) -> bytes:
    raw = json.dumps(
        message,
        ensure_ascii=False,
        separators=(",", ":"),
    ).encode("utf-8") + b"\n"

    if len(raw) > MAX_MESSAGE_BYTES:
        raise ProtocolError(
            f"outgoing message exceeds {MAX_MESSAGE_BYTES} bytes"
        )

    return raw


def receive_messages(
    connection: socket.socket,
    buffer: bytes,
) -> tuple[list[dict[str, Any]], bytes]:
    try:
        chunk = connection.recv(4096)
    except socket.timeout:
        return [], buffer

    if chunk == b"":
        raise PeerDisconnected("peer closed the connection")

    buffer += chunk

    if len(buffer) > MAX_MESSAGE_BYTES and b"\n" not in buffer:
        raise ProtocolError("incoming message exceeds maximum size")

    messages: list[dict[str, Any]] = []

    while b"\n" in buffer:
        raw_line, buffer = buffer.split(b"\n", 1)

        if not raw_line:
            continue

        if len(raw_line) > MAX_MESSAGE_BYTES:
            raise ProtocolError("incoming message exceeds maximum size")

        try:
            decoded = raw_line.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise ProtocolError(
                f"message is not valid UTF-8: {exc}"
            ) from exc

        try:
            message = json.loads(decoded)
        except json.JSONDecodeError as exc:
            raise ProtocolError(
                f"message is not valid JSON: {exc}"
            ) from exc

        messages.append(validate_envelope(message))

    return messages, buffer


class MockHardwareSession:
    def __init__(
        self,
        connection: socket.socket,
        peer_address: tuple[str, int],
        freeze_after_s: float | None,
    ) -> None:
        self.connection = connection
        self.peer_address = peer_address
        self.freeze_after_s = freeze_after_s

        self.state = "HANDSHAKING"
        self.estop_active = False

        self.active_start_time: float | None = None
        self.freeze_logged = False

        self.outgoing_seq = 0
        self.last_received_seq = 0

        now = time.monotonic()
        self.session_start_time = now
        self.last_valid_rx_time = now
        self.last_heartbeat_tx_time = 0.0
        self.last_joint_state_tx_time = 0.0

        self.receive_buffer = b""

    def send(self, message_type: str, payload: dict[str, Any]) -> None:
        self.outgoing_seq += 1

        message = {
            "protocol_version": PROTOCOL_VERSION,
            "type": message_type,
            "seq": self.outgoing_seq,
            "timestamp_monotonic": time.monotonic(),
            "payload": payload,
        }

        self.connection.sendall(encode_message(message))

    def send_error(
        self,
        code: str,
        message: str,
        *,
        fatal: bool,
        related_seq: int | None,
    ) -> None:
        self.send(
            "error",
            {
                "code": code,
                "message": message,
                "fatal": fatal,
                "related_seq": related_seq,
            },
        )

    def process_sequence_number(
        self,
        message: dict[str, Any],
    ) -> bool:
        seq = message["seq"]

        if seq <= self.last_received_seq:
            self.send_error(
                "STALE_SEQUENCE",
                (
                    f"received seq={seq}, but last accepted "
                    f"seq={self.last_received_seq}"
                ),
                fatal=False,
                related_seq=seq,
            )
            return False

        self.last_received_seq = seq
        return True

    def process_handshake(
        self,
        message: dict[str, Any],
    ) -> None:
        if message["type"] != "hello":
            self.send_error(
                "HANDSHAKE_REQUIRED",
                "first client message must be hello",
                fatal=True,
                related_seq=message["seq"],
            )
            raise ProtocolError("first message was not hello")

        payload = message["payload"]

        required_fields = {
            "role",
            "session_id",
            "supported_protocol_versions",
            "capabilities",
        }

        missing = required_fields - payload.keys()
        if missing:
            self.send_error(
                "INVALID_HELLO",
                f"hello is missing fields: {sorted(missing)}",
                fatal=True,
                related_seq=message["seq"],
            )
            raise ProtocolError("invalid hello payload")

        if payload["role"] != "ros2_client":
            self.send_error(
                "INVALID_CLIENT_ROLE",
                "hello role must be ros2_client",
                fatal=True,
                related_seq=message["seq"],
            )
            raise ProtocolError("invalid client role")

        supported_versions = payload["supported_protocol_versions"]
        if (
            not isinstance(supported_versions, list)
            or PROTOCOL_VERSION not in supported_versions
        ):
            self.send_error(
                "INCOMPATIBLE_PROTOCOL",
                "client does not support protocol version 1.0",
                fatal=True,
                related_seq=message["seq"],
            )
            raise ProtocolError("no compatible protocol version")

        self.send(
            "hello_ack",
            {
                "accepted_version": PROTOCOL_VERSION,
                "server_mode": "simulation",
                "hardware_connected": False,
                "command_enabled": False,
                "joint_names": MOCK_JOINT_NAMES,
            },
        )

        self.state = "ACTIVE"
        self.active_start_time = time.monotonic()

        logging.info(
            "Handshake completed: session_id=%s",
            payload["session_id"],
        )

    def process_active_message(
        self,
        message: dict[str, Any],
    ) -> None:
        message_type = message["type"]
        payload = message["payload"]

        if message_type == "heartbeat":
            return

        if message_type == "joint_command":
            command_id = payload.get("command_id")

            if not isinstance(command_id, str) or not command_id:
                command_id = "unknown"

            self.send(
                "command_ack",
                {
                    "command_id": command_id,
                    "accepted": False,
                    "reason": "command_disabled_in_stage_0",
                    "server_state": "SIMULATION",
                },
            )

            logging.warning(
                "Rejected joint command in stage 0: %s",
                command_id,
            )
            return

        if message_type == "estop":
            reason = payload.get("reason", "unspecified")
            self.estop_active = True

            logging.warning(
                "Emergency stop latched by protocol message: %s",
                reason,
            )
            return

        if message_type == "clear_estop":
            self.send_error(
                "CLEAR_ESTOP_DISABLED",
                "clear_estop is disabled during stage 0",
                fatal=False,
                related_seq=message["seq"],
            )
            return

        self.send_error(
            "UNEXPECTED_CLIENT_MESSAGE",
            f"client may not send message type {message_type!r}",
            fatal=False,
            related_seq=message["seq"],
        )

    def output_is_frozen(self, now: float) -> bool:
        if self.freeze_after_s is None:
            return False

        if self.active_start_time is None:
            return False

        if now - self.active_start_time < self.freeze_after_s:
            return False

        if not self.freeze_logged:
            logging.warning(
                "Stage 0 test freeze started after %.2f seconds. "
                "TCP remains connected, but the server will stop "
                "sending heartbeat and joint_state messages.",
                self.freeze_after_s,
            )
            self.freeze_logged = True

        return True

    def send_heartbeat_if_due(self, now: float) -> None:
        if now - self.last_heartbeat_tx_time < HEARTBEAT_PERIOD_S:
            return

        self.send(
            "heartbeat",
            {
                "state": self.state,
                "last_rx_seq": self.last_received_seq,
                "hardware_connected": False,
                "command_enabled": False,
                "estop_active": self.estop_active,
            },
        )

        self.last_heartbeat_tx_time = now

    def send_joint_state_if_due(self, now: float) -> None:
        if now - self.last_joint_state_tx_time < JOINT_STATE_PERIOD_S:
            return

        elapsed = now - self.session_start_time

        positions = [
            0.10 * math.sin(0.50 * elapsed + phase)
            for phase in (0.0, 0.5, 1.0, 1.5, 2.0, 2.5)
        ]

        velocities = [
            0.05 * math.cos(0.50 * elapsed + phase)
            for phase in (0.0, 0.5, 1.0, 1.5, 2.0, 2.5)
        ]

        self.send(
            "joint_state",
            {
                "names": MOCK_JOINT_NAMES,
                "position": positions,
                "velocity": velocities,
            },
        )

        self.last_joint_state_tx_time = now

    def run(self) -> None:
        self.connection.settimeout(0.02)
        self.connection.setsockopt(
            socket.IPPROTO_TCP,
            socket.TCP_NODELAY,
            1,
        )

        logging.info(
            "Client connected from %s:%s",
            self.peer_address[0],
            self.peer_address[1],
        )

        while True:
            now = time.monotonic()

            if self.state == "HANDSHAKING":
                if now - self.session_start_time > HANDSHAKE_TIMEOUT_S:
                    raise TimeoutError("client handshake timed out")

            elif self.state == "ACTIVE":
                if now - self.last_valid_rx_time > PEER_TIMEOUT_S:
                    raise TimeoutError(
                        "client heartbeat/message timeout"
                    )

            messages, self.receive_buffer = receive_messages(
                self.connection,
                self.receive_buffer,
            )

            for message in messages:
                if not self.process_sequence_number(message):
                    continue

                self.last_valid_rx_time = time.monotonic()

                if self.state == "HANDSHAKING":
                    self.process_handshake(message)
                else:
                    self.process_active_message(message)

            now = time.monotonic()

            if self.state == "ACTIVE":
                if not self.output_is_frozen(now):
                    self.send_heartbeat_if_due(now)
                    self.send_joint_state_if_due(now)


def run_server(
    host: str,
    port: int,
    freeze_after_s: float | None,
) -> None:
    if host != "127.0.0.1":
        raise ValueError(
            "stage 0 server must bind only to 127.0.0.1"
        )

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1,
        )

        server.bind((host, port))
        server.listen(1)
        server.settimeout(0.5)

        logging.info(
            "Mock hardware server listening on %s:%d",
            host,
            port,
        )
        logging.info(
            "Simulation only: no LeRobot imports, no serial ports, "
            "no motor commands"
        )

        while True:
            try:
                connection, peer_address = server.accept()
            except socket.timeout:
                continue

            with connection:
                session = MockHardwareSession(
                    connection,
                    peer_address,
                    freeze_after_s,
                )

                try:
                    session.run()
                except (
                    PeerDisconnected,
                    ProtocolError,
                    TimeoutError,
                    ConnectionError,
                    OSError,
                ) as exc:
                    logging.warning(
                        "Client session ended: %s",
                        exc,
                    )
                except Exception:
                    logging.exception(
                        "Unexpected client session failure"
                    )

            logging.info("Waiting for a new client")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Stage 0 SO-101 simulated hardware TCP server. "
            "Does not import LeRobot or access hardware."
        )
    )

    parser.add_argument(
        "--host",
        default=DEFAULT_HOST,
    )

    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_PORT,
    )

    parser.add_argument(
        "--freeze-after-s",
        type=float,
        default=None,
        help=(
            "Stage 0 test mode: after handshake, stop sending "
            "all heartbeat and joint_state messages while keeping "
            "the TCP connection open."
        ),
    )

    return parser.parse_args()


def main() -> int:
    args = parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format=(
            "%(asctime)s | %(levelname)s | %(message)s"
        ),
    )

    try:
        if (
            args.freeze_after_s is not None
            and args.freeze_after_s <= 0
        ):
            raise ValueError(
                "--freeze-after-s must be greater than zero"
            )

        run_server(
            args.host,
            args.port,
            args.freeze_after_s,
        )
    except KeyboardInterrupt:
        logging.info("Server stopped by user")
        return 0
    except Exception:
        logging.exception("Server failed")
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())