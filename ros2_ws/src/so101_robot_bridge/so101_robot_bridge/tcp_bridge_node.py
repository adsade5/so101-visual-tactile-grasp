from __future__ import annotations

import json
import math
import queue
import socket
import threading
import time
import uuid
from typing import Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import String


PROTOCOL_VERSION = "1.0"

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
    """Raised when a TCP message violates protocol v1."""


def require_finite_number(value: Any, field_name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolError(f"{field_name} must be numeric")

    numeric_value = float(value)

    if not math.isfinite(numeric_value):
        raise ProtocolError(f"{field_name} must be finite")

    return numeric_value


def validate_envelope(message: Any) -> dict[str, Any]:
    if not isinstance(message, dict):
        raise ProtocolError("message must be a JSON object")

    fields = set(message)
    missing = REQUIRED_ENVELOPE_FIELDS - fields
    extra = fields - REQUIRED_ENVELOPE_FIELDS

    if missing:
        raise ProtocolError(f"missing fields: {sorted(missing)}")

    if extra:
        raise ProtocolError(
            f"unexpected fields: {sorted(extra)}"
        )

    if message["protocol_version"] != PROTOCOL_VERSION:
        raise ProtocolError(
            "unsupported protocol version: "
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

    timestamp = require_finite_number(
        message["timestamp_monotonic"],
        "timestamp_monotonic",
    )

    if timestamp < 0:
        raise ProtocolError(
            "timestamp_monotonic must be non-negative"
        )

    if not isinstance(message["payload"], dict):
        raise ProtocolError("payload must be a JSON object")

    return message


def validate_joint_state_payload(
    payload: dict[str, Any],
) -> dict[str, Any]:
    names = payload.get("names")
    positions = payload.get("position")
    velocities = payload.get("velocity")
    efforts = payload.get("effort")

    if not isinstance(names, list) or not names:
        raise ProtocolError(
            "joint_state names must be a non-empty list"
        )

    if not all(
        isinstance(name, str) and name
        for name in names
    ):
        raise ProtocolError(
            "every joint name must be a non-empty string"
        )

    if not isinstance(positions, list):
        raise ProtocolError(
            "joint_state position must be a list"
        )

    if len(names) != len(positions):
        raise ProtocolError(
            "joint_state names and position length mismatch"
        )

    validated_positions = [
        require_finite_number(value, f"position[{index}]")
        for index, value in enumerate(positions)
    ]

    validated_velocities: list[float] = []

    if velocities not in (None, []):
        if not isinstance(velocities, list):
            raise ProtocolError(
                "joint_state velocity must be a list"
            )

        if len(velocities) != len(names):
            raise ProtocolError(
                "joint_state velocity length mismatch"
            )

        validated_velocities = [
            require_finite_number(
                value,
                f"velocity[{index}]",
            )
            for index, value in enumerate(velocities)
        ]

    validated_efforts: list[float] = []

    if efforts not in (None, []):
        if not isinstance(efforts, list):
            raise ProtocolError(
                "joint_state effort must be a list"
            )

        if len(efforts) != len(names):
            raise ProtocolError(
                "joint_state effort length mismatch"
            )

        validated_efforts = [
            require_finite_number(
                value,
                f"effort[{index}]",
            )
            for index, value in enumerate(efforts)
        ]

    return {
        "names": list(names),
        "position": validated_positions,
        "velocity": validated_velocities,
        "effort": validated_efforts,
    }


class So101RobotBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("so101_robot_bridge")

        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8765)
        self.declare_parameter("heartbeat_period_s", 0.2)
        self.declare_parameter("peer_timeout_s", 1.0)
        self.declare_parameter("handshake_timeout_s", 2.0)
        self.declare_parameter("reconnect_delay_s", 1.0)
        self.declare_parameter("max_message_bytes", 65_536)
        self.declare_parameter("joint_state_frame_id", "")

        self.host = str(
            self.get_parameter("host").value
        )
        self.port = int(
            self.get_parameter("port").value
        )
        self.heartbeat_period_s = float(
            self.get_parameter("heartbeat_period_s").value
        )
        self.peer_timeout_s = float(
            self.get_parameter("peer_timeout_s").value
        )
        self.handshake_timeout_s = float(
            self.get_parameter(
                "handshake_timeout_s"
            ).value
        )
        self.reconnect_delay_s = float(
            self.get_parameter("reconnect_delay_s").value
        )
        self.max_message_bytes = int(
            self.get_parameter("max_message_bytes").value
        )
        self.joint_state_frame_id = str(
            self.get_parameter(
                "joint_state_frame_id"
            ).value
        )

        if self.host != "127.0.0.1":
            raise ValueError(
                "stage 0 bridge may connect only to 127.0.0.1"
            )

        if not 1 <= self.port <= 65_535:
            raise ValueError("port must be between 1 and 65535")

        if self.heartbeat_period_s <= 0:
            raise ValueError(
                "heartbeat_period_s must be positive"
            )

        if self.peer_timeout_s <= self.heartbeat_period_s:
            raise ValueError(
                "peer_timeout_s must be greater than "
                "heartbeat_period_s"
            )

        self.joint_state_publisher = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )

        self.connection_state_publisher = (
            self.create_publisher(
                String,
                "/robot_bridge/connection_state",
                10,
            )
        )

        self.hardware_connected_publisher = (
            self.create_publisher(
                Bool,
                "/robot_bridge/hardware_connected",
                10,
            )
        )

        self.command_enabled_publisher = (
            self.create_publisher(
                Bool,
                "/robot_bridge/command_enabled",
                10,
            )
        )

        self.estop_active_publisher = (
            self.create_publisher(
                Bool,
                "/robot_bridge/estop_active",
                10,
            )
        )

        self.connection_state = "DISCONNECTED"
        self.hardware_connected = False
        self.command_enabled = False
        self.estop_active = False

        self.event_queue: queue.Queue[
            tuple[str, Any]
        ] = queue.Queue()

        self.stop_event = threading.Event()
        self.socket_lock = threading.Lock()
        self.active_socket: socket.socket | None = None

        self.event_timer = self.create_timer(
            0.02,
            self.process_network_events,
        )

        self.status_timer = self.create_timer(
            0.5,
            self.publish_status,
        )

        self.network_thread = threading.Thread(
            target=self.network_worker,
            name="so101-tcp-bridge",
            daemon=False,
        )
        self.network_thread.start()

        self.get_logger().info(
            "SO-101 TCP bridge started in receive-only stage 0 mode"
        )
        self.get_logger().info(
            "No joint command subscriber exists; "
            "no hardware command can be sent"
        )

    def enqueue_event(
        self,
        event_type: str,
        data: Any,
    ) -> None:
        self.event_queue.put((event_type, data))

    def update_connection_state(
        self,
        new_state: str,
    ) -> None:
        if new_state == self.connection_state:
            return

        previous_state = self.connection_state
        self.connection_state = new_state

        self.get_logger().info(
            f"Connection state: "
            f"{previous_state} -> {new_state}"
        )

    def publish_status(self) -> None:
        state_message = String()
        state_message.data = self.connection_state
        self.connection_state_publisher.publish(state_message)

        hardware_message = Bool()
        hardware_message.data = self.hardware_connected
        self.hardware_connected_publisher.publish(
            hardware_message
        )

        command_message = Bool()
        command_message.data = self.command_enabled
        self.command_enabled_publisher.publish(
            command_message
        )

        estop_message = Bool()
        estop_message.data = self.estop_active
        self.estop_active_publisher.publish(estop_message)

    def publish_joint_state(
        self,
        payload: dict[str, Any],
    ) -> None:
        validated = validate_joint_state_payload(payload)

        message = JointState()
        message.header.stamp = (
            self.get_clock().now().to_msg()
        )
        message.header.frame_id = (
            self.joint_state_frame_id
        )

        message.name = validated["names"]
        message.position = validated["position"]
        message.velocity = validated["velocity"]
        message.effort = validated["effort"]

        self.joint_state_publisher.publish(message)

    def process_network_events(self) -> None:
        processed = 0
        max_events_per_cycle = 200

        while processed < max_events_per_cycle:
            try:
                event_type, data = (
                    self.event_queue.get_nowait()
                )
            except queue.Empty:
                break

            processed += 1

            try:
                if event_type == "connection_state":
                    self.update_connection_state(str(data))

                elif event_type == "joint_state":
                    self.publish_joint_state(data)

                elif event_type == "server_status":
                    self.hardware_connected = bool(
                        data["hardware_connected"]
                    )
                    self.command_enabled = bool(
                        data["command_enabled"]
                    )
                    self.estop_active = bool(
                        data["estop_active"]
                    )

                elif event_type == "info":
                    self.get_logger().info(str(data))

                elif event_type == "warning":
                    self.get_logger().warning(str(data))

                elif event_type == "error":
                    self.get_logger().error(str(data))

            except Exception as exc:
                self.get_logger().error(
                    f"Failed to process network event "
                    f"{event_type!r}: {exc}"
                )

    def create_outgoing_message(
        self,
        message_type: str,
        payload: dict[str, Any],
        sequence_number: int,
    ) -> bytes:
        message = {
            "protocol_version": PROTOCOL_VERSION,
            "type": message_type,
            "seq": sequence_number,
            "timestamp_monotonic": time.monotonic(),
            "payload": payload,
        }

        raw = json.dumps(
            message,
            ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8") + b"\n"

        if len(raw) > self.max_message_bytes:
            raise ProtocolError(
                "outgoing message exceeds maximum size"
            )

        return raw

    def close_active_socket(self) -> None:
        with self.socket_lock:
            connection = self.active_socket
            self.active_socket = None

        if connection is None:
            return

        try:
            connection.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass

        try:
            connection.close()
        except OSError:
            pass

    def sleep_interruptibly(self, duration_s: float) -> None:
        self.stop_event.wait(timeout=duration_s)

    def network_worker(self) -> None:
        while not self.stop_event.is_set():
            self.enqueue_event(
                "connection_state",
                "CONNECTING",
            )

            try:
                connection = socket.create_connection(
                    (self.host, self.port),
                    timeout=1.0,
                )

                with self.socket_lock:
                    self.active_socket = connection

                self.run_connection_session(connection)

            except (
                ConnectionError,
                TimeoutError,
                OSError,
                ProtocolError,
                json.JSONDecodeError,
                UnicodeDecodeError,
            ) as exc:
                if not self.stop_event.is_set():
                    self.enqueue_event(
                        "warning",
                        f"TCP session ended: {exc}",
                    )

            except Exception as exc:
                if not self.stop_event.is_set():
                    self.enqueue_event(
                        "error",
                        f"Unexpected TCP failure: {exc}",
                    )

            finally:
                self.close_active_socket()

                self.enqueue_event(
                    "server_status",
                    {
                        "hardware_connected": False,
                        "command_enabled": False,
                        "estop_active": False,
                    },
                )

                if not self.stop_event.is_set():
                    self.enqueue_event(
                        "connection_state",
                        "DISCONNECTED",
                    )
                    self.sleep_interruptibly(
                        self.reconnect_delay_s
                    )

        self.enqueue_event(
            "connection_state",
            "STOPPED",
        )

    def run_connection_session(
        self,
        connection: socket.socket,
    ) -> None:
        connection.settimeout(0.02)
        connection.setsockopt(
            socket.IPPROTO_TCP,
            socket.TCP_NODELAY,
            1,
        )

        self.enqueue_event(
            "connection_state",
            "HANDSHAKING",
        )

        outgoing_seq = 1
        last_server_seq = 0
        receive_buffer = b""

        connected_time = time.monotonic()
        last_valid_rx_time = connected_time
        last_heartbeat_tx_time = 0.0

        handshake_complete = False

        hello_message = self.create_outgoing_message(
            "hello",
            {
                "role": "ros2_client",
                "session_id": str(uuid.uuid4()),
                "supported_protocol_versions": [
                    PROTOCOL_VERSION
                ],
                "capabilities": [
                    "receive_joint_state",
                    "send_heartbeat",
                ],
            },
            outgoing_seq,
        )
        connection.sendall(hello_message)
        outgoing_seq += 1

        while not self.stop_event.is_set():
            now = time.monotonic()

            if not handshake_complete:
                if (
                    now - connected_time
                    > self.handshake_timeout_s
                ):
                    raise TimeoutError(
                        "server handshake timed out"
                    )

            else:
                if (
                    now - last_valid_rx_time
                    > self.peer_timeout_s
                ):
                    self.enqueue_event(
                        "connection_state",
                        "STALE",
                    )
                    raise TimeoutError(
                        "server heartbeat/message timeout"
                    )

                if (
                    now - last_heartbeat_tx_time
                    >= self.heartbeat_period_s
                ):
                    heartbeat_message = (
                        self.create_outgoing_message(
                            "heartbeat",
                            {
                                "state": "ACTIVE",
                                "last_rx_seq": (
                                    last_server_seq
                                ),
                                "hardware_connected": False,
                                "command_enabled": False,
                                "estop_active": False,
                            },
                            outgoing_seq,
                        )
                    )

                    connection.sendall(
                        heartbeat_message
                    )
                    outgoing_seq += 1
                    last_heartbeat_tx_time = now

            try:
                chunk = connection.recv(4096)
            except socket.timeout:
                continue

            if chunk == b"":
                raise ConnectionError(
                    "server closed the connection"
                )

            receive_buffer += chunk

            if (
                len(receive_buffer)
                > self.max_message_bytes
                and b"\n" not in receive_buffer
            ):
                raise ProtocolError(
                    "incoming message exceeds maximum size"
                )

            while b"\n" in receive_buffer:
                raw_line, receive_buffer = (
                    receive_buffer.split(b"\n", 1)
                )

                if not raw_line:
                    continue

                if len(raw_line) > self.max_message_bytes:
                    raise ProtocolError(
                        "incoming message exceeds maximum size"
                    )

                decoded_line = raw_line.decode("utf-8")
                message = validate_envelope(
                    json.loads(decoded_line)
                )

                sequence_number = message["seq"]

                if sequence_number <= last_server_seq:
                    raise ProtocolError(
                        "stale or duplicate server sequence: "
                        f"{sequence_number} <= "
                        f"{last_server_seq}"
                    )

                last_server_seq = sequence_number
                last_valid_rx_time = time.monotonic()

                if not handshake_complete:
                    self.process_hello_ack(message)
                    handshake_complete = True

                    self.enqueue_event(
                        "connection_state",
                        "ACTIVE",
                    )
                    continue

                self.process_server_message(message)

    def process_hello_ack(
        self,
        message: dict[str, Any],
    ) -> None:
        if message["type"] != "hello_ack":
            raise ProtocolError(
                "first server message must be hello_ack"
            )

        payload = message["payload"]

        required_fields = {
            "accepted_version",
            "server_mode",
            "hardware_connected",
            "command_enabled",
            "joint_names",
        }

        missing = required_fields - payload.keys()

        if missing:
            raise ProtocolError(
                "hello_ack missing fields: "
                f"{sorted(missing)}"
            )

        if payload["accepted_version"] != PROTOCOL_VERSION:
            raise ProtocolError(
                "server accepted an incompatible version"
            )

        if payload["server_mode"] != "simulation":
            raise ProtocolError(
                "stage 0 requires server_mode=simulation"
            )

        if payload["hardware_connected"] is not False:
            raise ProtocolError(
                "stage 0 server must report "
                "hardware_connected=false"
            )

        if payload["command_enabled"] is not False:
            raise ProtocolError(
                "stage 0 server must report "
                "command_enabled=false"
            )

        joint_names = payload["joint_names"]

        if not isinstance(joint_names, list):
            raise ProtocolError(
                "hello_ack joint_names must be a list"
            )

        if not joint_names:
            raise ProtocolError(
                "hello_ack joint_names must not be empty"
            )

        if not all(
            isinstance(name, str)
            and name.startswith("mock_")
            for name in joint_names
        ):
            raise ProtocolError(
                "stage 0 joint names must start with mock_"
            )

        self.enqueue_event(
            "server_status",
            {
                "hardware_connected": False,
                "command_enabled": False,
                "estop_active": False,
            },
        )

        self.enqueue_event(
            "info",
            "TCP handshake completed with simulation server",
        )

    def process_server_message(
        self,
        message: dict[str, Any],
    ) -> None:
        message_type = message["type"]
        payload = message["payload"]

        if message_type == "heartbeat":
            required_fields = {
                "state",
                "last_rx_seq",
                "hardware_connected",
                "command_enabled",
                "estop_active",
            }

            missing = required_fields - payload.keys()

            if missing:
                raise ProtocolError(
                    "heartbeat missing fields: "
                    f"{sorted(missing)}"
                )

            if not all(
                isinstance(payload[field], bool)
                for field in (
                    "hardware_connected",
                    "command_enabled",
                    "estop_active",
                )
            ):
                raise ProtocolError(
                    "heartbeat status flags must be bool"
                )

            if payload["hardware_connected"] is not False:
                raise ProtocolError(
                    "stage 0 server unexpectedly reports "
                    "hardware_connected=true"
                )

            if payload["command_enabled"] is not False:
                raise ProtocolError(
                    "stage 0 server unexpectedly reports "
                    "command_enabled=true"
                )

            self.enqueue_event(
                "server_status",
                {
                    "hardware_connected": (
                        payload["hardware_connected"]
                    ),
                    "command_enabled": (
                        payload["command_enabled"]
                    ),
                    "estop_active": (
                        payload["estop_active"]
                    ),
                },
            )
            return

        if message_type == "joint_state":
            validated = validate_joint_state_payload(
                payload
            )

            if not all(
                name.startswith("mock_")
                for name in validated["names"]
            ):
                raise ProtocolError(
                    "stage 0 joint state contains "
                    "non-mock joint names"
                )

            self.enqueue_event(
                "joint_state",
                validated,
            )
            return

        if message_type == "error":
            code = payload.get("code", "UNKNOWN")
            description = payload.get(
                "message",
                "server reported an unspecified error",
            )
            fatal = payload.get("fatal", False)

            self.enqueue_event(
                "error" if fatal else "warning",
                f"Server error {code}: {description}",
            )

            if fatal:
                raise ProtocolError(
                    f"fatal server error: {code}"
                )

            return

        if message_type == "command_ack":
            self.enqueue_event(
                "warning",
                "Received command_ack although stage 0 "
                "client never sends joint commands",
            )
            return

        raise ProtocolError(
            "unexpected server message type: "
            f"{message_type!r}"
        )

    def destroy_node(self) -> bool:
        self.stop_event.set()
        self.close_active_socket()

        if self.network_thread.is_alive():
            self.network_thread.join(timeout=2.0)

        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)

    node: So101RobotBridgeNode | None = None

    try:
        node = So101RobotBridgeNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()