from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
EXAMPLES_PATH = (
    PROJECT_ROOT
    / "shared_protocol"
    / "examples"
    / "protocol_examples.jsonl"
)
SCHEMA_PATH = (
    PROJECT_ROOT
    / "shared_protocol"
    / "message_envelope.schema.json"
)

PROTOCOL_VERSION = "1.0"

ALLOWED_TYPES = {
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


class ProtocolValidationError(ValueError):
    """Raised when a protocol message violates the v1 contract."""


def require_fields(
    payload: dict[str, Any],
    fields: set[str],
    message_type: str,
) -> None:
    missing = fields - payload.keys()
    if missing:
        raise ProtocolValidationError(
            f"{message_type} payload missing fields: {sorted(missing)}"
        )


def require_bool(value: Any, field_name: str) -> None:
    if not isinstance(value, bool):
        raise ProtocolValidationError(
            f"{field_name} must be bool, got {type(value).__name__}"
        )


def require_non_empty_string(value: Any, field_name: str) -> None:
    if not isinstance(value, str) or not value:
        raise ProtocolValidationError(
            f"{field_name} must be a non-empty string"
        )


def require_number(value: Any, field_name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolValidationError(f"{field_name} must be numeric")

    if not math.isfinite(float(value)):
        raise ProtocolValidationError(f"{field_name} must be finite")


def validate_joint_arrays(payload: dict[str, Any]) -> None:
    names = payload.get("names")
    positions = payload.get("position")

    if not isinstance(names, list) or not names:
        raise ProtocolValidationError("names must be a non-empty list")

    if not all(isinstance(name, str) and name for name in names):
        raise ProtocolValidationError(
            "every joint name must be a non-empty string"
        )

    if not isinstance(positions, list):
        raise ProtocolValidationError("position must be a list")

    if len(names) != len(positions):
        raise ProtocolValidationError(
            "names and position must have the same length"
        )

    for index, value in enumerate(positions):
        require_number(value, f"position[{index}]")

    for optional_field in ("velocity", "effort"):
        values = payload.get(optional_field)
        if values is None:
            continue

        if not isinstance(values, list):
            raise ProtocolValidationError(
                f"{optional_field} must be a list"
            )

        if len(values) != len(names):
            raise ProtocolValidationError(
                f"{optional_field} length must equal names length"
            )

        for index, value in enumerate(values):
            require_number(value, f"{optional_field}[{index}]")


def validate_payload(message_type: str, payload: dict[str, Any]) -> None:
    if message_type == "hello":
        require_fields(
            payload,
            {
                "role",
                "session_id",
                "supported_protocol_versions",
                "capabilities",
            },
            message_type,
        )
        require_non_empty_string(payload["role"], "role")
        require_non_empty_string(payload["session_id"], "session_id")

        versions = payload["supported_protocol_versions"]
        if (
            not isinstance(versions, list)
            or PROTOCOL_VERSION not in versions
        ):
            raise ProtocolValidationError(
                "hello must support protocol version 1.0"
            )

        if not isinstance(payload["capabilities"], list):
            raise ProtocolValidationError("capabilities must be a list")

    elif message_type == "hello_ack":
        require_fields(
            payload,
            {
                "accepted_version",
                "server_mode",
                "hardware_connected",
                "command_enabled",
                "joint_names",
            },
            message_type,
        )

        if payload["accepted_version"] != PROTOCOL_VERSION:
            raise ProtocolValidationError(
                "hello_ack accepted_version must be 1.0"
            )

        require_bool(
            payload["hardware_connected"],
            "hardware_connected",
        )
        require_bool(payload["command_enabled"], "command_enabled")

        joint_names = payload["joint_names"]
        if not isinstance(joint_names, list) or not joint_names:
            raise ProtocolValidationError(
                "joint_names must be a non-empty list"
            )

    elif message_type == "heartbeat":
        require_fields(
            payload,
            {
                "state",
                "last_rx_seq",
                "hardware_connected",
                "command_enabled",
                "estop_active",
            },
            message_type,
        )

        require_non_empty_string(payload["state"], "state")

        if (
            isinstance(payload["last_rx_seq"], bool)
            or not isinstance(payload["last_rx_seq"], int)
            or payload["last_rx_seq"] < 0
        ):
            raise ProtocolValidationError(
                "last_rx_seq must be a non-negative integer"
            )

        require_bool(
            payload["hardware_connected"],
            "hardware_connected",
        )
        require_bool(payload["command_enabled"], "command_enabled")
        require_bool(payload["estop_active"], "estop_active")

    elif message_type == "joint_state":
        require_fields(payload, {"names", "position"}, message_type)
        validate_joint_arrays(payload)

    elif message_type == "joint_command":
        require_fields(
            payload,
            {
                "command_id",
                "mode",
                "names",
                "position",
                "duration_s",
            },
            message_type,
        )

        require_non_empty_string(payload["command_id"], "command_id")

        if payload["mode"] != "position":
            raise ProtocolValidationError(
                "protocol v1 supports only position mode"
            )

        validate_joint_arrays(payload)
        require_number(payload["duration_s"], "duration_s")

        if float(payload["duration_s"]) <= 0:
            raise ProtocolValidationError(
                "duration_s must be positive"
            )

    elif message_type == "command_ack":
        require_fields(
            payload,
            {
                "command_id",
                "accepted",
                "reason",
                "server_state",
            },
            message_type,
        )

        require_non_empty_string(payload["command_id"], "command_id")
        require_bool(payload["accepted"], "accepted")
        require_non_empty_string(payload["reason"], "reason")
        require_non_empty_string(
            payload["server_state"],
            "server_state",
        )

    elif message_type == "estop":
        require_fields(payload, {"reason"}, message_type)
        require_non_empty_string(payload["reason"], "reason")

    elif message_type == "clear_estop":
        require_fields(
            payload,
            {"request_id", "reason"},
            message_type,
        )
        require_non_empty_string(payload["request_id"], "request_id")
        require_non_empty_string(payload["reason"], "reason")

    elif message_type == "error":
        require_fields(
            payload,
            {
                "code",
                "message",
                "fatal",
                "related_seq",
            },
            message_type,
        )

        require_non_empty_string(payload["code"], "code")
        require_non_empty_string(payload["message"], "message")
        require_bool(payload["fatal"], "fatal")

        related_seq = payload["related_seq"]
        if related_seq is not None:
            if (
                isinstance(related_seq, bool)
                or not isinstance(related_seq, int)
                or related_seq < 1
            ):
                raise ProtocolValidationError(
                    "related_seq must be null or a positive integer"
                )


def validate_message(message: Any) -> None:
    if not isinstance(message, dict):
        raise ProtocolValidationError("message must be a JSON object")

    actual_fields = set(message)
    missing = REQUIRED_ENVELOPE_FIELDS - actual_fields
    extra = actual_fields - REQUIRED_ENVELOPE_FIELDS

    if missing:
        raise ProtocolValidationError(
            f"message missing fields: {sorted(missing)}"
        )

    if extra:
        raise ProtocolValidationError(
            f"message has unexpected fields: {sorted(extra)}"
        )

    if message["protocol_version"] != PROTOCOL_VERSION:
        raise ProtocolValidationError(
            "protocol_version must equal 1.0"
        )

    message_type = message["type"]
    if message_type not in ALLOWED_TYPES:
        raise ProtocolValidationError(
            f"unsupported message type: {message_type!r}"
        )

    seq = message["seq"]
    if isinstance(seq, bool) or not isinstance(seq, int) or seq < 1:
        raise ProtocolValidationError(
            "seq must be a positive integer"
        )

    require_number(
        message["timestamp_monotonic"],
        "timestamp_monotonic",
    )

    if float(message["timestamp_monotonic"]) < 0:
        raise ProtocolValidationError(
            "timestamp_monotonic must be non-negative"
        )

    payload = message["payload"]
    if not isinstance(payload, dict):
        raise ProtocolValidationError("payload must be an object")

    validate_payload(message_type, payload)


def main() -> int:
    if not SCHEMA_PATH.is_file():
        print(f"FAIL: schema not found: {SCHEMA_PATH}")
        return 1

    if not EXAMPLES_PATH.is_file():
        print(f"FAIL: examples not found: {EXAMPLES_PATH}")
        return 1

    try:
        with SCHEMA_PATH.open("r", encoding="utf-8") as file:
            schema = json.load(file)
    except (OSError, json.JSONDecodeError) as exc:
        print(f"FAIL: schema is not valid JSON: {exc}")
        return 1

    if schema.get("title") != "SO-101 TCP Bridge Message Envelope v1":
        print("FAIL: unexpected schema title")
        return 1

    messages: list[dict[str, Any]] = []

    try:
        with EXAMPLES_PATH.open("r", encoding="utf-8") as file:
            for line_number, raw_line in enumerate(file, start=1):
                stripped = raw_line.strip()
                if not stripped:
                    continue

                try:
                    message = json.loads(stripped)
                except json.JSONDecodeError as exc:
                    raise ProtocolValidationError(
                        f"line {line_number} contains invalid JSON: {exc}"
                    ) from exc

                try:
                    validate_message(message)
                except ProtocolValidationError as exc:
                    raise ProtocolValidationError(
                        f"line {line_number}: {exc}"
                    ) from exc

                messages.append(message)

    except (OSError, ProtocolValidationError) as exc:
        print(f"FAIL: {exc}")
        return 1

    found_types = {message["type"] for message in messages}
    expected_example_types = {
        "hello",
        "hello_ack",
        "heartbeat",
        "joint_state",
        "joint_command",
        "command_ack",
        "estop",
        "error",
    }

    missing_examples = expected_example_types - found_types
    if missing_examples:
        print(
            "FAIL: missing example message types: "
            f"{sorted(missing_examples)}"
        )
        return 1

    print(f"PASS: validated {len(messages)} protocol examples")
    print(f"PASS: message types: {sorted(found_types)}")
    print(f"PASS: schema: {SCHEMA_PATH}")
    print(f"PASS: examples: {EXAMPLES_PATH}")
    return 0


if __name__ == "__main__":
    sys.exit(main())