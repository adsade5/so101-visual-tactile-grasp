from __future__ import annotations

import json
import math
import socket
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from .command_gate_validator import (
    DEFAULT_PROJECT_ROOT,
    load_command_gate_joint_limits,
)


EXPECTED_TOPIC_PERIOD_S = 0.05


class RealJointStateBridgeNode(Node):
    def __init__(
        self,
        *,
        project_root_override: str | Path | None = None,
        host_override: str | None = None,
        port_override: int | None = None,
        timeout_s_override: float | None = None,
    ) -> None:
        super().__init__("real_joint_state_bridge_node")

        self.declare_parameter("project_root", DEFAULT_PROJECT_ROOT)
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8766)
        self.declare_parameter("timeout_s", 0.30)

        self.project_root = Path(
            str(project_root_override or self.get_parameter("project_root").value)
        ).resolve()
        self.host = str(host_override or self.get_parameter("host").value)
        self.port = int(port_override or self.get_parameter("port").value)
        self.timeout_s = float(timeout_s_override or self.get_parameter("timeout_s").value)
        self.lower, self.upper, self.model_metadata = load_command_gate_joint_limits(
            self.project_root
        )
        self.joint_names = [str(name) for name in self.model_metadata["joint_names"]]

        self.joint_state_publisher = self.create_publisher(
            JointState,
            "/real_joint_states",
            10,
        )
        self.valid_publisher = self.create_publisher(
            Bool,
            "/real_joint_state_valid",
            10,
        )
        self.status_publisher = self.create_publisher(
            String,
            "/real_joint_state_status",
            10,
        )
        self.diagnostic_publisher = self.create_publisher(
            String,
            "/real_joint_state_diagnostic",
            10,
        )

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._last_sequence: int | None = None
        self._last_message_time: float | None = None
        self._latest_valid = False
        self._latest_reason = "tcp_disconnected"
        self._latest_status: dict[str, Any] = {
            "status": "INVALID",
            "reason": "tcp_disconnected",
        }
        self._message_count = 0
        self._valid_message_count = 0
        self._invalid_message_count = 0
        self._first_valid_time: float | None = None
        self._last_valid_time: float | None = None
        self._last_positions_rad: list[float] | None = None
        self._last_raw_payload: dict[str, Any] | None = None

        self.create_timer(EXPECTED_TOPIC_PERIOD_S, self.publish_heartbeat)
        self._thread = threading.Thread(target=self._network_loop, daemon=True)
        self._thread.start()

        self.get_logger().warning(
            "REAL SO-101 READ-ONLY JOINT STATE BRIDGE | "
            "publishes /real_joint_states only | hardware motion disabled | "
            f"tcp={self.host}:{self.port}"
        )

    def destroy_node(self) -> bool:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        return super().destroy_node()

    def publish_heartbeat(self) -> None:
        now = time.monotonic()
        with self._lock:
            latest = dict(self._latest_status)
            last_message_time = self._last_message_time
            valid = bool(self._latest_valid)
            if last_message_time is None:
                valid = False
                latest["status"] = "INVALID"
                latest["reason"] = self._latest_reason
            else:
                age_s = now - last_message_time
                latest["last_message_age_s"] = age_s
                if age_s > self.timeout_s:
                    valid = False
                    latest["status"] = "INVALID"
                    latest["reason"] = "real_joint_state_timeout"
                    self._latest_valid = False
                    self._latest_reason = "real_joint_state_timeout"
            latest["timestamp_ros_s"] = self._ros_timestamp_float()
            latest["message_count"] = self._message_count
            latest["valid_message_count"] = self._valid_message_count
            latest["invalid_message_count"] = self._invalid_message_count

        valid_message = Bool()
        valid_message.data = valid
        self.valid_publisher.publish(valid_message)

        status_message = String()
        status_message.data = json.dumps(latest, ensure_ascii=False, allow_nan=False)
        self.status_publisher.publish(status_message)

    def _network_loop(self) -> None:
        while not self._stop.is_set():
            try:
                with socket.create_connection((self.host, self.port), timeout=0.25) as sock:
                    sock.settimeout(0.25)
                    with self._lock:
                        self._latest_reason = "waiting_for_real_joint_state"
                        self._latest_status = {
                            "status": "INVALID",
                            "reason": "waiting_for_real_joint_state",
                            "tcp_connected": True,
                            "host": self.host,
                            "port": self.port,
                        }
                    self._read_lines(sock)
            except OSError as error:
                self._mark_disconnected(repr(error))
                self._stop.wait(0.10)

    def _read_lines(self, sock: socket.socket) -> None:
        buffer = b""
        while not self._stop.is_set():
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                self._mark_disconnected("peer_closed")
                return
            buffer += chunk
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                if line.strip():
                    self._handle_json_line(line.decode("utf-8", errors="replace"))

    def _mark_disconnected(self, detail: str) -> None:
        with self._lock:
            self._latest_valid = False
            self._latest_reason = "tcp_disconnected"
            self._last_message_time = None
            self._latest_status = {
                "status": "INVALID",
                "reason": "tcp_disconnected",
                "detail": detail,
                "tcp_connected": False,
                "host": self.host,
                "port": self.port,
            }
            status = dict(self._latest_status)
        self._publish_status_now(False, status)

    def _publish_status_now(self, valid: bool, status: dict[str, Any]) -> None:
        payload = dict(status)
        payload["timestamp_ros_s"] = self._ros_timestamp_float()
        valid_message = Bool()
        valid_message.data = bool(valid)
        self.valid_publisher.publish(valid_message)

        status_message = String()
        status_message.data = json.dumps(payload, ensure_ascii=False, allow_nan=False)
        self.status_publisher.publish(status_message)

    def _handle_json_line(self, line: str) -> None:
        now = time.monotonic()
        try:
            payload = json.loads(line)
        except json.JSONDecodeError as error:
            self._publish_diagnostic(
                {
                    "status": "INVALID",
                    "reason": "schema_error",
                    "detail": repr(error),
                    "raw_line": line,
                }
            )
            self._record_invalid("schema_error", {"detail": repr(error)})
            return
        if not isinstance(payload, dict):
            self._publish_diagnostic(
                {
                    "status": "INVALID",
                    "reason": "schema_error",
                    "detail": "payload_not_object",
                    "payload": payload,
                }
            )
            self._record_invalid("schema_error", {"detail": "payload_not_object"})
            return
        self._publish_diagnostic(payload)
        if payload.get("type") == "joint_state_status":
            reason = str(payload.get("reason", payload.get("status", "remote_status")))
            self._record_invalid(reason, self._copy_diagnostic_fields(payload))
            return
        valid, reason, positions = self._validate_payload(payload)
        with self._lock:
            self._message_count += 1
            self._last_raw_payload = dict(payload)
            if not valid:
                self._invalid_message_count += 1
                self._latest_valid = False
                self._latest_reason = reason
                self._latest_status = self._status_payload(
                    reason,
                    payload,
                    positions,
                    now,
                    False,
                )
                return
            assert positions is not None
            sequence = int(payload["sequence"])
            self._last_sequence = sequence
            self._last_message_time = now
            self._last_positions_rad = positions
            self._valid_message_count += 1
            if self._first_valid_time is None:
                self._first_valid_time = now
            self._last_valid_time = now
            self._latest_valid = True
            self._latest_reason = "valid_real_joint_state"
            self._latest_status = self._status_payload(
                "valid_real_joint_state",
                payload,
                positions,
                now,
                True,
            )
        self._publish_joint_state(positions)

    def _validate_payload(
        self,
        payload: dict[str, Any],
    ) -> tuple[bool, str, list[float] | None]:
        if payload.get("type") != "joint_state":
            return False, "schema_error", None
        if payload.get("read_only") is not True:
            return False, "read_only_flag_missing", None
        for forbidden_flag in (
            "torque_state_changed",
            "motion_command_sent",
            "goal_position_written",
            "motion_parameters_written",
        ):
            if bool(payload.get(forbidden_flag, False)):
                return False, "forbidden_motion_flag_true", None
        try:
            sequence = int(payload["sequence"])
        except (KeyError, TypeError, ValueError):
            return False, "schema_error", None
        if self._last_sequence is not None and sequence <= self._last_sequence:
            return False, "sequence_regression", None
        names = payload.get("joint_names")
        positions = payload.get("positions_rad")
        if names != self.joint_names:
            return False, "wrong_joint_names", None
        if not isinstance(positions, list) or len(positions) != len(self.joint_names):
            return False, "wrong_position_length", None
        try:
            values = [float(value) for value in positions]
        except (TypeError, ValueError):
            return False, "non_finite_position", None
        if not all(math.isfinite(value) for value in values):
            return False, "non_finite_position", None
        array = np.asarray(values, dtype=np.float64)
        if np.any(array < self.lower - 1.0e-10) or np.any(array > self.upper + 1.0e-10):
            return False, "current_joint_state_out_of_bounds", values
        return True, "valid_real_joint_state", values

    def _publish_diagnostic(self, payload: dict[str, Any]) -> None:
        diagnostic = String()
        diagnostic.data = json.dumps(payload, ensure_ascii=False, allow_nan=False)
        self.diagnostic_publisher.publish(diagnostic)

    def _copy_diagnostic_fields(self, payload: dict[str, Any]) -> dict[str, Any]:
        keys = (
            "source_positions",
            "source_unit",
            "raw_encoder_values",
            "motor_degrees_before_calibration",
            "lerobot_calibrated_degrees",
            "mapped_positions_rad",
            "positions_rad",
            "per_joint_validation",
            "violating_joints",
            "calibration_applied",
            "calibration_path",
            "mapping_status",
            "mapped_joint_state_valid",
            "mapping_failure_reason",
            "motor_bus_read_valid",
        )
        return {key: payload.get(key) for key in keys if key in payload}

    def _record_invalid(self, reason: str, details: dict[str, Any]) -> None:
        with self._lock:
            self._message_count += 1
            self._invalid_message_count += 1
            self._latest_valid = False
            self._latest_reason = reason
            self._latest_status = {
                "status": "INVALID",
                "reason": reason,
                "tcp_connected": True,
                "host": self.host,
                "port": self.port,
                **details,
            }

    def _status_payload(
        self,
        reason: str,
        payload: dict[str, Any],
        positions_rad: list[float] | None,
        now_monotonic: float,
        valid: bool,
    ) -> dict[str, Any]:
        sample_rate_hz = 0.0
        if (
            self._first_valid_time is not None
            and self._last_valid_time is not None
            and self._valid_message_count > 1
            and self._last_valid_time > self._first_valid_time
        ):
            sample_rate_hz = (
                float(self._valid_message_count - 1)
                / (self._last_valid_time - self._first_valid_time)
            )
        diagnostic_fields = self._copy_diagnostic_fields(payload)
        return {
            "status": "VALID" if valid else "INVALID",
            "reason": reason,
            "tcp_connected": True,
            "host": self.host,
            "port": self.port,
            "sequence": payload.get("sequence"),
            "source": payload.get("source"),
            "joint_names": list(self.joint_names),
            "positions_rad": positions_rad if positions_rad is not None else payload.get("positions_rad"),
            "sample_rate_hz": sample_rate_hz,
            "last_message_age_s": time.monotonic() - now_monotonic,
            "read_only": payload.get("read_only") is True,
            "torque_state_changed": bool(payload.get("torque_state_changed", False)),
            "motion_command_sent": bool(payload.get("motion_command_sent", False)),
            "goal_position_written": bool(payload.get("goal_position_written", False)),
            "motion_parameters_written": bool(
                payload.get("motion_parameters_written", False)
            ),
            "calibration_path": payload.get("calibration_path"),
            "follower_port": payload.get("follower_port"),
            "mapping_status": payload.get("mapping_status"),
            **diagnostic_fields,
        }

    def _publish_joint_state(self, positions: list[float]) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self.joint_names)
        message.position = [float(value) for value in positions]
        message.velocity = [0.0] * len(self.joint_names)
        message.effort = []
        self.joint_state_publisher.publish(message)

    def _ros_timestamp_float(self) -> float:
        stamp = self.get_clock().now().to_msg()
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: RealJointStateBridgeNode | None = None
    try:
        node = RealJointStateBridgeNode()
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
