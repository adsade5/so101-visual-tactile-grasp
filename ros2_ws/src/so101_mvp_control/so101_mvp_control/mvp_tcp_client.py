from __future__ import annotations

import json
import socket
import threading
from typing import Any


class MvpTcpError(RuntimeError):
    def __init__(self, kind: str, message: str) -> None:
        self.kind = kind
        super().__init__(f"{kind}: {message}")


class MvpTcpMotionResultUnknown(MvpTcpError):
    def __init__(self, reason: str) -> None:
        super().__init__("motion_result_unknown", reason)


def _normalize_error(exc: BaseException) -> MvpTcpError:
    if isinstance(exc, MvpTcpError):
        return exc
    if isinstance(exc, ConnectionRefusedError):
        return MvpTcpError("tcp_connection_refused", str(exc))
    if isinstance(exc, ConnectionResetError):
        return MvpTcpError("tcp_connection_closed", str(exc))
    if isinstance(exc, BrokenPipeError):
        return MvpTcpError("tcp_connection_closed", str(exc))
    if isinstance(exc, socket.timeout):
        return MvpTcpError("tcp_request_timeout", str(exc) or "timed out")
    if isinstance(exc, TimeoutError):
        return MvpTcpError("tcp_request_timeout", str(exc) or "timed out")
    if isinstance(exc, json.JSONDecodeError):
        return MvpTcpError("protocol_error", str(exc))
    if isinstance(exc, OSError):
        return MvpTcpError(type(exc).__name__, str(exc))
    return MvpTcpError(type(exc).__name__, str(exc))


class MvpTcpClient:
    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 8770,
        timeout_s: float | None = None,
        *,
        connect_timeout_s: float = 2.0,
        state_request_timeout_s: float = 2.0,
        motion_request_timeout_s: float = 120.0,
        request_timeout_s: float | None = None,
    ) -> None:
        if timeout_s is not None:
            connect_timeout_s = float(timeout_s)
            state_request_timeout_s = float(timeout_s)
        if request_timeout_s is not None:
            state_request_timeout_s = float(request_timeout_s)
        self.host = host
        self.port = int(port)
        self.connect_timeout_s = float(connect_timeout_s)
        self.state_request_timeout_s = float(state_request_timeout_s)
        self.motion_request_timeout_s = float(motion_request_timeout_s)
        self._socket: socket.socket | None = None
        self._receive_buffer = b""
        self._request_lock = threading.RLock()
        self.last_error: BaseException | None = None
        self._last_send_completed = False

    @property
    def is_connected(self) -> bool:
        return self._socket is not None

    def connect(self) -> None:
        if self._socket is not None:
            return
        try:
            sock = socket.create_connection(
                (self.host, self.port),
                timeout=self.connect_timeout_s,
            )
            self._socket = sock
            self._receive_buffer = b""
            self.last_error = None
        except BaseException as exc:
            error = _normalize_error(exc)
            self.last_error = error
            self.close()
            raise error from exc

    def close(self) -> None:
        sock = self._socket
        self._socket = None
        self._receive_buffer = b""
        if sock is None:
            return
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            sock.close()
        except OSError:
            pass

    def _request_once(self, payload: dict[str, Any], timeout_s: float) -> dict[str, Any]:
        self.connect()
        if self._socket is None:
            raise MvpTcpError("tcp_connection_closed", "socket is not connected")
        self._socket.settimeout(timeout_s)
        self._last_send_completed = False
        try:
            data = json.dumps(payload, separators=(",", ":")).encode("utf-8") + b"\n"
            self._socket.sendall(data)
            self._last_send_completed = True
            while b"\n" not in self._receive_buffer:
                chunk = self._socket.recv(4096)
                if chunk == b"":
                    raise MvpTcpError("tcp_connection_closed", "server closed the connection")
                self._receive_buffer += chunk
            line, self._receive_buffer = self._receive_buffer.split(b"\n", 1)
            value = json.loads(line.decode("utf-8"))
            if not isinstance(value, dict):
                raise MvpTcpError("protocol_error", "response is not a JSON object")
            self.last_error = None
            return value
        except BaseException as exc:
            error = _normalize_error(exc)
            self.last_error = error
            self.close()
            raise error from exc

    def get_state(self) -> dict[str, Any]:
        with self._request_lock:
            return self._request_once(
                {"command": "get_state"},
                self.state_request_timeout_s,
            )

    def move_joints_sequential(
        self,
        target_rad: list[float],
        speed_rad_s: float,
        joint_order: list[int],
        *,
        confirm: str = "MVP_MOVE",
        gripper_target_pos: float | None = None,
        stop_gripper_on_tactile_contact: bool = False,
    ) -> dict[str, Any]:
        payload = {
            "command": "move_joints_sequential",
            "target_rad": target_rad,
            "speed_rad_s": speed_rad_s,
            "joint_order": joint_order,
            "confirm": confirm,
        }
        if gripper_target_pos is not None:
            payload["gripper_target_pos"] = float(gripper_target_pos)
        if stop_gripper_on_tactile_contact:
            payload["stop_gripper_on_tactile_contact"] = True
        with self._request_lock:
            try:
                return self._request_once(payload, self.motion_request_timeout_s)
            except MvpTcpError as exc:
                if self._last_send_completed:
                    raise MvpTcpMotionResultUnknown(f"{exc.kind}: {exc}") from exc
                raise

    def request(self, payload: dict[str, Any]) -> dict[str, Any]:
        command = payload.get("command")
        if command == "move_joints_sequential":
            return self.move_joints_sequential(
                list(payload["target_rad"]),
                float(payload["speed_rad_s"]),
                [int(value) for value in payload["joint_order"]],
                confirm=str(payload.get("confirm", "MVP_MOVE")),
                gripper_target_pos=payload.get("gripper_target_pos"),
                stop_gripper_on_tactile_contact=bool(payload.get("stop_gripper_on_tactile_contact", False)),
            )
        return self.get_state()
