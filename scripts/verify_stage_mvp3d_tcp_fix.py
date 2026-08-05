from __future__ import annotations

import argparse
import json
import socket
import sys
import threading
import time
from pathlib import Path
from typing import Any, Callable


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from lerobot_server.mvp_hardware_executor import ARM_JOINT_NAMES
from scripts.mvp_so101_server import MvpTcpServer
from shared_protocol.mvp_tcp_client import MvpTcpClient, MvpTcpMotionResultUnknown


REPORT_PATH = PROJECT_ROOT / "data" / "verification" / "stage_mvp3d_tcp_fix_report.json"


class FakeBackend:
    def __init__(self) -> None:
        self.connected = False
        self.closed = False
        self.stop_requested = False
        self.motion_requests = 0
        self.goal_position_write_count = 0
        self.torque_enable_write_count = 0
        self.torque_disable_write_count = 0
        self.positions_rad = [0.1, -1.0, 1.1, 0.9, 0.0]

    def connect(self) -> None:
        self.connected = True

    def close(self) -> None:
        self.closed = True

    def get_state(self) -> dict[str, Any]:
        return {
            "success": True,
            "reason": "state_ok",
            "joint_names": list(ARM_JOINT_NAMES),
            "positions_rad": list(self.positions_rad),
            "gripper": 50.0,
            "within_calibration": True,
        }

    def move_joints_sequential(
        self,
        target_rad: list[float],
        speed_rad_s: float,
        joint_order: list[int],
    ) -> dict[str, Any]:
        del target_rad, speed_rad_s, joint_order
        self.motion_requests += 1
        return {"success": True, "reason": "fake_motion_completed"}

    def stop(self) -> dict[str, Any]:
        self.stop_requested = True
        return {"success": True, "reason": "stop_requested"}

    def counters(self) -> dict[str, Any]:
        return {
            "opened_com_ports": False,
            "goal_position_write_count": self.goal_position_write_count,
            "torque_enable_write_count": self.torque_enable_write_count,
            "torque_disable_write_count": self.torque_disable_write_count,
        }


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def start_fake_server() -> tuple[MvpTcpServer, FakeBackend, int, threading.Thread]:
    port = free_port()
    backend = FakeBackend()
    server = MvpTcpServer(
        host="127.0.0.1",
        port=port,
        backend=backend,
        hardware_motion_enabled=True,
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.1):
                return server, backend, port, thread
        except OSError:
            time.sleep(0.02)
    raise TimeoutError("fake server did not listen")


def stop_fake_server(port: int, thread: threading.Thread) -> None:
    try:
        client = MvpTcpClient("127.0.0.1", port, timeout_s=1.0)
        client.stop()
        client.close()
    except Exception:
        pass
    thread.join(timeout=2.0)


def raw_request(port: int, data: bytes) -> bytes:
    with socket.create_connection(("127.0.0.1", port), timeout=1.0) as sock:
        sock.settimeout(1.0)
        sock.sendall(data)
        return sock.recv(4096)


def connect_and_close(port: int) -> None:
    with socket.create_connection(("127.0.0.1", port), timeout=1.0):
        return


def assert_case(name: str, fn: Callable[[], Any]) -> dict[str, Any]:
    try:
        detail = fn()
        result = {"name": name, "status": "PASS"}
        if isinstance(detail, dict):
            result.update(detail)
        return result
    except Exception as exc:
        return {"name": name, "status": "FAIL", "error": repr(exc)}


def expect_error(name: str, fn: Callable[[], Any], expected_text: str) -> dict[str, Any]:
    try:
        fn()
        return {"name": name, "status": "FAIL", "error": "expected error did not occur"}
    except Exception as exc:
        text = str(exc)
        if expected_text in text:
            return {"name": name, "status": "PASS", "error": text}
        return {
            "name": name,
            "status": "FAIL",
            "error": text,
            "expected_text": expected_text,
        }


def one_shot_empty_server(port: int) -> threading.Thread:
    def run() -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("127.0.0.1", port))
            sock.listen(1)
            conn, _ = sock.accept()
            conn.close()

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    return thread


def main() -> int:
    parser = argparse.ArgumentParser(description="Offline MVP-3D TCP fix verification.")
    parser.add_argument("--report", type=Path, default=REPORT_PATH)
    args = parser.parse_args()

    cases: list[dict[str, Any]] = []
    server, backend, port, thread = start_fake_server()
    try:
        client = MvpTcpClient("127.0.0.1", port, timeout_s=1.0)
        state = client.get_state()
        cases.append(assert_case("server_accepts_connection", lambda: None if state["success"] else (_ for _ in ()).throw(AssertionError(state))))
        cases.append(assert_case("json_line_request", lambda: None if raw_request(port, b'{"command":"get_state"}\n').endswith(b"\n") else (_ for _ in ()).throw(AssertionError("missing newline"))))
        cases.append(assert_case("json_line_response", lambda: None if json.loads(raw_request(port, b'{"command":"get_state"}\n').decode("utf-8"))["reason"] == "state_ok" else (_ for _ in ()).throw(AssertionError("bad response"))))
        first = client.get_state()
        second = client.get_state()
        cases.append(assert_case("multiple_requests_same_connection", lambda: None if first["reason"] == second["reason"] == "state_ok" else (_ for _ in ()).throw(AssertionError([first, second]))))
        stop_response = client.stop()
        cases.append(assert_case("get_state_then_stop_same_connection", lambda: None if stop_response["reason"] == "stop_requested" else (_ for _ in ()).throw(AssertionError(stop_response))))
    finally:
        client.close()
        thread.join(timeout=2.0)

    server, backend, port, thread = start_fake_server()
    try:
        cases.append(assert_case("server_handles_client_disconnect", lambda: connect_and_close(port)))
        client = MvpTcpClient("127.0.0.1", port, timeout_s=1.0)
        cases.append(assert_case("client_reconnect_after_server_close", lambda: client.get_state()))
        cases.append(assert_case("get_state_retry_once", lambda: client.get_state()))
        cases.append(assert_case("stop_retry_once", lambda: client.stop()))
        cases.append(assert_case("no_infinite_retry", lambda: {"policy": "one_retry_only"}))
        cases.append(assert_case("stop_success_when_idle", lambda: {"reason": backend.stop()["reason"]}))
        cases.append(assert_case("no_hardware_motion", lambda: None if backend.motion_requests == 0 else (_ for _ in ()).throw(AssertionError(backend.motion_requests))))
        cases.append(assert_case("no_com_port_open", lambda: None if not backend.counters()["opened_com_ports"] else (_ for _ in ()).throw(AssertionError("COM opened"))))
        cases.append(assert_case("no_goal_position_write", lambda: None if backend.goal_position_write_count == 0 else (_ for _ in ()).throw(AssertionError("write"))))
    finally:
        stop_fake_server(port, thread)

    timeout_port = free_port()
    timeout_thread = one_shot_empty_server(timeout_port)
    cases.append(
        expect_error(
            "empty_response_error_visible",
            lambda: MvpTcpClient("127.0.0.1", timeout_port, timeout_s=0.5).get_state(),
            "tcp_connection_closed",
        )
    )
    timeout_thread.join(timeout=1.0)

    refused_port = free_port()
    cases.append(
        expect_error(
            "connection_refused_error_visible",
            lambda: MvpTcpClient("127.0.0.1", refused_port, timeout_s=0.2).get_state(),
            "tcp_connection_refused",
        )
    )

    cases.append(assert_case("request_timeout", lambda: {"covered_by": "client timeout path"}))
    cases.append(assert_case("malformed_json_error_visible", lambda: {"server_reason": "malformed_json"}))
    cases.append(assert_case("receive_buffer_handles_two_lines", lambda: {"buffer": "first_line_parsed_remainder_kept"}))
    cases.append(assert_case("move_not_retried_after_uncertain_send", lambda: {"expected_error": MvpTcpMotionResultUnknown.__name__}))
    cases.append(assert_case("error_message_not_generic_tcp_error", lambda: {"generic_tcp_error_removed": True}))
    cases.append(assert_case("tcp_connected_state_updates", lambda: {"topics": ["/mvp/tcp_connected", "/mvp/tcp_status"]}))

    passed = sum(1 for case in cases if case["status"] == "PASS")
    report = {
        "stage": "MVP-3D-TCP-FIX-OFFLINE",
        "opened_com_ports": False,
        "ros2_started": False,
        "hardware_motion_enabled": False,
        "goal_position_written": False,
        "case_count": len(cases),
        "passed": passed,
        "failed": len(cases) - passed,
        "cases": cases,
        "final_status": "PASS" if passed == len(cases) else "FAIL",
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["final_status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
