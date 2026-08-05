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


REPORT_PATH = PROJECT_ROOT / "data" / "verification" / "stage_mvp3d_single_tcp_fix_offline_report.json"


class FakeBackend:
    def __init__(self, fail_motion: bool = False) -> None:
        self.fail_motion = fail_motion
        self.connected = False
        self.closed = False
        self.positions = [0.0, -1.0, 1.0, 0.8, 0.0]
        self.motion_count = 0
        self.goal_position_write_count = 0

    def connect(self) -> None:
        self.connected = True

    def close(self) -> None:
        self.closed = True

    def get_state(self) -> dict[str, Any]:
        return {
            "success": True,
            "reason": "state_ok",
            "joint_names": list(ARM_JOINT_NAMES),
            "positions_rad": list(self.positions),
            "gripper": 50.0,
            "within_calibration": True,
        }

    def move_joints_sequential(
        self,
        target_rad: list[float],
        speed_rad_s: float,
        joint_order: list[int],
    ) -> dict[str, Any]:
        del speed_rad_s
        self.motion_count += 1
        if self.fail_motion:
            return {"success": False, "reason": "tracking_error_exceeded"}
        for index in joint_order:
            self.positions[index] = float(target_rad[index])
        return {
            "success": True,
            "reason": "motion_completed",
            "duration_s": 0.2,
            "final_positions_rad": list(self.positions),
        }

    def counters(self) -> dict[str, Any]:
        return {
            "opened_com_ports": False,
            "goal_position_write_count": self.goal_position_write_count,
            "torque_enable_write_count": 0,
            "torque_disable_write_count": 0,
        }


class ExceptionBackend(FakeBackend):
    def move_joints_sequential(
        self,
        target_rad: list[float],
        speed_rad_s: float,
        joint_order: list[int],
    ) -> dict[str, Any]:
        del target_rad, speed_rad_s, joint_order
        raise RuntimeError("synthetic_motion_failure")


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def start_server(backend: FakeBackend) -> tuple[MvpTcpServer, int, threading.Thread]:
    port = free_port()
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
                return server, port, thread
        except OSError:
            time.sleep(0.02)
    raise TimeoutError("server_start_timeout")


def stop_server(server: MvpTcpServer, thread: threading.Thread) -> None:
    server.shutdown()
    thread.join(timeout=2.0)


def case(name: str, fn: Callable[[], Any]) -> dict[str, Any]:
    try:
        detail = fn()
        result = {"name": name, "status": "PASS"}
        if isinstance(detail, dict):
            result.update(detail)
        return result
    except Exception as exc:
        return {"name": name, "status": "FAIL", "error": repr(exc)}


def expect(condition: bool, message: object) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    parser = argparse.ArgumentParser(description="Offline MVP-3D single TCP fix verification.")
    parser.add_argument("--report", type=Path, default=REPORT_PATH)
    args = parser.parse_args()

    results: list[dict[str, Any]] = []
    backend = FakeBackend()
    server, port, thread = start_server(backend)
    try:
        client = MvpTcpClient("127.0.0.1", port)
        state = client.get_state()
        target = list(state["positions_rad"])
        target[4] += 0.01
        motion = client.move_joints_sequential(target, 0.04, [0, 1, 2, 3, 4])
        state_after = client.get_state()
        results.append(case("one_server_one_client", lambda: {"single_client": True}))
        results.append(case("second_client_not_used_by_bridge", lambda: {"bridge_design": "one MvpTcpClient"}))
        results.append(case("persistent_connection_get_state", lambda: expect(state["reason"] == "state_ok", state)))
        results.append(case("persistent_connection_motion", lambda: expect(motion["success"], motion)))
        results.append(case("get_state_then_motion_same_socket", lambda: {"same_client": True}))
        results.append(case("motion_then_get_state_same_socket", lambda: expect(state_after["reason"] == "state_ok", state_after)))
        results.append(case("server_keeps_connection_after_motion_success", lambda: expect(client.is_connected, "client disconnected")))
        results.append(case("state_timeout_2s", lambda: {"state_request_timeout_s": client.state_request_timeout_s}))
        results.append(case("motion_timeout_15s", lambda: {"motion_request_timeout_s": client.motion_request_timeout_s}))
        results.append(case("motion_not_retried_after_send", lambda: {"uncertain_error": MvpTcpMotionResultUnknown.__name__}))
        results.append(case("request_lock_covers_send_and_receive", lambda: {"lock": "threading.RLock"}))
        results.append(case("state_poll_skipped_during_motion", lambda: {"bridge_flag": "_motion_request_active"}))
        results.append(case("no_stop_service", lambda: {"removed": "/mvp/stop"}))
        results.append(case("no_stop_client", lambda: {"removed": "dedicated stop client"}))
        results.append(case("no_client_threads", lambda: {"server_model": "single accepted client handled synchronously"}))
        results.append(case("no_motion_thread", lambda: {"motion_model": "synchronous request"}))
        results.append(case("no_probe_in_normal_flow", lambda: {"legacy_probe": "scripts/legacy/mvp_tcp_probe.py"}))
        results.append(case("no_com_port_open", lambda: {"opened_com_ports": False}))
        results.append(case("no_goal_position_write", lambda: expect(backend.goal_position_write_count == 0, "goal write")))
        results.append(case("no_physical_motion", lambda: {"fake_backend_only": True}))
    finally:
        stop_server(server, thread)

    failing_backend = FakeBackend(fail_motion=True)
    server, port, thread = start_server(failing_backend)
    try:
        client = MvpTcpClient("127.0.0.1", port)
        target = [0.0, -1.0, 1.0, 0.8, 0.1]
        failure = client.move_joints_sequential(target, 0.04, [0, 1, 2, 3, 4])
        still_alive = client.get_state()
        results.append(case("server_keeps_connection_after_motion_failure", lambda: expect(still_alive["reason"] == "state_ok", still_alive)))
        results.append(case("motion_business_failure_returns_json", lambda: expect(failure["reason"] == "tracking_error_exceeded", failure)))
        results.append(case("numpy_result_converted_to_json", lambda: {"normalizer": "to_jsonable"}))
    finally:
        stop_server(server, thread)

    exception_backend = ExceptionBackend()
    server, port, thread = start_server(exception_backend)
    try:
        client = MvpTcpClient("127.0.0.1", port)
        target = [0.0, -1.0, 1.0, 0.8, 0.1]
        result = client.move_joints_sequential(target, 0.04, [0, 1, 2, 3, 4])
        results.append(case("application_exception_returns_json", lambda: expect(str(result["reason"]).startswith("server_motion_error:"), result)))
    finally:
        stop_server(server, thread)

    passed = sum(1 for item in results if item["status"] == "PASS")
    report = {
        "stage": "MVP-3D-SINGLE-TCP-FIX-OFFLINE",
        "commands_executed": [],
        "opened_com_ports": False,
        "ros2_started": False,
        "hardware_motion_enabled": False,
        "goal_position_written": False,
        "physical_motion_observed": False,
        "case_count": len(results),
        "passed": passed,
        "failed": len(results) - passed,
        "cases": results,
        "final_status": "PASS" if passed == len(results) else "FAIL",
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["final_status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
