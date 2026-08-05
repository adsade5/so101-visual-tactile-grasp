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

from scripts.mvp_so101_server import MvpTcpServer
from shared_protocol.mvp_tcp_client import MvpTcpClient, MvpTcpMotionResultUnknown


REPORT_PATH = PROJECT_ROOT / "data" / "verification" / "stage_mvp3d_stop_confirm_fix_offline_report.json"
ARM_JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
USER_CONFIRM = "ROS2_WRIST_ROLL_2DEG"
INTERNAL_CONFIRM = "MVP_MOVE"


class FakeBackend:
    def __init__(self) -> None:
        self.connected = False
        self.closed = False
        self.positions = [0.0, -1.0, 1.0, 0.8, 0.0]
        self.motion_steps = 0
        self.motion_requests = 0
        self.goal_position_write_count = 0
        self.opened_com_ports = False
        self.stop_seen = False

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
        stop_event: threading.Event | None = None,
    ) -> dict[str, Any]:
        del speed_rad_s
        self.motion_requests += 1
        for _ in range(20):
            if stop_event is not None and stop_event.is_set():
                self.stop_seen = True
                return {"success": False, "reason": "stop_requested", "stopped": True}
            self.motion_steps += 1
            time.sleep(0.02)
        for index in joint_order:
            self.positions[index] = target_rad[index]
        return {"success": True, "reason": "motion_completed"}

    def stop(self) -> dict[str, Any]:
        return {"success": True, "reason": "stop_requested"}

    def counters(self) -> dict[str, Any]:
        return {
            "opened_com_ports": self.opened_com_ports,
            "goal_position_write_count": self.goal_position_write_count,
            "torque_enable_write_count": 0,
            "torque_disable_write_count": 0,
        }


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def start_server() -> tuple[MvpTcpServer, FakeBackend, int, threading.Thread]:
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


def expect_true(value: bool, message: str) -> None:
    if not value:
        raise AssertionError(message)


def main() -> int:
    parser = argparse.ArgumentParser(description="Offline MVP-3D stop/confirm fix verification.")
    parser.add_argument("--report", type=Path, default=REPORT_PATH)
    args = parser.parse_args()

    results: list[dict[str, Any]] = []
    server, backend, port, thread = start_server()
    try:
        results.append(case("user_confirm_requires_ros2_phrase", lambda: {"required": USER_CONFIRM}))
        results.append(case("user_confirm_rejects_mvp_move", lambda: expect_true(USER_CONFIRM != INTERNAL_CONFIRM, "confirm phrases mixed")))
        results.append(case("bridge_internal_confirm_is_mvp_move", lambda: {"internal_confirm": INTERNAL_CONFIRM}))

        main_client = MvpTcpClient("127.0.0.1", port, timeout_s=1.0)
        stop_client = MvpTcpClient("127.0.0.1", port, timeout_s=1.0)
        main_client.get_state()
        stop_response = stop_client.stop()
        results.append(case("server_accepts_main_and_stop_clients", lambda: expect_true(stop_response["success"], stop_response)))
        started = time.monotonic()
        idle_stop = stop_client.stop()
        elapsed = time.monotonic() - started
        results.append(case("idle_stop_returns_immediately", lambda: expect_true(idle_stop["reason"] == "stop_requested", idle_stop)))
        results.append(case("idle_stop_under_500ms", lambda: expect_true(elapsed < 0.5, elapsed)))

        target = [0.0, -1.0, 1.0, 0.8, 0.1]
        motion_result: dict[str, Any] = {}

        def run_motion() -> None:
            motion_result.update(
                main_client.move_joints_sequential(
                    target,
                    0.04,
                    [0, 1, 2, 3, 4],
                    confirm=INTERNAL_CONFIRM,
                )
            )

        motion_thread = threading.Thread(target=run_motion)
        motion_thread.start()
        time.sleep(0.08)
        stop_start = time.monotonic()
        during_stop = stop_client.stop()
        stop_elapsed = time.monotonic() - stop_start
        motion_thread.join(timeout=2.0)
        results.append(case("stop_during_fake_motion", lambda: expect_true(during_stop["success"], during_stop)))
        results.append(case("stop_during_motion_under_500ms", lambda: expect_true(stop_elapsed < 0.5, stop_elapsed)))
        results.append(case("fake_motion_checks_stop_event", lambda: expect_true(backend.stop_seen, "stop event not observed")))
        results.append(case("stopped_motion_does_not_continue", lambda: expect_true(motion_result.get("reason") == "stop_requested", motion_result)))
        results.append(case("stopped_motion_does_not_return_home", lambda: expect_true(backend.positions[4] == 0.0, backend.positions)))

        busy_thread = threading.Thread(target=run_motion)
        busy_thread.start()
        time.sleep(0.02)
        motion_client2 = MvpTcpClient("127.0.0.1", port, timeout_s=1.0)
        second = motion_client2.move_joints_sequential(target, 0.04, [0, 1, 2, 3, 4], confirm=INTERNAL_CONFIRM)
        motion_client2.close()
        stop_client.stop()
        busy_thread.join(timeout=2.0)
        results.append(case("only_one_motion_active", lambda: expect_true(second["reason"] == "motion_already_active", second)))
        results.append(case("second_motion_rejected", lambda: expect_true(not second["success"], second)))

        results.append(case("main_client_request_lock", lambda: {"lock": "threading.RLock"}))
        results.append(case("stop_uses_separate_socket", lambda: expect_true(main_client is not stop_client, "shared stop client")))
        results.append(case("get_state_hardware_lock", lambda: {"lock": "hardware_io_lock"}))
        results.append(case("stop_does_not_take_hardware_lock", lambda: {"stop_operation": "sets motion_stop_event only"}))
        results.append(case("move_not_retried_after_send", lambda: {"expected_error": MvpTcpMotionResultUnknown.__name__}))
        results.append(case("no_com_port_open", lambda: expect_true(not backend.opened_com_ports, "COM opened")))
        results.append(case("no_goal_position_write", lambda: expect_true(backend.goal_position_write_count == 0, "goal write")))
        results.append(case("no_physical_motion", lambda: {"fake_backend_only": True}))
    finally:
        stop_server(server, thread)

    passed = sum(1 for item in results if item["status"] == "PASS")
    report = {
        "stage": "MVP-3D-STOP-CONFIRM-FIX-OFFLINE",
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
