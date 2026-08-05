from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import threading
import time
from pathlib import Path
from typing import Any, Callable

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from lerobot_server.mvp_hardware_executor import ARM_JOINT_NAMES, MvpSo101HardwareExecutor
from scripts.mvp_so101_server import MvpTcpServer
from shared_protocol.mvp_tcp_client import MvpTcpClient


REPORT_PATH = PROJECT_ROOT / "data" / "verification" / "stage_mvp3c_tcp_bridge_report.json"
CONFIG_PATH = PROJECT_ROOT / "config" / "mvp_hardware.json"


class FakeBackend:
    def __init__(self, config_path: Path) -> None:
        self.executor = MvpSo101HardwareExecutor(config_path)
        self.positions_rad = [-0.162641670955076, -1.3525342730839574, 1.06714417121939, 1.1929613129016186, -0.02531686387508258]
        self.gripper = 46.92513368983957
        self.connected = False
        self.closed = False
        self.motion_requests: list[dict[str, Any]] = []
        self.stop_requested = False

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
            "gripper": float(self.gripper),
            "within_calibration": True,
        }

    def move_joints_sequential(
        self,
        target_rad: list[float],
        speed_rad_s: float,
        joint_order: list[int],
    ) -> dict[str, Any]:
        self.motion_requests.append(
            {
                "target_rad": list(target_rad),
                "speed_rad_s": float(speed_rad_s),
                "joint_order": list(joint_order),
            }
        )
        for index in joint_order:
            self.positions_rad[index] = float(target_rad[index])
        return {"success": True, "reason": "motion_completed"}

    def stop(self) -> dict[str, Any]:
        self.stop_requested = True
        return {"success": True, "reason": "stop_requested"}


def find_free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def start_server(*, motion_enabled: bool) -> tuple[MvpTcpServer, FakeBackend, int, threading.Thread]:
    port = find_free_port()
    backend = FakeBackend(CONFIG_PATH)
    server = MvpTcpServer(
        host="127.0.0.1",
        port=port,
        backend=backend,
        hardware_motion_enabled=motion_enabled,
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
    raise TimeoutError("fake TCP server did not start")


def stop_server(port: int, thread: threading.Thread) -> None:
    try:
        with MvpTcpClient("127.0.0.1", port, timeout_s=1.0) as client:
            client.stop()
    except Exception:
        pass
    thread.join(timeout=2.0)


def contains_key(value: Any, key: str) -> bool:
    if isinstance(value, dict):
        return key in value or any(contains_key(item, key) for item in value.values())
    if isinstance(value, list):
        return any(contains_key(item, key) for item in value)
    return False


def request_raw(port: int, line: str, timeout_s: float = 1.0) -> dict[str, Any]:
    with socket.create_connection(("127.0.0.1", port), timeout=timeout_s) as sock:
        sock.settimeout(timeout_s)
        sock.sendall(line.encode("utf-8"))
        data = sock.recv(4096)
    return json.loads(data.decode("utf-8").strip())


def assert_case(name: str, fn: Callable[[], dict[str, Any] | None]) -> dict[str, Any]:
    try:
        detail = fn() or {}
        return {"name": name, "status": "PASS", **detail}
    except Exception as exc:
        return {"name": name, "status": "FAIL", "error": repr(exc)}


def main() -> int:
    parser = argparse.ArgumentParser(description="Offline MVP-3C TCP bridge protocol verification.")
    parser.add_argument("--report", type=Path, default=REPORT_PATH)
    args = parser.parse_args()

    cases: list[dict[str, Any]] = []
    ro_server, ro_backend, ro_port, ro_thread = start_server(motion_enabled=False)
    try:
        with MvpTcpClient("127.0.0.1", ro_port, timeout_s=1.0) as client:
            state = client.get_state()
            cases.append(assert_case("get_state_success", lambda: {"reason": state["reason"]} if state["success"] else (_ for _ in ()).throw(AssertionError(state))))
            cases.append(assert_case("get_state_joint_order", lambda: None if state["joint_names"] == ARM_JOINT_NAMES else (_ for _ in ()).throw(AssertionError(state["joint_names"]))))
            cases.append(assert_case("get_state_units_rad", lambda: None if all(abs(v) < 4.0 for v in state["positions_rad"]) else (_ for _ in ()).throw(AssertionError(state["positions_rad"]))))
            cases.append(assert_case("gripper_separate", lambda: None if "gripper" in state and "gripper" not in state["joint_names"] else (_ for _ in ()).throw(AssertionError(state))))
            cases.append(assert_case("read_only_move_rejected", lambda: None if not client.move_joints_sequential(ro_backend.positions_rad, 0.04, [0, 1, 2, 3, 4])["success"] else (_ for _ in ()).throw(AssertionError("move accepted"))))
            response_one = client.get_state()
            response_two = client.get_state()
            cases.append(assert_case("one_request_one_response", lambda: None if response_one["reason"] == "state_ok" and response_two["reason"] == "state_ok" else (_ for _ in ()).throw(AssertionError([response_one, response_two]))))
            cases.append(assert_case("no_plan_id_fields", lambda: None if not contains_key(response_one, "plan_id") else (_ for _ in ()).throw(AssertionError(response_one))))
            cases.append(assert_case("no_trajectory_hash_fields", lambda: None if not contains_key(response_one, "trajectory_hash") else (_ for _ in ()).throw(AssertionError(response_one))))
        cases.append(assert_case("malformed_json_rejected", lambda: None if request_raw(ro_port, "{bad json\n")["reason"] == "malformed_json" else (_ for _ in ()).throw(AssertionError("malformed accepted"))))
        cases.append(assert_case("unknown_command_rejected", lambda: None if request_raw(ro_port, "{\"command\":\"bogus\"}\n")["reason"] == "unknown_command" else (_ for _ in ()).throw(AssertionError("unknown accepted"))))

        def idle_has_no_heartbeat() -> None:
            with socket.create_connection(("127.0.0.1", ro_port), timeout=1.0) as sock:
                sock.settimeout(0.25)
                try:
                    data = sock.recv(1)
                except socket.timeout:
                    return
                raise AssertionError(data)

        cases.append(assert_case("no_heartbeat_messages", lambda: idle_has_no_heartbeat()))
    finally:
        stop_server(ro_port, ro_thread)

    motion_server, motion_backend, motion_port, motion_thread = start_server(motion_enabled=True)
    try:
        with MvpTcpClient("127.0.0.1", motion_port, timeout_s=1.0) as client:
            base = list(motion_backend.positions_rad)
            cases.append(assert_case("motion_confirmation_missing", lambda: None if client.request({"command": "move_joints_sequential", "target_rad": base, "speed_rad_s": 0.04, "joint_order": [0, 1, 2, 3, 4]})["reason"] == "confirmation_missing" else (_ for _ in ()).throw(AssertionError("missing confirmation accepted"))))
            cases.append(assert_case("motion_wrong_confirmation", lambda: None if not client.move_joints_sequential(base, 0.04, [0, 1, 2, 3, 4], confirm="WRONG")["success"] else (_ for _ in ()).throw(AssertionError("wrong confirmation accepted"))))
            cases.append(assert_case("invalid_target_length", lambda: None if client.move_joints_sequential(base[:4], 0.04, [0, 1, 2, 3, 4])["reason"] == "invalid_target_length" else (_ for _ in ()).throw(AssertionError("bad length accepted"))))
            non_finite = list(base)
            non_finite[0] = math.nan
            cases.append(assert_case("non_finite_target", lambda: None if client.move_joints_sequential(non_finite, 0.04, [0, 1, 2, 3, 4])["reason"] == "non_finite_target" else (_ for _ in ()).throw(AssertionError("nan accepted"))))
            out_of_range = list(base)
            out_of_range[0] = 999.0
            cases.append(assert_case("calibration_out_of_range", lambda: None if client.move_joints_sequential(out_of_range, 0.04, [0, 1, 2, 3, 4])["reason"] == "calibration_out_of_range" else (_ for _ in ()).throw(AssertionError("out-of-range accepted"))))
            cases.append(assert_case("speed_above_limit", lambda: None if client.move_joints_sequential(base, 0.081, [0, 1, 2, 3, 4])["reason"] == "speed_above_limit" else (_ for _ in ()).throw(AssertionError("fast accepted"))))
            cases.append(assert_case("invalid_joint_order", lambda: None if client.move_joints_sequential(base, 0.04, [0, 1, 2, 3, 3])["reason"] == "invalid_joint_order" else (_ for _ in ()).throw(AssertionError("bad order accepted"))))
            target = list(base)
            target[4] += math.radians(0.5)
            cases.append(assert_case("valid_motion_request_reaches_fake_executor", lambda: None if client.move_joints_sequential(target, 0.04, [0, 1, 2, 3, 4])["success"] and len(motion_backend.motion_requests) == 1 else (_ for _ in ()).throw(AssertionError(motion_backend.motion_requests))))
            cases.append(assert_case("stop_request", lambda: None if client.stop()["reason"] == "stop_requested" else (_ for _ in ()).throw(AssertionError("stop failed"))))
    finally:
        motion_thread.join(timeout=2.0)

    cases.append(assert_case("client_timeout", lambda: None if raises_connection_error() else (_ for _ in ()).throw(AssertionError("unexpected connection"))))
    cases.append(assert_case("disconnect_no_automatic_motion", lambda: None if ro_backend.closed and len(ro_backend.motion_requests) == 0 else (_ for _ in ()).throw(AssertionError("motion on disconnect"))))

    passed = sum(1 for case in cases if case["status"] == "PASS")
    report = {
        "stage": "MVP-3C",
        "scope": "offline_tcp_protocol_and_bridge_readiness",
        "opens_com4": False,
        "sends_real_motion": False,
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


def raises_connection_error() -> bool:
    port = find_free_port()
    try:
        with MvpTcpClient("127.0.0.1", port, timeout_s=0.1) as client:
            client.get_state()
    except OSError:
        return True
    return False


if __name__ == "__main__":
    raise SystemExit(main())
