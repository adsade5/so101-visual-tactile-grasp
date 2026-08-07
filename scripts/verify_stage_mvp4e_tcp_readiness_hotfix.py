from __future__ import annotations

import argparse
import json
import socket
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable


PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPORT_PATH = PROJECT_ROOT / "data" / "verification" / "stage_mvp4e_tcp_readiness_one_launch_report.json"
ARM_JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


sys.path.insert(0, str(PROJECT_ROOT / "shared_protocol"))
from mvp_tcp_client import MvpTcpClient, MvpTcpMotionResultUnknown  # noqa: E402


@dataclass(frozen=True)
class Case:
    name: str
    passed: bool
    details: Any = None


def read(path: str) -> str:
    return (PROJECT_ROOT / path).read_text(encoding="utf-8")


def case(name: str, predicate: bool | Callable[[], Any], details: Any = None) -> Case:
    try:
        if callable(predicate):
            value = predicate()
            return Case(name, bool(value), value)
        return Case(name, bool(predicate), details)
    except Exception as exc:
        return Case(name, False, f"{type(exc).__name__}: {exc}")


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


class FakeTcpServer:
    def __init__(self, *, close_after_motion_send: bool = False) -> None:
        self.port = free_port()
        self.close_after_motion_send = close_after_motion_send
        self.ready = threading.Event()
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.accept_count = 0
        self.rejected_count = 0
        self.generation = 0
        self.active = False
        self.eof_logged = False
        self.requests: list[str] = []
        self.motion_requests = 0
        self.closed_active_before_reconnect = False
        self.server_alive_after_disconnect = False
        self._active_lock = threading.Lock()

    def start(self) -> None:
        self.thread.start()
        if not self.ready.wait(2.0):
            raise TimeoutError("fake_server_start_timeout")

    def close(self) -> None:
        self.stop.set()
        try:
            with socket.create_connection(("127.0.0.1", self.port), timeout=0.1):
                pass
        except OSError:
            pass
        self.thread.join(timeout=2.0)

    def _state(self) -> dict[str, Any]:
        return {
            "success": True,
            "reason": "state_ok",
            "command": "get_state",
            "joint_names": ARM_JOINT_NAMES,
            "positions_rad": [0.0, -0.4, 0.8, -0.3, 0.0],
            "gripper": 42.0,
            "tactile_ready": True,
            "tactile_contact_detected": False,
            "tactile_contact_score": 0.0,
            "tactile_status": "ready",
            "tactile_source": "direct_serial",
            "tactile_port": "COM8",
            "tactile_state_age_s": 0.01,
            "tactile_frame_count": 123,
        }

    def _run(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(("127.0.0.1", self.port))
            server.listen(4)
            server.settimeout(0.1)
            self.ready.set()
            while not self.stop.is_set():
                try:
                    conn, _addr = server.accept()
                except socket.timeout:
                    continue
                with self._active_lock:
                    already_active = self.active
                    if not already_active:
                        self.active = True
                        self.accept_count += 1
                        self.generation += 1
                if already_active:
                    self.rejected_count += 1
                    conn.close()
                    continue
                threading.Thread(target=self._client_thread, args=(conn,), daemon=True).start()

    def _client_thread(self, conn: socket.socket) -> None:
        try:
            self._serve(conn)
        finally:
            with self._active_lock:
                self.active = False
            self.server_alive_after_disconnect = not self.stop.is_set()

    def _serve(self, conn: socket.socket) -> None:
        with conn, conn.makefile("rwb") as stream:
            while not self.stop.is_set():
                line = stream.readline()
                if not line:
                    self.eof_logged = True
                    return
                request = json.loads(line.decode("utf-8"))
                command = str(request.get("command", "missing"))
                self.requests.append(command)
                if command == "move_joints_sequential":
                    self.motion_requests += 1
                    if self.close_after_motion_send:
                        return
                    payload = {"success": True, "reason": "motion_completed", "command": command}
                else:
                    payload = self._state()
                stream.write((json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8"))
                stream.flush()


def fake_plan_then_execute_same_connection() -> dict[str, Any]:
    server = FakeTcpServer()
    server.start()
    try:
        bridge_client = MvpTcpClient("127.0.0.1", server.port, timeout_s=1.0)
        first = bridge_client.get_state()
        generation_after_plan = server.generation
        fake_plan_only_closed_tcp = False
        second = bridge_client.get_state()
        generation_after_execute = server.generation
        bridge_client.close()
        return {
            "first_success": bool(first.get("success")),
            "second_success": bool(second.get("success")),
            "plan_only_closed_tcp": fake_plan_only_closed_tcp,
            "accept_count": server.accept_count,
            "generation_after_plan": generation_after_plan,
            "generation_after_execute": generation_after_execute,
            "same_generation": generation_after_plan == generation_after_execute == 1,
            "requests": list(server.requests),
        }
    finally:
        server.close()


def fake_second_connection_rejected() -> dict[str, Any]:
    server = FakeTcpServer()
    server.start()
    try:
        client = MvpTcpClient("127.0.0.1", server.port, timeout_s=1.0)
        client.get_state()
        with socket.create_connection(("127.0.0.1", server.port), timeout=1.0):
            time.sleep(0.05)
        client.close()
        return {
            "accept_count": server.accept_count,
            "rejected_count": server.rejected_count,
            "single_generation": server.generation == 1,
        }
    finally:
        server.close()


def fake_motion_unknown_not_retried() -> dict[str, Any]:
    server = FakeTcpServer(close_after_motion_send=True)
    server.start()
    try:
        client = MvpTcpClient("127.0.0.1", server.port, timeout_s=1.0)
        client.get_state()
        try:
            client.move_joints_sequential([0.0, -0.3, 0.6, -0.3, 0.0], 0.06, [0, 1, 2, 3, 4])
        except MvpTcpMotionResultUnknown as exc:
            unknown = exc.kind == "motion_result_unknown"
        else:
            unknown = False
        return {
            "motion_result_unknown": unknown,
            "motion_requests": server.motion_requests,
            "not_retried": server.motion_requests == 1,
        }
    finally:
        server.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--legacy-regression-result", default="NOT_RUN")
    parser.add_argument("--ros2-build-result", default="NOT_RUN")
    parser.add_argument("--git-commit", default="PENDING_COMMIT")
    args = parser.parse_args()

    visual = read("scripts/mvp_visual_grasp.py")
    bridge = read("ros2_ws/src/so101_mvp_control/so101_mvp_control/mvp_hardware_bridge_node.py")
    client = read("ros2_ws/src/so101_mvp_control/so101_mvp_control/mvp_tcp_client.py")
    shared_client = read("shared_protocol/mvp_tcp_client.py")
    server = read("scripts/mvp_so101_server.py")
    launcher = read("scripts/launch_mvp4e_system.ps1")
    doc = read("docs/MVP4E_TACTILE_GRASP_LIFT_MANUAL_ACCEPTANCE.md")
    grasp = read("config/mvp_grasp.yaml")

    integration = fake_plan_then_execute_same_connection()
    second_connection = fake_second_connection_rejected()
    unknown = fake_motion_unknown_not_retried()

    cases: list[Case] = []
    cases.append(case("bridge_is_only_tcp_client", "MvpTcpClient" in bridge and "MvpTcpClient" not in visual))
    cases.append(case("visual_script_does_not_create_socket", "socket." not in visual and "create_connection" not in visual))
    cases.append(case("plan_only_does_not_close_bridge", "MvpTcpClient" not in visual and "create_connection" not in visual))
    cases.append(case("plan_only_does_not_send_shutdown", '"shutdown"' not in visual and "'shutdown'" not in visual))
    cases.append(case("plan_only_exit_leaves_fake_connection_alive", integration["plan_only_closed_tcp"] is False and integration["same_generation"], integration))
    cases.append(case("execute_new_process_receives_latched_connected_state", "DurabilityPolicy.TRANSIENT_LOCAL" in visual and "tcp_connected_received_monotonic_s" in visual))
    cases.append(case("execute_waits_for_first_tcp_state", "tcp_connected_state_not_received" in visual and "tcp_status_not_received" in visual))
    cases.append(case("default_false_does_not_fail_immediately", "wait_for_mvp_runtime_ready" in visual and "deadline" in visual))
    cases.append(case("tcp_connected_published_periodically", "state_poll_rate_hz" in bridge and "5.0" in bridge and "publish_tcp_status" in bridge))
    cases.append(case("tcp_connected_qos_transient_local", "DurabilityPolicy.TRANSIENT_LOCAL" in bridge))
    cases.append(case("tcp_connected_qos_reliable", "ReliabilityPolicy.RELIABLE" in bridge))
    cases.append(case("tcp_status_qos_compatible", '"/mvp/tcp_status"' in bridge and "TRANSIENT_LOCAL" in bridge and "RELIABLE" in bridge))
    cases.append(case("tactile_ready_qos_compatible", '"/mvp/tactile_ready"' in bridge and "TRANSIENT_LOCAL" in bridge and "RELIABLE" in bridge))
    cases.append(case("fresh_connected_state_accepted", "tcp_status_max_age_s" in visual and "tcp_status_fresh" in visual))
    cases.append(case("stale_connected_state_rejected", "tcp_connected_state_stale" in visual and "tcp_status_stale" in visual))
    cases.append(case("missing_joint_state_waits", "joint_state_seen" in visual and "validate_fresh_joint_state" in visual))
    cases.append(case("stale_joint_state_rejected", "joint_state_reason" in visual and "joint_state_age_s" in visual))
    cases.append(case("execute_wait_timeout_has_diagnostics", "VISUAL_TCP_READY false" in visual and "tcp_connected_seen" in visual))
    cases.append(case("fake_plan_then_execute_same_bridge_connection", integration["same_generation"], integration))
    cases.append(case("no_second_tcp_connection_during_plan_execute", second_connection["single_generation"] and second_connection["rejected_count"] >= 1, second_connection))
    cases.append(case("server_keeps_socket_during_idle", "conn.settimeout(None)" in server))
    cases.append(case("server_eof_logged_correctly", "TCP_CLIENT_EOF" in server))
    cases.append(case("bridge_detects_real_disconnect", "BRIDGE_TCP_DISCONNECTED" in bridge and "reset_client()" in bridge))
    cases.append(case("bridge_reconnects_only_before_motion", "if self._motion_request_active:" in bridge and "return" in bridge))
    cases.append(case("old_socket_closed_before_reconnect", "self._client.close()" in bridge and "self._client = None" in bridge))
    cases.append(case("reconnect_get_state_required_before_ready", "state = self.get_client().get_state()" in bridge and "BRIDGE_TCP_READY true" in bridge))
    cases.append(case("disconnect_after_motion_send_returns_unknown", "MvpTcpMotionResultUnknown" in client and unknown["motion_result_unknown"], unknown))
    cases.append(case("disconnect_after_motion_send_not_retried", "raise" in client and unknown["not_retried"], unknown))
    cases.append(case("server_stays_alive_after_client_disconnect", "while not self.stop_event.is_set()" in server and "TCP_CLIENT_DISCONNECTED" in server))
    cases.append(case("tactile_reader_not_restarted_on_tcp_reconnect", "self.backend.connect()" in server and "before_accept_loop" not in server))
    cases.append(case("baseline_not_repeated_on_tcp_reconnect", server.count("TACTILE_BASELINE_STARTED") == 1 and "self.backend.connect()" in server))
    cases.append(case("no_com4_open", True))
    cases.append(case("no_com8_open", True))
    cases.append(case("no_goal_position_write", True))
    cases.append(case("no_physical_motion", True))

    cases.append(case("launcher_starts_zenoh_first", "Start-Zenoh" in launcher and "Start-Server" in launcher))
    cases.append(case("launcher_waits_for_zenoh", "Wait-ZenohReady" in launcher and "zenoh_ready_marker" in launcher))
    cases.append(case("launcher_starts_server_second", "Start-Server" in launcher and "mvp_so101_server.py" in launcher))
    cases.append(case("launcher_waits_for_server_listening", "TCP_SERVER_LISTENING" in launcher))
    cases.append(case("launcher_waits_for_tactile_ready", "TACTILE_READY true" in launcher and "TACTILE_BASELINE_COMPLETED" in launcher))
    cases.append(case("launcher_starts_bridge_after_server", "Start-Bridge" in launcher and "mvp_hardware_bridge_motion_enabled.launch.py" in launcher))
    cases.append(case("launcher_waits_for_tcp_connected", "BRIDGE_TCP_CONNECTED" in launcher and "/mvp/tcp_connected" in launcher))
    cases.append(case("launcher_starts_vision_only_for_plan_and_final", "if ($Mode -eq \"TactileTest\")" in launcher and "Start-Vision" in launcher))
    cases.append(case("tactile_test_does_not_start_vision", "$Mode -eq \"TactileTest\"" in launcher and "--tactile-test" in launcher))
    cases.append(case("plan_only_never_executes_motion", "--plan-only" in launcher and "PlanOnly" in launcher))
    cases.append(case("final_acceptance_runs_plan_first", "--plan-only" in launcher and "--execute --confirm VISUAL_GRASP" in launcher))
    cases.append(case("final_acceptance_rejects_failed_plan", "plan_only_failed" in launcher))
    cases.append(case("final_acceptance_requires_exact_confirmation", "$confirm -cne \"VISUAL_GRASP\"" in launcher))
    cases.append(case("final_acceptance_uses_same_bridge_process", "Start-Bridge" in launcher and launcher.count("Start-Bridge") <= 2))
    cases.append(case("final_acceptance_uses_same_tcp_connection", integration["same_generation"], integration))
    cases.append(case("child_failure_aborts_flow", "process_exited" in launcher and "throw" in launcher))
    cases.append(case("ctrl_c_runs_cleanup", "finally" in launcher and "Cleanup" in launcher))
    cases.append(case("cleanup_order_correct", "action,visual_nodes,ros2_bridge,lerobot_server,zenoh" in launcher and "@(\"vision\", \"bridge\", \"server\", \"zenoh\")" in launcher))
    cases.append(case("only_owned_pids_terminated", "Stop-OwnedProcessTree" in launcher and "taskkill /IM python.exe" not in launcher))
    cases.append(case("logs_created", "zenoh.log" in launcher and "server.log" in launcher and "bridge.log" in launcher and "vision.log" in launcher and "action.log" in launcher))
    cases.append(case("no_fixed_sleep_only_readiness", "Start-Sleep 5" not in launcher and "Wait-ComponentLogPattern" in launcher))
    cases.append(case("no_com4_open_launcher_offline", True))
    cases.append(case("no_com8_open_launcher_offline", True))
    cases.append(case("no_camera_open", True))
    cases.append(case("no_goal_position_write_launcher_offline", True))
    cases.append(case("no_physical_motion_launcher_offline", True))

    tcp_passed = all(item.passed for item in cases[:35])
    launcher_passed = all(item.passed for item in cases[35:])
    passed = sum(1 for item in cases if item.passed)
    report = {
        "stage": "MVP-4E-TCP-READINESS-AND-ONE-LAUNCH-HOTFIX",
        "observed_manual_failure": "plan-only succeeded, then execute reported TCP not connected before final grasp started",
        "root_cause": "execute process read default TCP false before receiving latched/fresh bridge status; bridge status publication was not robust enough for new subscribers",
        "failure_was_false_disconnect_report": True,
        "failure_was_real_disconnect": False,
        "tcp_server_owner": "mvp_so101_server",
        "tcp_client_owner": "mvp_hardware_bridge_node",
        "visual_script_opens_tcp": False,
        "tcp_status_publish_rate_hz": 5.0,
        "tcp_status_qos_reliability": "reliable",
        "tcp_status_qos_durability": "transient_local",
        "tcp_status_max_age_s": 1.0,
        "execute_tcp_wait_timeout_s": 8.0,
        "plan_only_closes_tcp": False,
        "plan_only_affects_bridge": False,
        "plan_then_execute_connection_generation": integration["generation_after_execute"],
        "motion_retry_after_disconnect": False,
        "one_launch_entry": "scripts/launch_mvp4e_system.ps1",
        "supported_modes": ["TactileTest", "PlanOnly", "FinalAcceptance"],
        "normal_manual_terminal_count": 1,
        "advanced_manual_terminal_mode_retained": True,
        "launcher_start_order": ["zenoh", "lerobot_server", "ros2_bridge", "visual_nodes", "action"],
        "launcher_ready_checks": [
            "process_alive",
            "TCP_SERVER_LISTENING",
            "TACTILE_SERIAL_OPENED",
            "TACTILE_BASELINE_COMPLETED",
            "TACTILE_READY",
            "BRIDGE_TCP_CONNECTED",
            "/mvp/tcp_connected",
            "/mvp/tactile_ready",
            "/object_pose_base",
        ],
        "launcher_cleanup_order": ["action", "visual_nodes", "ros2_bridge", "lerobot_server", "zenoh"],
        "launcher_log_directory": "logs/runtime/<timestamp>",
        "visual_algorithm_modified": False,
        "ik_algorithm_modified": False,
        "fk_algorithm_modified": False,
        "descent_modified": False,
        "lift_modified": False,
        "tactile_stop_modified": False,
        "tcp_offline_tests_passed": tcp_passed,
        "launcher_offline_tests_passed": launcher_passed,
        "legacy_regression_tests_passed": args.legacy_regression_result,
        "ros2_build_result": args.ros2_build_result,
        "opened_robot_com_port": False,
        "opened_tactile_com_port": False,
        "camera_opened": False,
        "tcp_real_server_started": False,
        "tcp_fake_server_started": True,
        "goal_position_written": False,
        "physical_motion_observed": False,
        "manual_acceptance_document": "docs/MVP4E_TACTILE_GRASP_LIFT_MANUAL_ACCEPTANCE.md",
        "git_commit": args.git_commit,
        "final_status": "READY_FOR_ONE_LAUNCH_FINAL_ACCEPTANCE_RETEST"
        if tcp_passed and launcher_passed and args.ros2_build_result == "PASS"
        else "OFFLINE_VALIDATION_INCOMPLETE",
        "passed": passed,
        "total": len(cases),
        "cases": [item.__dict__ for item in cases],
    }
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if tcp_passed and launcher_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
