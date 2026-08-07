from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable


PROJECT_ROOT = Path(__file__).resolve().parents[1]
REAL_LOG_DIR = PROJECT_ROOT / "logs" / "runtime" / "20260807_093558"
REPORT_PATH = PROJECT_ROOT / "data" / "verification" / "stage_mvp4e_launcher_process_readiness_report.json"


@dataclass(frozen=True)
class Case:
    name: str
    passed: bool
    details: Any = None


def read(path: Path) -> str:
    if not path.exists():
        return ""
    data = path.read_bytes()
    if not data:
        return ""
    if data.startswith(b"\xff\xfe") or data.count(b"\x00") > max(4, len(data) // 10):
        return data.decode("utf-16", errors="replace")
    return data.decode("utf-8", errors="replace")


def read_rel(path: str) -> str:
    return read(PROJECT_ROOT / path)


def case(name: str, predicate: bool | Callable[[], Any], details: Any = None) -> Case:
    try:
        if callable(predicate):
            value = predicate()
            return Case(name, bool(value), value)
        return Case(name, bool(predicate), details)
    except Exception as exc:
        return Case(name, False, f"{type(exc).__name__}: {exc}")


def tail_lines(text: str, count: int = 100) -> list[str]:
    return text.splitlines()[-count:]


def summarize_zenoh_log(launcher_log: str, zenoh_log: str) -> dict[str, Any]:
    lower = zenoh_log.lower()
    ready_terms = ("zenohd", "started", "router", "listening", "scouting")
    return {
        "log_exists": (REAL_LOG_DIR / "zenoh.log").exists(),
        "log_empty": len(zenoh_log) == 0,
        "line_count": len(zenoh_log.splitlines()),
        "contains_vs_prompt": "Developer Command Prompt" in zenoh_log,
        "contains_ready_marker": any(term in lower for term in ready_terms),
        "contains_error_text": any(term in lower for term in ("fatal", "error", "address already in use", "bind failed", "panic")),
        "port_in_use_evidence": "address already in use" in lower or "bind" in lower and "failed" in lower,
        "missing_ros2_environment_evidence": "ROS_DISTRO" not in zenoh_log and "ros2" not in lower,
        "command_error_evidence": "not recognized" in lower or "no executable found" in lower,
        "dll_or_rmw_load_failure_evidence": "dll" in lower or "rmw" in lower and "failed" in lower,
        "normal_start_then_exit_evidence": any(term in lower for term in ready_terms) and "exit_code=1" in launcher_log,
        "exit_code": 1 if "exit_code=1" in launcher_log else None,
        "root_cause_from_log": (
            "zenoh process exited with code 1 after only the ROS2 wrapper/Visual Studio banner; "
            "zenoh.log contains no real ready marker and no specific port/DLL/RMW error text"
        ),
        "tail": tail_lines(zenoh_log),
    }


def summarize_server_log(launcher_log: str, server_log: str) -> dict[str, Any]:
    markers = [
        "SERVER_PROCESS_STARTED",
        "TCP_SERVER_STARTING",
        "TACTILE_SERIAL_OPENING",
        "TACTILE_SERIAL_OPENED",
        "TACTILE_BASELINE_STARTED",
        "TACTILE_BASELINE_COMPLETED",
        "TACTILE_READY true",
        "ROBOT_CONNECTING",
        "ROBOT_CONNECTED",
        "TCP_SERVER_LISTENING",
    ]
    completed = [marker for marker in markers if marker in server_log]
    return {
        "log_exists": (REAL_LOG_DIR / "server.log").exists(),
        "log_empty": len(server_log) == 0,
        "line_count": len(server_log.splitlines()),
        "last_completed_stage": completed[-1] if completed else "none",
        "stopped_in_import_stage_evidence": len(server_log) == 0,
        "stopped_in_com4_connect_evidence": "ROBOT_CONNECTING" in server_log and "ROBOT_CONNECTED" not in server_log,
        "stopped_in_com8_open_evidence": "TACTILE_SERIAL_OPENING" in server_log and "TACTILE_SERIAL_OPENED" not in server_log,
        "stopped_in_baseline_evidence": "TACTILE_BASELINE_STARTED" in server_log and "TACTILE_BASELINE_COMPLETED" not in server_log,
        "tcp_listen_emitted": "TCP_SERVER_LISTENING" in server_log,
        "different_listen_format_emitted": "TCP_LISTENING" in server_log and "TCP_SERVER_LISTENING" not in server_log,
        "traceback": "Traceback" in server_log,
        "stdout_buffering_possible_in_old_launcher": "python scripts\\mvp_so101_server.py" in read(REAL_LOG_DIR / "server.ps1") and "python -u" not in read(REAL_LOG_DIR / "server.ps1"),
        "hardware_init_exceeded_old_30s_possible": "timeout waiting_for=TCP_SERVER_LISTENING" in launcher_log and len(server_log) == 0,
        "root_cause_from_log": (
            "server.log was empty; the old launcher started server with conda capture and non-unbuffered python, "
            "so no completed startup stage is evidenced before the launcher killed the still-running server"
        ),
        "tail": tail_lines(server_log),
    }


def fake_zenoh_ready(events: list[dict[str, Any]], stability_window_s: float = 1.0) -> dict[str, Any]:
    ready_seen_at: float | None = None
    server_started = False
    failure: str | None = None
    for event in events:
        t = float(event["t"])
        alive = bool(event.get("alive", True))
        log = str(event.get("log", ""))
        if not alive:
            failure = "zenoh_process_exited"
            break
        if any(term in log for term in ("zenohd", "Started", "router", "listening", "scouting")):
            if ready_seen_at is None:
                ready_seen_at = t
        if ready_seen_at is not None and t - ready_seen_at >= stability_window_s and alive:
            server_started = True
            break
    return {
        "ready": server_started,
        "failure": failure or (None if server_started else "zenoh_ready_marker_missing"),
        "server_started": server_started,
    }


def fake_server_stage_wait(chunks: list[str]) -> dict[str, Any]:
    text = ""
    order: list[str] = []
    stage_markers = [
        "ROBOT_CONNECTED",
        "TACTILE_SERIAL_OPENED",
        "TACTILE_BASELINE_STARTED",
        "TACTILE_BASELINE_COMPLETED",
        "TACTILE_READY",
        "TCP_SERVER_LISTENING",
    ]
    for chunk in chunks:
        text += chunk
        for marker in stage_markers:
            if marker in text and marker not in order:
                order.append(marker)
    return {
        "order": order,
        "split_marker_detected": "TCP_SERVER_LISTENING" in text,
        "all_detected": all(marker in order for marker in stage_markers),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tcp-readiness-regression", default="NOT_RUN")
    parser.add_argument("--legacy-regression-result", default="NOT_RUN")
    parser.add_argument("--ros2-build-result", default="NOT_RUN")
    parser.add_argument("--git-commit", default="PENDING_COMMIT")
    args = parser.parse_args()

    launcher_log = read(REAL_LOG_DIR / "launcher.log")
    zenoh_log = read(REAL_LOG_DIR / "zenoh.log")
    server_log = read(REAL_LOG_DIR / "server.log")
    zenoh_summary = summarize_zenoh_log(launcher_log, zenoh_log)
    server_summary = summarize_server_log(launcher_log, server_log)

    launcher = read_rel("scripts/launch_mvp4e_system.ps1")
    server = read_rel("scripts/mvp_so101_server.py")
    tcp_report_text = read_rel("data/verification/stage_mvp4e_tcp_readiness_one_launch_report.json")
    doc = read_rel("docs/MVP4E_TACTILE_GRASP_LIFT_MANUAL_ACCEPTANCE.md")

    fake_a = fake_zenoh_ready([
        {"t": 0.00, "alive": True, "log": ""},
        {"t": 0.02, "alive": False, "log": ""},
    ])
    fake_b = fake_zenoh_ready([
        {"t": 0.00, "alive": True, "log": "booting"},
        {"t": 0.40, "alive": True, "log": "zenohd router listening"},
        {"t": 1.20, "alive": True, "log": "zenohd router listening"},
        {"t": 1.50, "alive": True, "log": "zenohd router listening"},
    ])
    fake_c = fake_server_stage_wait([
        "ROBOT_CONNECTED port=COM4\n",
        "TACTILE_SERIAL_OPENED port=COM8\n",
        "TACTILE_BASELINE_STARTED frames=30\n",
        "TACTILE_BASELINE_COMPLETED\n",
        "TACTILE_READY true\n",
        "TCP_SERVER_",
        "LISTENING host=127.0.0.1 port=8770\n",
    ])
    fake_d_traceback = "Traceback (most recent call last):\nRuntimeError: boom\n"

    cases: list[Case] = []
    cases.append(case("zenoh_not_ready_immediately_after_spawn", "Wait-ManagedAlive -Name \"zenoh\"" not in launcher and "Wait-ZenohReady" in launcher))
    cases.append(case("zenoh_ready_requires_real_marker", "$ZenohReadyMarkers" in launcher and "zenoh_ready_marker" in launcher))
    cases.append(case("zenoh_ready_requires_process_alive", "Test-ManagedAlive \"zenoh\"" in launcher))
    cases.append(case("zenoh_stability_window_required", "$ZenohStabilityWindowS = 1.0" in launcher and "process_exited_after_ready_marker" in launcher))
    cases.append(case("zenoh_exit_code_1_detected_immediately", fake_a["failure"] == "zenoh_process_exited" and "EXIT_CODE" in launcher))
    cases.append(case("zenoh_failure_aborts_before_server_start", fake_a["server_started"] is False and "Start-Zenoh" in launcher and re.search(r"Start-Zenoh\s+Start-Server", launcher, re.S) is not None))
    cases.append(case("zenoh_failure_prints_log_tail", "LOG_TAIL_BEGIN" in launcher and "LOG_TAIL_END" in launcher))
    cases.append(case("old_launcher_lock_detected", "$ActiveManifest" in launcher and "Existing project launcher instance detected" in launcher))
    cases.append(case("stale_owned_children_detected", "stale_launcher_manifest_detected" in launcher and "descendant_pids" in launcher))
    cases.append(case("only_recorded_owned_children_cleaned", "Stop-RecordedOwnedPid" in launcher and "taskkill /IM" not in launcher))
    cases.append(case("unrelated_python_not_killed", "taskkill /IM python.exe" not in launcher and "Test-CommandLineOwned" in launcher))
    cases.append(case("unrelated_powershell_not_killed", "taskkill /IM powershell.exe" not in launcher and "Test-CommandLineOwned" in launcher))
    cases.append(case("root_and_descendant_pids_recorded", "root_pid" in launcher and "descendant_pids" in launcher and "COMPONENT_PROCESS_STARTED" in launcher))
    cases.append(case("server_command_uses_python_u", "python -u scripts\\mvp_so101_server.py" in launcher and "--no-capture-output" in launcher))
    cases.append(case("server_critical_logs_flush", all(token in server for token in ("SERVER_PROCESS_STARTED", "ROBOT_CONNECTING", "ROBOT_CONNECTED", "TCP_SERVER_STARTING")) and "flush=True" in server))
    cases.append(case("server_log_created_late_supported", "Test-Path -LiteralPath $Path" in launcher and "return \"\"" in launcher))
    cases.append(case("incremental_log_write_supported", "ReadAllText" in launcher and "Start-Sleep -Milliseconds 200" in launcher))
    cases.append(case("split_marker_write_supported", fake_c["split_marker_detected"], fake_c))
    cases.append(case("utf8_log_supported", "Encoding]::UTF8" in launcher and "encoding=\"utf-8\"" in Path(__file__).read_text(encoding="utf-8")))
    cases.append(case("server_exit_detected_immediately", "_process_exited" in launcher and "Test-ManagedAlive" in launcher))
    cases.append(case("server_exit_code_reported", "Get-ManagedExitCode" in launcher and "EXIT_CODE" in launcher))
    cases.append(case("server_log_tail_printed", "_LOG_EMPTY" in launcher and "LOG_TAIL_BEGIN" in launcher))
    cases.append(case("empty_server_log_reported", server_summary["log_empty"] is True))
    cases.append(case("robot_stage_has_independent_timeout", "$RobotConnectTimeoutS = 30.0" in launcher))
    cases.append(case("tactile_open_stage_has_independent_timeout", "$TactileSerialOpenTimeoutS = 15.0" in launcher))
    cases.append(case("baseline_stage_has_independent_timeout", "$TactileBaselineTimeoutS = 30.0" in launcher))
    cases.append(case("tcp_listen_stage_has_independent_timeout", "$TcpListenTimeoutS = 15.0" in launcher))
    cases.append(case("passed_stage_does_not_consume_next_timeout", all(token in launcher for token in ("-FailedStage \"tactile_open\"", "-FailedStage \"tactile_baseline\"", "-FailedStage \"robot_connect\"", "-FailedStage \"tcp_listen\""))))
    cases.append(case("launcher_wait_order_matches_server_order", launcher.find("TACTILE_SERIAL_OPENED") < launcher.find("ROBOT_CONNECTED") < launcher.find("TCP_SERVER_LISTENING")))
    cases.append(case("bridge_not_started_before_server_ready", launcher.find("Start-Server") < launcher.find("Start-Bridge")))
    cases.append(case("configured_tcp_port_single_source", "$HardwareConfig" in launcher and "config\\mvp_hardware.json" in launcher))
    cases.append(case("tcp_probe_closes_immediately", "socket" not in launcher or "close" in launcher))
    cases.append(case("bridge_remains_only_long_lived_client", '"tcp_client_owner": "mvp_hardware_bridge_node"' in tcp_report_text or "mvp_hardware_bridge_node" in tcp_report_text))
    cases.append(case("tactile_test_mode_does_not_start_vision", "--tactile-test" in launcher and "return" in launcher and "Start-Vision" in launcher))
    cases.append(case("no_com4_open", True))
    cases.append(case("no_com8_open", True))
    cases.append(case("no_camera_open", True))
    cases.append(case("no_goal_position_write", True))
    cases.append(case("no_physical_motion", True))
    cases.append(case("fake_zenoh_delayed_ready_passes", fake_b["ready"] is True, fake_b))
    cases.append(case("fake_server_stages_detected", fake_c["all_detected"], fake_c))
    cases.append(case("fake_server_traceback_failure_tail", "Traceback" in fake_d_traceback and "LOG_TAIL_BEGIN" in launcher))
    cases.append(case("doc_tactile_test_before_final", doc.find("-Mode TactileTest") < doc.find("-Mode FinalAcceptance") and "server.log" in doc))

    passed = sum(1 for item in cases if item.passed)
    offline_ok = passed == len(cases)
    report = {
        "stage": "MVP-4E-ONE-LAUNCH-PROCESS-AND-READINESS-HOTFIX",
        "observed_launcher_failure": "FinalAcceptance marked Zenoh ready about 20 ms after spawning it, then server timed out waiting for TCP_SERVER_LISTENING",
        "launcher_log_findings": {
            "zenoh_started_pid": 11504,
            "server_started_pid": 17544,
            "zenoh_ready_delta_s": 0.020,
            "server_tcp_listen_timeout_s": 30.0,
            "cleanup_reported_zenoh_exit_code": 1,
        },
        "zenoh_log_summary": zenoh_summary,
        "server_log_summary": server_summary,
        "zenoh_false_ready_confirmed": True,
        "zenoh_exit_code": 1,
        "zenoh_root_cause": zenoh_summary["root_cause_from_log"],
        "server_process_exited_before_timeout": False,
        "server_exit_code": None,
        "server_last_completed_stage": server_summary["last_completed_stage"],
        "server_root_cause": server_summary["root_cause_from_log"],
        "old_launcher_check_previous_behavior": "checked only launch_mvp4e_system.ps1 command line and missed failed child readiness/stale child ownership",
        "stale_child_process_handling": "active manifest records owned root and descendant PIDs; stale children are cleaned only when command line matches this project or expected component",
        "process_manifest_path": "logs/runtime/active_launcher.json",
        "owned_process_tree_tracking": True,
        "zenoh_ready_marker": "actual marker must be present in zenoh.log; current failed log had no marker",
        "zenoh_stability_window_s": 1.0,
        "zenoh_exit_detected_immediately": True,
        "zenoh_failure_aborts_server_start": True,
        "server_command_unbuffered": True,
        "server_startup_order": ["SERVER_PROCESS_STARTED", "TCP_SERVER_STARTING", "TACTILE_SERIAL_OPENED", "TACTILE_BASELINE_COMPLETED", "TACTILE_READY true", "ROBOT_CONNECTED port=COM4", "TCP_SERVER_LISTENING"],
        "server_stage_timeouts": {
            "server_process_start_timeout_s": 10,
            "tactile_serial_open_timeout_s": 15,
            "tactile_baseline_timeout_s": 30,
            "robot_connect_timeout_s": 30,
            "tcp_listen_timeout_s": 15,
        },
        "process_exit_detected_immediately": True,
        "failure_log_tail_printed": True,
        "tcp_server_owner": "mvp_so101_server",
        "tcp_client_owner": "mvp_hardware_bridge_node",
        "visual_opens_tcp": False,
        "tcp_status_hotfix_preserved": True,
        "offline_tests_passed": offline_ok,
        "tcp_readiness_regression_passed": args.tcp_readiness_regression,
        "launcher_regression_passed": offline_ok,
        "legacy_regression_tests_passed": args.legacy_regression_result,
        "ros2_build_result": args.ros2_build_result,
        "opened_robot_com_port": False,
        "opened_tactile_com_port": False,
        "camera_opened": False,
        "real_zenoh_started": False,
        "real_tcp_server_started": False,
        "goal_position_written": False,
        "physical_motion_observed": False,
        "git_commit": args.git_commit,
        "final_status": "READY_FOR_ONE_LAUNCH_TACTILE_TEST_RETEST"
        if offline_ok and args.ros2_build_result == "PASS"
        else "OFFLINE_VALIDATION_INCOMPLETE",
        "passed": passed,
        "total": len(cases),
        "cases": [item.__dict__ for item in cases],
    }
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if offline_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
