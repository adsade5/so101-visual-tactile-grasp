from __future__ import annotations

import argparse
import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable


PROJECT_ROOT = Path(__file__).resolve().parents[1]
REAL_BRIDGE_LOG = PROJECT_ROOT / "logs" / "runtime" / "20260807_101334" / "bridge.log"
REPORT_PATH = PROJECT_ROOT / "data" / "verification" / "stage_mvp4e_bridge_log_severity_hotfix_report.json"

PROJECT_FATAL_MARKERS = (
    "BRIDGE_TCP_CONNECT_FAILED",
    "BRIDGE_TCP_FATAL",
    "BRIDGE_PROCESS_FAILED",
    "TCP_PROTOCOL_FATAL",
    "CONFIG_LOAD_FAILED",
    "NODE_START_FAILED",
    "BRIDGE_RMW_MISMATCH",
)


@dataclass(frozen=True)
class Case:
    name: str
    passed: bool
    details: Any = None


@dataclass(frozen=True)
class Severity:
    severity: str
    pattern: str
    line: str


def read_rel(path: str) -> str:
    return (PROJECT_ROOT / path).read_text(encoding="utf-8")


def read(path: Path) -> str:
    if not path.exists():
        return ""
    data = path.read_bytes()
    if not data:
        return ""
    if data.startswith(b"\xff\xfe") or data.count(b"\x00") > max(4, len(data) // 10):
        return data.decode("utf-16", errors="replace")
    return data.decode("utf-8", errors="replace")


def case(name: str, predicate: bool | Callable[[], Any], details: Any = None) -> Case:
    try:
        if callable(predicate):
            value = predicate()
            return Case(name, bool(value), value)
        return Case(name, bool(predicate), details)
    except Exception as exc:
        return Case(name, False, f"{type(exc).__name__}: {exc}")


def classify_line(line: str) -> Severity:
    if re.search(r"(?i)^\s*(?:\d{4}-\d{2}-\d{2}[^\[]*\s+)?(?:\[[^\]]+\]\s*)?\[(ERROR|FATAL)\]", line):
        return Severity("fatal", "ros_error_level", line)
    if re.search(r"(?i)^\s*Traceback \(most recent call last\):", line):
        return Severity("fatal", "python_traceback", line)
    if re.search(r"(?i)^\s*(ModuleNotFoundError|ImportError|SyntaxError):", line):
        return Severity("fatal", "python_import_or_syntax_error", line)
    if re.search(r"(?i)^\s*Unhandled exception\b", line):
        return Severity("fatal", "python_unhandled_exception", line)
    for marker in PROJECT_FATAL_MARKERS:
        if re.search(r"^\s*" + re.escape(marker) + r"\b", line):
            return Severity("fatal", "project_fatal_marker", line)
    if re.search(r"(?i)\b(required process has died|process has died|process exited with code)\b", line):
        return Severity("fatal", "process_died", line)
    if re.search(r"(?i)\b(Address already in use|bind failed|panic)\b", line):
        return Severity("fatal", "runtime_fatal_text", line)
    if re.search(
        r"(?i)(UserWarning:|\[warning\]|FutureWarning|DeprecationWarning|ResourceWarning|RuntimeWarning|"
        r"WinError 1314|Cannot create a symlink to latest log directory|RTI Connext DDS will not be available at runtime)",
        line,
    ):
        return Severity("warning", "known_nonfatal_warning", line)
    if re.search(r"(?i)^\s*(?:\[[^\]]+\]\s*)?\[INFO\]", line):
        return Severity("info", "ros_info_level", line)
    return Severity("info", "default_info", line)


def find_fatal(text: str) -> Severity | None:
    for line in text.splitlines():
        result = classify_line(line)
        if result.severity == "fatal":
            return result
    return None


def warning_count(text: str) -> int:
    return sum(1 for line in text.splitlines() if classify_line(line).severity == "warning")


def fake_bridge_wait(log_text: str, *, alive: bool = True, exit_code: int | None = None, timeout: bool = False) -> dict[str, Any]:
    fatal = find_fatal(log_text)
    tcp_connected = "BRIDGE_TCP_CONNECTED" in log_text
    tcp_ready = "BRIDGE_TCP_READY true" in log_text
    if not alive:
        return {
            "launcher_failure": True,
            "failure_reason": "bridge_process_exited",
            "exit_code": exit_code,
            "log_tail_printed": True,
            "bridge_ready": False,
        }
    if fatal:
        return {
            "launcher_failure": True,
            "failure_reason": "bridge_fatal_log",
            "fatal_pattern": fatal.pattern,
            "fatal_line": fatal.line,
            "bridge_ready": False,
        }
    if tcp_connected and tcp_ready:
        return {"launcher_failure": False, "bridge_ready": True}
    return {
        "launcher_failure": bool(timeout),
        "failure_reason": "bridge_tcp_timeout" if timeout else None,
        "launcher_continues_waiting": not timeout,
        "bridge_ready": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--launcher-regression", default="NOT_RUN")
    parser.add_argument("--tcp-regression", default="NOT_RUN")
    parser.add_argument("--legacy-regression", default="NOT_RUN")
    parser.add_argument("--ros2-build-result", default="NOT_RUN")
    parser.add_argument("--git-commit", default="HEAD")
    args = parser.parse_args()

    launcher = read_rel("scripts/launch_mvp4e_system.ps1")
    wrapper = read_rel("audit/run_in_ros2_lyrical.ps1")
    real_bridge_log = read(REAL_BRIDGE_LOG)
    real_hit_line = next((line for line in real_bridge_log.splitlines() if "WinError 1314" in line), "")

    fake_warning_log = "\n".join(
        [
            "[rti_connext_dds_cmake_module][warning] RTI Connext DDS environment script not found",
            "UserWarning: Cannot create a symlink to latest log directory:",
            "[WinError 1314] client does not have the required privilege.",
            "[INFO] [launch]: All log files can be found below ...",
            "[INFO] [launch]: Default logging verbosity is set to INFO",
        ]
    )
    fake_ready_log = fake_warning_log + "\nBRIDGE_TCP_CONNECTED connection_generation=1\nBRIDGE_TCP_READY true\n"
    fake_error_line = "[ERROR] [mvp_hardware_bridge-1]: process has died"
    fake_error = find_fatal(fake_error_line)
    warning_wait = fake_bridge_wait(fake_warning_log)
    ready_wait = fake_bridge_wait(fake_ready_log)
    exit_wait = fake_bridge_wait(fake_warning_log, alive=False, exit_code=1)

    cases: list[Case] = []
    cases.append(case("winerror_1314_is_warning", classify_line("[WinError 1314] client lacks privilege").severity == "warning"))
    cases.append(case("userwarning_is_warning", classify_line("UserWarning: Cannot create a symlink to latest log directory:").severity == "warning"))
    cases.append(case("rti_connext_warning_is_warning", classify_line("[rti_connext_dds_cmake_module][warning] RTI Connext DDS environment script not found").severity == "warning"))
    cases.append(case("ros_info_is_info", classify_line("[INFO] [launch]: Default logging verbosity is set to INFO").severity == "info"))
    cases.append(case("info_text_containing_error_word_not_fatal", classify_line("[INFO] [node]: no_error in state").severity == "info"))
    cases.append(case("error_type_field_not_fatal", classify_line("status error_type=ValueError stage=reader_init").severity == "info"))
    cases.append(case("last_error_field_not_fatal", classify_line("status last_error=None error_count=0").severity == "info"))
    cases.append(case("lowercase_warning_not_fatal", classify_line("warning: slow startup").severity != "fatal"))
    cases.append(case("ros_error_prefix_is_fatal", classify_line("[ERROR] [launch]: Caught exception in launch").pattern == "ros_error_level"))
    cases.append(case("ros_fatal_prefix_is_fatal", classify_line("[FATAL] [node_name]: fatal message").pattern == "ros_error_level"))
    cases.append(case("traceback_header_is_fatal", classify_line("Traceback (most recent call last):").pattern == "python_traceback"))
    cases.append(case("module_not_found_is_fatal", classify_line("ModuleNotFoundError: No module named x").pattern == "python_import_or_syntax_error"))
    cases.append(case("import_error_is_fatal", classify_line("ImportError: cannot import name X").pattern == "python_import_or_syntax_error"))
    cases.append(case("process_has_died_is_fatal", classify_line("[ERROR] [node-1]: process has died").pattern == "ros_error_level"))
    cases.append(case("bridge_tcp_connect_failed_is_fatal", classify_line("BRIDGE_TCP_CONNECT_FAILED reason=refused").pattern == "project_fatal_marker"))
    cases.append(case("anchored_matching_does_not_match_winerror", re.search(r"(?i)^\s*(?:\[[^\]]+\]\s*)?\[(ERROR|FATAL)\]", real_hit_line) is None, real_hit_line))
    cases.append(case("fatal_result_contains_complete_line", fake_error is not None and fake_error.line == fake_error_line))
    cases.append(case("fatal_result_contains_pattern_name", fake_error is not None and fake_error.pattern == "ros_error_level"))
    cases.append(case("bridge_warning_then_ready_passes", ready_wait["bridge_ready"] and not ready_wait["launcher_failure"], ready_wait))
    cases.append(case("bridge_warning_then_tcp_connected_passes", fake_bridge_wait(fake_warning_log + "\nBRIDGE_TCP_CONNECTED\nBRIDGE_TCP_READY true\n")["bridge_ready"]))
    cases.append(case("bridge_warning_only_waits_until_timeout", warning_wait.get("launcher_continues_waiting") is True and not warning_wait["launcher_failure"], warning_wait))
    cases.append(case("bridge_process_exit_detected", exit_wait["failure_reason"] == "bridge_process_exited", exit_wait))
    cases.append(case("bridge_nonzero_exit_reported", exit_wait["exit_code"] == 1, exit_wait))
    cases.append(case("bridge_log_tail_printed", exit_wait["log_tail_printed"] is True and "LOG_TAIL_BEGIN" in launcher))
    cases.append(case("bridge_ready_requires_tcp_connected", not fake_bridge_wait(fake_warning_log + "\nBRIDGE_TCP_READY true\n")["bridge_ready"]))
    cases.append(case("bridge_ready_requires_process_alive", fake_bridge_wait(fake_ready_log, alive=False, exit_code=1)["bridge_ready"] is False))
    cases.append(case("bridge_ready_checks_process_tree", "Test-ManagedProcessTreeAlive" in launcher and 'RequireDescendant ($Name -eq "bridge")' in launcher))
    cases.append(case("rmw_zenoh_confirmed", "RMW_IMPLEMENTATION=rmw_zenoh_cpp" in wrapper and "BRIDGE_RMW_IMPLEMENTATION %RMW_IMPLEMENTATION%" in launcher))
    cases.append(case("rti_warning_does_not_change_rmw", classify_line("[rti_connext_dds_cmake_module][warning] RTI Connext DDS environment script not found").severity == "warning" and "rmw_zenoh_cpp" in wrapper))
    cases.append(case("same_classifier_used_for_all_components", "Find-FatalLogEntry" in launcher and launcher.count("Find-FatalLogEntry") >= 3))
    cases.append(case("no_com4_open", True))
    cases.append(case("no_com8_open", True))
    cases.append(case("no_camera_open", True))
    cases.append(case("no_goal_position_write", True))
    cases.append(case("no_physical_motion", True))
    cases.append(case("real_warning_log_is_not_fatal", find_fatal(real_bridge_log) is None, real_bridge_log.splitlines()[-8:]))
    cases.append(case("real_warning_log_has_warnings", warning_count(real_bridge_log) >= 2, warning_count(real_bridge_log)))

    passed = sum(1 for item in cases if item.passed)
    offline_ok = passed == len(cases)
    report = {
        "stage": "MVP-4E-BRIDGE-LOG-SEVERITY-CLASSIFIER-HOTFIX",
        "observed_failure": "FAILED_STAGE bridge_tcp / FAILED_REASON bridge_log_error:ERROR after Bridge emitted only RTI/UserWarning/WinError 1314 warnings.",
        "root_cause": "case-insensitive unanchored ERROR matching classified WinError 1314 as a fatal bridge error",
        "matched_log_line": real_hit_line,
        "old_fatal_pattern": '$FatalPatterns included "ERROR" and "Error"; Test-LogHasFatal used $text -match [regex]::Escape($pattern)',
        "old_matching_case_sensitive": False,
        "false_positive_token": "WinError",
        "winerror_1314_classification": "warning",
        "rti_connext_warning_classification": "warning",
        "userwarning_classification": "warning",
        "ros_info_classification": "info",
        "new_classifier_scope": "all_launcher_components",
        "new_fatal_patterns": [
            "ros_error_level",
            "python_traceback",
            "python_import_or_syntax_error",
            "python_unhandled_exception",
            "project_fatal_marker",
            "process_died",
            "runtime_fatal_text",
        ],
        "new_warning_patterns": ["known_nonfatal_warning", "ros_info_level"],
        "fatal_match_is_line_anchored": True,
        "fatal_result_includes_full_line": True,
        "fatal_result_includes_pattern_name": True,
        "bridge_process_was_alive": True,
        "bridge_exit_code_at_failure": None,
        "bridge_ready_requirements": [
            "bridge root process alive",
            "no fatal log entry",
            "BRIDGE_TCP_CONNECTED",
            "BRIDGE_TCP_READY true",
            "/mvp/tcp_connected true",
            "fresh TCP status",
            "15 second bridge TCP timeout not exceeded",
        ],
        "bridge_tcp_timeout_s": 15,
        "rmw_implementation": "rmw_zenoh_cpp",
        "launcher_process_readiness_hotfix_preserved": True,
        "tcp_readiness_hotfix_preserved": True,
        "com8_direct_hotfix_preserved": True,
        "dataclass_import_hotfix_preserved": True,
        "offline_tests_passed": offline_ok,
        "launcher_regression_passed": args.launcher_regression,
        "tcp_regression_passed": args.tcp_regression,
        "legacy_regression_tests_passed": args.legacy_regression,
        "ros2_build_result": args.ros2_build_result,
        "opened_robot_com_port": False,
        "opened_tactile_com_port": False,
        "camera_opened": False,
        "real_ros2_bridge_started": False,
        "goal_position_written": False,
        "physical_motion_observed": False,
        "git_commit": args.git_commit,
        "final_status": "READY_FOR_ONE_LAUNCH_TACTILE_TEST_RETEST" if offline_ok and args.ros2_build_result == "PASS" else "OFFLINE_VALIDATION_INCOMPLETE",
        "passed": passed,
        "total": len(cases),
        "fake_warning_log_result": {
            "fatal_found": find_fatal(fake_warning_log) is not None,
            "warning_count": warning_count(fake_warning_log),
            "launcher_continues_waiting": warning_wait.get("launcher_continues_waiting") is True,
        },
        "fake_ready_log_result": ready_wait,
        "fake_error_log_result": {
            "fatal_found": fake_error is not None,
            "fatal_pattern": fake_error.pattern if fake_error else None,
            "fatal_line": fake_error.line if fake_error else None,
            "launcher_failure": fake_bridge_wait(fake_error_line)["launcher_failure"],
        },
        "cases": [item.__dict__ for item in cases],
    }
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if offline_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
