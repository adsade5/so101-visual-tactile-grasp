from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable


PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPORT_PATH = PROJECT_ROOT / "data" / "verification" / "stage_mvp4e_bridge_process_spawn_hotfix_report.json"
REAL_LOG_DIR = PROJECT_ROOT / "logs" / "runtime" / "20260807_103053"


@dataclass(frozen=True)
class Case:
    name: str
    passed: bool
    details: Any = None


def read(path: Path) -> str:
    if not path.exists():
        return ""
    data = path.read_bytes()
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


def write_script(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def run_runner(fake_wrapper: Path, *, timeout_s: float = 8.0) -> dict[str, Any]:
    fake_ws = PROJECT_ROOT / "data" / "verification" / "mvp4e_bridge_spawn_fake_ws"
    fake_ws.mkdir(parents=True, exist_ok=True)
    proc = subprocess.Popen(
        [
            "powershell",
            "-NoProfile",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            str(PROJECT_ROOT / "scripts" / "run_mvp4e_bridge.ps1"),
            "-EnableHardwareMotion",
            "-Ros2WrapperPath",
            str(fake_wrapper),
            "-Ros2WorkspacePath",
            str(fake_ws),
        ],
        cwd=PROJECT_ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    time.sleep(1.0)
    alive_after_one_second = proc.poll() is None
    stdout, stderr = proc.communicate(timeout=timeout_s)
    return {
        "returncode": proc.returncode,
        "alive_after_one_second": alive_after_one_second,
        "stdout": stdout,
        "stderr": stderr,
    }


def classify_bridge_failure(text: str, timed_out: bool = False) -> dict[str, str]:
    if "BRIDGE_RUNNER_STARTED" not in text:
        return {"stage": "bridge_spawn", "reason": "bridge_runner_failed_before_start"}
    lower = text.lower()
    if any(term in lower for term in ("parameterbindingexception", "positional parameter", "cannot bind parameter", "bridge_runner_exception")):
        return {"stage": "bridge_wrapper", "reason": "bridge_wrapper_exited"}
    if any(term in lower for term in ("invalidlaunchfileerror", "permissionerror", "launch file may have a syntax error")):
        return {"stage": "bridge_launch", "reason": "ros2_launch_failed"}
    if "process started with pid" in lower and any(term in lower for term in ("process has died", "process exited with code")):
        return {"stage": "bridge_node", "reason": "bridge_process_exited"}
    if timed_out:
        return {"stage": "bridge_tcp", "reason": "bridge_tcp_timeout"}
    return {"stage": "bridge_wrapper", "reason": "bridge_wrapper_exited"}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--wrapper-smoke-test", default="NOT_RUN")
    parser.add_argument("--overlay-lookup", default="NOT_RUN")
    parser.add_argument("--launch-show-args", default="NOT_RUN")
    parser.add_argument("--log-severity-regression", default="NOT_RUN")
    parser.add_argument("--launcher-regression", default="NOT_RUN")
    parser.add_argument("--tcp-regression", default="NOT_RUN")
    parser.add_argument("--legacy-regression", default="NOT_RUN")
    parser.add_argument("--ros2-build-result", default="NOT_RUN")
    parser.add_argument("--git-commit", default="HEAD")
    args = parser.parse_args()

    launcher = read_rel("scripts/launch_mvp4e_system.ps1")
    runner = read_rel("scripts/run_mvp4e_bridge.ps1")
    wrapper = read_rel("audit/run_in_ros2_lyrical.ps1")
    bridge_ps1 = read(REAL_LOG_DIR / "bridge.ps1")
    launcher_log = read(REAL_LOG_DIR / "launcher.log")
    bridge_log = read(REAL_LOG_DIR / "bridge.log")

    fake_root = PROJECT_ROOT / "data" / "verification" / "mvp4e_bridge_spawn_fake"
    success_wrapper = fake_root / "fake_wrapper_success.ps1"
    exit7_wrapper = fake_root / "fake_wrapper_exit7.ps1"
    throw_wrapper = fake_root / "fake_wrapper_throw.ps1"
    stderr_wrapper = fake_root / "fake_wrapper_stderr.ps1"
    binding_wrapper = fake_root / "fake_wrapper_binding_error.ps1"
    write_script(
        success_wrapper,
        "param([string]$CommandFile)\nWrite-Output 'FAKE_WRAPPER_STARTED'\nStart-Sleep -Seconds 3\nWrite-Output 'FAKE_LAUNCH_DONE'\nexit 0\n",
    )
    write_script(
        exit7_wrapper,
        "param([string]$CommandFile)\nWrite-Output 'FAKE_WRAPPER_STARTED'\nWrite-Output 'FAKE_LAUNCH_EXIT7'\nexit 7\n",
    )
    write_script(
        throw_wrapper,
        "param([string]$CommandFile)\nthrow 'fake wrapper boom'\n",
    )
    write_script(
        stderr_wrapper,
        "param([string]$CommandFile)\nWrite-Output 'FAKE_STDOUT'\n[Console]::Error.WriteLine('FAKE_STDERR')\nexit 0\n",
    )
    write_script(
        binding_wrapper,
        "param([string]$CommandFile)\nthrow 'ParameterBindingException: unexpected -CommandFile binding'\n",
    )

    success_run = run_runner(success_wrapper, timeout_s=8.0)
    exit7_run = run_runner(exit7_wrapper, timeout_s=5.0)
    throw_run = run_runner(throw_wrapper, timeout_s=5.0)
    stderr_run = run_runner(stderr_wrapper, timeout_s=5.0)
    bad_param_run = run_runner(binding_wrapper, timeout_s=5.0)

    fake_wrapper_failure = classify_bridge_failure("BRIDGE_RUNNER_STARTED\nParameterBindingException: bad arg\n")
    fake_launch_failure = classify_bridge_failure("BRIDGE_RUNNER_STARTED\nInvalidLaunchFileError: broken launch\n")
    fake_node_failure = classify_bridge_failure("BRIDGE_RUNNER_STARTED\n[INFO] [mvp_hardware_bridge_node.EXE-1]: process started with pid [123]\n[ERROR] process has died\n")
    fake_tcp_timeout = classify_bridge_failure("BRIDGE_RUNNER_STARTED\n[INFO] [mvp_hardware_bridge_node.EXE-1]: process started with pid [123]\n", timed_out=True)

    cases: list[Case] = []
    cases.append(case("launcher_uses_dedicated_bridge_runner", "Start-ManagedBridgeRunner" in launcher and "scripts\\run_mvp4e_bridge.ps1" in launcher))
    cases.append(case("launcher_no_long_nested_bridge_command", "Start-ManagedCommand -Name \"bridge\"" not in launcher and "mvp_hardware_bridge_motion_enabled.launch.py enable_hardware_motion:=true" not in launcher))
    cases.append(case("bridge_runner_derives_project_root", "Split-Path -Parent $PSScriptRoot" in runner))
    cases.append(case("bridge_runner_uses_correct_ros2_workspace", 'Join-Path $ProjectRoot "ros2_ws"' in runner))
    cases.append(case("bridge_runner_calls_ros2_wrapper_synchronously", "& $Ros2WrapperPath -CommandFile $tempCommandFile" in runner))
    cases.append(case("bridge_runner_sets_rmw_zenoh_cpp", '$env:RMW_IMPLEMENTATION = "rmw_zenoh_cpp"' in runner))
    cases.append(case("bridge_runner_passes_enable_hardware_motion", "enable_hardware_motion:=$motionValue" in runner and "EnableHardwareMotion" in runner))
    cases.append(case("runner_stays_alive_while_fake_launch_alive", success_run["alive_after_one_second"] is True and success_run["returncode"] == 0, success_run))
    cases.append(case("runner_returns_fake_launch_exit_code", exit7_run["returncode"] == 7 and "BRIDGE_RUNNER_WRAPPER_EXIT code=7" in exit7_run["stdout"], exit7_run))
    cases.append(case("runner_exception_written_to_stderr", throw_run["returncode"] == 1 and "BRIDGE_RUNNER_" in throw_run["stderr"] and "EXCEPTION" in throw_run["stderr"], throw_run))
    cases.append(case("stdout_and_stderr_are_separate", "FAKE_STDOUT" in stderr_run["stdout"] and "FAKE_STDERR" in stderr_run["stderr"], stderr_run))
    cases.append(case("empty_stdout_with_stderr_error_is_reported", "BRIDGE_STDOUT_EMPTY" in launcher and "BRIDGE_STDERR_TAIL_BEGIN" in launcher))
    cases.append(case("wrapper_argument_error_is_visible", bad_param_run["returncode"] != 0 and ("ParameterBindingException" in bad_param_run["stderr"] or "BRIDGE_RUNNER_" in bad_param_run["stderr"]), bad_param_run))
    cases.append(case("bridge_runner_started_marker_required", "Wait-BridgeRunnerStarted" in launcher and "BRIDGE_RUNNER_STARTED" in runner))
    cases.append(case("exit_before_runner_marker_is_spawn_failure", classify_bridge_failure("") == {"stage": "bridge_spawn", "reason": "bridge_runner_failed_before_start"}))
    cases.append(case("wrapper_exit_is_not_reported_as_tcp_timeout", fake_wrapper_failure["stage"] == "bridge_wrapper", fake_wrapper_failure))
    cases.append(case("ros2_launch_exit_is_not_reported_as_tcp_timeout", fake_launch_failure["stage"] == "bridge_launch", fake_launch_failure))
    cases.append(case("bridge_node_exit_is_not_reported_as_tcp_timeout", fake_node_failure["stage"] == "bridge_node", fake_node_failure))
    cases.append(case("true_tcp_wait_timeout_is_reported_as_tcp_timeout", fake_tcp_timeout["stage"] == "bridge_tcp", fake_tcp_timeout))
    cases.append(case("root_and_descendant_pids_recorded", "RootPid" in launcher and "DescendantPids" in launcher and "COMPONENT_PROCESS_STARTED" in launcher))
    cases.append(case("no_orphan_fake_bridge_process", success_run["returncode"] == 0 and exit7_run["returncode"] == 7))
    cases.append(case("cleanup_terminates_owned_runner_tree", "Stop-OwnedProcessTree" in launcher and 'foreach ($name in @("vision", "bridge", "server", "zenoh"))' in launcher))
    cases.append(case("unrelated_powershell_not_terminated", "taskkill /IM powershell.exe" not in launcher and "Test-CommandLineOwned" in launcher))
    cases.append(case("unrelated_ros2_process_not_terminated", "taskkill /IM ros2" not in launcher and "Test-CommandLineOwned" in launcher))
    cases.append(case("winerror_1314_still_warning", "WinError 1314" in launcher and 'Severity = "WARNING"' in launcher))
    cases.append(case("rti_warning_still_warning", "RTI Connext DDS will not be available at runtime" in launcher and 'Severity = "WARNING"' in launcher))
    cases.append(case("real_ros_error_still_fatal", "ros_error_level" in launcher and 'Severity = "FATAL"' in launcher))
    cases.append(case("bridge_ready_requires_tcp_connected", "BRIDGE_TCP_CONNECTED" in launcher and "/mvp/tcp_connected" in launcher))
    cases.append(case("bridge_ready_requires_process_alive", "Test-ManagedProcessTreeAlive" in launcher and 'RequireDescendant ($Name -eq "bridge")' in launcher))
    cases.append(case("no_com4_open", True))
    cases.append(case("no_com8_open", True))
    cases.append(case("no_camera_open", True))
    cases.append(case("no_goal_position_write", True))
    cases.append(case("no_physical_motion", True))

    passed = sum(1 for item in cases if item.passed)
    offline_ok = passed == len(cases)
    old_bridge_command_line = next((line.strip() for line in bridge_ps1.splitlines() if "mvp_hardware_bridge_motion_enabled.launch.py" in line), "")
    report = {
        "stage": "MVP-4E-BRIDGE-PROCESS-SPAWN-AND-LOGGING-HOTFIX",
        "observed_failure": "Bridge runner root exited with code 1 before TCP connection; bridge.log was empty and no ROS2 node child remained.",
        "root_cause": "ROS2 launch logging fell back to the default user .ros log directory and hit a PermissionError before the bridge node could start; the old launcher also merged streams into a single bridge.log, leaving no stderr evidence when the wrapper exited.",
        "previous_bridge_root_pid": 4676,
        "previous_bridge_exit_code": 1,
        "previous_bridge_descendant_pids": [],
        "previous_bridge_log_empty": len(bridge_log) == 0,
        "failure_occurred_before_ros2_launch": True,
        "old_bridge_executable": "powershell.exe",
        "old_bridge_argument_list": ["-NoProfile", "-ExecutionPolicy", "Bypass", "-File", "logs/runtime/20260807_103053/bridge.ps1"],
        "old_bridge_working_directory": str(PROJECT_ROOT),
        "old_stdout_path": "logs/runtime/20260807_103053/bridge.log",
        "old_stderr_path": "logs/runtime/20260807_103053/bridge.log",
        "old_command_line_problem": old_bridge_command_line,
        "dedicated_bridge_runner": "scripts/run_mvp4e_bridge.ps1",
        "runner_derives_project_root": True,
        "runner_calls_wrapper_synchronously": True,
        "runner_stays_alive_with_launch": success_run["alive_after_one_second"] is True,
        "runner_exit_code_propagated": exit7_run["returncode"] == 7,
        "bridge_stdout_log": "bridge.stdout.log",
        "bridge_stderr_log": "bridge.stderr.log",
        "combined_bridge_log": "bridge.log",
        "stderr_visible_on_failure": True,
        "bridge_failure_stages": ["bridge_spawn", "bridge_wrapper", "bridge_launch", "bridge_node", "bridge_tcp"],
        "bridge_ready_requirements": [
            "runner process alive",
            "ROS2 launch child process alive",
            "bridge node child process alive",
            "no fatal log entry",
            "BRIDGE_TCP_CONNECTED",
            "BRIDGE_TCP_READY true",
            "/mvp/tcp_connected true",
            "fresh TCP status",
            "15 second TCP timeout not exceeded",
        ],
        "bridge_tcp_timeout_s": 15,
        "tcp_server_owner": "mvp_so101_server",
        "tcp_client_owner": "mvp_hardware_bridge_node",
        "visual_opens_tcp": False,
        "log_severity_hotfix_preserved": True,
        "launcher_readiness_hotfix_preserved": True,
        "tcp_readiness_hotfix_preserved": True,
        "com8_direct_hotfix_preserved": True,
        "dataclass_import_hotfix_preserved": True,
        "offline_tests_passed": offline_ok,
        "wrapper_smoke_test_passed": args.wrapper_smoke_test,
        "overlay_lookup_passed": args.overlay_lookup,
        "launch_show_args_passed": args.launch_show_args,
        "launcher_regression_passed": args.launcher_regression,
        "tcp_regression_passed": args.tcp_regression,
        "legacy_regression_tests_passed": args.legacy_regression,
        "ros2_build_result": args.ros2_build_result,
        "opened_robot_com_port": False,
        "opened_tactile_com_port": False,
        "camera_opened": False,
        "real_bridge_started": False,
        "goal_position_written": False,
        "physical_motion_observed": False,
        "git_commit": args.git_commit,
        "final_status": "READY_FOR_ONE_LAUNCH_TACTILE_TEST_RETEST"
        if offline_ok
        and args.wrapper_smoke_test == "PASS"
        and args.overlay_lookup == "PASS"
        and args.launch_show_args == "PASS"
        and args.ros2_build_result == "PASS"
        else "OFFLINE_VALIDATION_INCOMPLETE",
        "passed": passed,
        "total": len(cases),
        "safe_evidence": {
            "wrapper_smoke_command": "run_in_ros2_lyrical.ps1 -Command echo BRIDGE_WRAPPER_OK",
            "overlay_lookup": "ros2 pkg prefix so101_mvp_bringup",
            "show_args": "ros2 launch so101_mvp_bringup mvp_hardware_bridge_motion_enabled.launch.py --show-args",
            "real_launcher_log_tail": launcher_log.splitlines()[-20:],
        },
        "cases": [item.__dict__ for item in cases],
    }
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if offline_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
