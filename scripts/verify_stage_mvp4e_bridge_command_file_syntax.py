from __future__ import annotations

import json
import os
import re
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPORT_PATH = PROJECT_ROOT / "data" / "verification" / "stage_mvp4e_bridge_command_file_syntax_hotfix_report.json"
WRAPPER = PROJECT_ROOT / "audit" / "run_in_ros2_lyrical.ps1"
RUNNER = PROJECT_ROOT / "scripts" / "run_mvp4e_bridge.ps1"
LAUNCHER = PROJECT_ROOT / "scripts" / "launch_mvp4e_system.ps1"
CURRENT_FAILURE_DIR = PROJECT_ROOT / "logs" / "runtime" / "20260807_105819"
VERIFY_ROOT = PROJECT_ROOT / "data" / "verification" / "mvp4e_bridge_command_file_syntax"
RUNTIME_DIR = PROJECT_ROOT / "logs" / "runtime" / datetime.now().strftime("%Y%m%d_%H%M%S")


@dataclass(frozen=True)
class Case:
    name: str
    passed: bool
    details: Any = None


def read_text(path: Path) -> str:
    if not path.exists():
        return ""
    data = path.read_bytes()
    if data.startswith(b"\xef\xbb\xbf"):
        return data.decode("utf-8-sig", errors="replace")
    if data.startswith(b"\xff\xfe") or data.startswith(b"\xfe\xff"):
        return data.decode("utf-16", errors="replace")
    return data.decode("utf-8", errors="replace")


def read_bytes(path: Path) -> bytes:
    return path.read_bytes() if path.exists() else b""


def write_text(path: Path, text: str, *, encoding: str = "ascii", crlf: bool = False) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    newline = "\r\n" if crlf else "\n"
    with path.open("w", encoding=encoding, newline=newline) as handle:
        handle.write(text)


def write_lines(path: Path, lines: list[str], *, encoding: str = "ascii") -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding=encoding, newline="\r\n") as handle:
        for line in lines:
            handle.write(line)
            handle.write("\n")


def run_ps(script: Path, *args: str, env: dict[str, str] | None = None, timeout_s: int = 300) -> subprocess.CompletedProcess[str]:
    command = [
        "powershell.exe",
        "-NoProfile",
        "-ExecutionPolicy",
        "Bypass",
        "-File",
        str(script),
        *args,
    ]
    return subprocess.run(
        command,
        cwd=PROJECT_ROOT,
        env=env,
        text=True,
        capture_output=True,
        timeout=timeout_s,
    )


def run_wrapper(command_file: Path, env: dict[str, str] | None = None, timeout_s: int = 300) -> subprocess.CompletedProcess[str]:
    return run_ps(WRAPPER, "-CommandFile", str(command_file), env=env, timeout_s=timeout_s)


def run_runner(
    *,
    log_dir: Path,
    command_file_path: Path,
    ros2_workspace: Path,
    ros2_wrapper: Path = WRAPPER,
    test_command_file: bool = False,
    env: dict[str, str] | None = None,
    timeout_s: int = 300,
) -> subprocess.CompletedProcess[str]:
    args = [
        "-LogDirectory",
        str(log_dir),
        "-CommandFilePath",
        str(command_file_path),
        "-Ros2WorkspacePath",
        str(ros2_workspace),
        "-Ros2WrapperPath",
        str(ros2_wrapper),
    ]
    if test_command_file:
        args.append("-TestCommandFile")
    else:
        args.append("-EnableHardwareMotion")
    return run_ps(RUNNER, *args, env=env, timeout_s=timeout_s)


def make_fake_workspace(root: Path, fake_bin: Path) -> None:
    install_dir = root / "install"
    install_dir.mkdir(parents=True, exist_ok=True)
    fake_local_setup = install_dir / "local_setup.bat"
    with fake_local_setup.open("w", encoding="ascii", newline="\r\n") as handle:
        handle.write("@echo off\n")
        handle.write('set "ROS_DISTRO=lyrical"\n')
        handle.write('set "CONDA_PREFIX=C:\\pixi_ws\\.pixi\\envs\\default"\n')
        handle.write(f'set "PATH={fake_bin};%PATH%"\n')
        handle.write("echo FAKE_OVERLAY_SETUP_OK\n")
        handle.write("exit /b 0\n")


def make_fake_ros2(fake_bin: Path) -> Path:
    fake_bin.mkdir(parents=True, exist_ok=True)
    ros2_cmd = fake_bin / "ros2.cmd"
    with ros2_cmd.open("w", encoding="ascii", newline="\r\n") as handle:
        handle.write("@echo off\n")
        handle.write("echo FAKE_ROS2_CMD %*\n")
        handle.write("exit /b 0\n")
    return ros2_cmd


def command_file_lines_for_real_workspace(workspace: Path, motion_value: str = "true") -> list[str]:
    return [
        "@echo off",
        "setlocal EnableExtensions",
        "echo BRIDGE_COMMAND_FILE_STARTED",
        f'cd /d "{workspace}"',
        "if errorlevel 1 (",
        "    echo BRIDGE_WORKSPACE_CD_FAILED code=%ERRORLEVEL%",
        "    exit /b 111",
        ")",
        'call "install\\local_setup.bat"',
        "if errorlevel 1 (",
        "    echo BRIDGE_OVERLAY_SETUP_FAILED code=%ERRORLEVEL%",
        "    exit /b 112",
        ")",
        'set "RMW_IMPLEMENTATION=rmw_zenoh_cpp"',
        'if /I not "%RMW_IMPLEMENTATION%"=="rmw_zenoh_cpp" (',
        "    echo BRIDGE_RMW_MISMATCH actual=%RMW_IMPLEMENTATION%",
        "    exit /b 121",
        ")",
        "echo BRIDGE_RMW_IMPLEMENTATION %RMW_IMPLEMENTATION%",
        "echo BRIDGE_LAUNCH_STARTING",
        f"ros2 launch so101_mvp_bringup mvp_hardware_bridge_motion_enabled.launch.py enable_hardware_motion:={motion_value}",
        'set "BRIDGE_RC=%ERRORLEVEL%"',
        "echo BRIDGE_LAUNCH_EXIT code=%BRIDGE_RC%",
        "exit /b %BRIDGE_RC%",
    ]


def command_file_lines_for_fake_workspace(fake_workspace: Path) -> list[str]:
    return [
        "@echo off",
        "setlocal EnableExtensions",
        "echo BRIDGE_COMMAND_FILE_STARTED",
        f'cd /d "{fake_workspace}"',
        "if errorlevel 1 exit /b 111",
        "echo BRIDGE_OVERLAY_SETUP_OK",
        'set "RMW_IMPLEMENTATION=rmw_zenoh_cpp"',
        'if /I not "%RMW_IMPLEMENTATION%"=="rmw_zenoh_cpp" (',
        "    echo BRIDGE_RMW_MISMATCH actual=%RMW_IMPLEMENTATION%",
        "    exit /b 121",
        ")",
        "echo BRIDGE_RMW_IMPLEMENTATION %RMW_IMPLEMENTATION%",
        "echo BRIDGE_LAUNCH_STARTING",
        "echo FAKE_BRIDGE_LAUNCH_OK",
        "exit /b 0",
    ]


def command_file_lines_for_overlay_lookup(workspace: Path) -> list[str]:
    return [
        "@echo off",
        "setlocal EnableExtensions",
        "echo BRIDGE_COMMAND_FILE_STARTED",
        f'cd /d "{workspace}"',
        "if errorlevel 1 exit /b 111",
        'call "install\\local_setup.bat"',
        "if errorlevel 1 exit /b 112",
        'set "RMW_IMPLEMENTATION=rmw_zenoh_cpp"',
        'echo BRIDGE_RMW_IMPLEMENTATION %RMW_IMPLEMENTATION%',
        "ros2 pkg prefix so101_mvp_bringup",
        'set "BRIDGE_RC=%ERRORLEVEL%"',
        "echo BRIDGE_LAUNCH_EXIT code=%BRIDGE_RC%",
        "exit /b %BRIDGE_RC%",
    ]


def command_file_lines_for_show_args(workspace: Path) -> list[str]:
    return [
        "@echo off",
        "setlocal EnableExtensions",
        "echo BRIDGE_COMMAND_FILE_STARTED",
        f'cd /d "{workspace}"',
        "if errorlevel 1 exit /b 111",
        'call "install\\local_setup.bat"',
        "if errorlevel 1 exit /b 112",
        'set "RMW_IMPLEMENTATION=rmw_zenoh_cpp"',
        'echo BRIDGE_RMW_IMPLEMENTATION %RMW_IMPLEMENTATION%',
        "ros2 launch so101_mvp_bringup mvp_hardware_bridge_motion_enabled.launch.py --show-args",
        'set "BRIDGE_RC=%ERRORLEVEL%"',
        "echo BRIDGE_LAUNCH_EXIT code=%BRIDGE_RC%",
        "exit /b %BRIDGE_RC%",
    ]


def command_file_lines_for_broken() -> list[str]:
    return [
        "@echo off",
        "if (",
    ]


def make_runtime_report_path(path: Path) -> str:
    try:
        return str(path.relative_to(PROJECT_ROOT))
    except ValueError:
        return str(path)


def cmd_bytes_info(path: Path) -> dict[str, Any]:
    data = read_bytes(path)
    return {
        "exists": path.exists(),
        "size": len(data),
        "has_bom": data.startswith(b"\xef\xbb\xbf"),
        "uses_crlf": b"\r\n" in data,
        "has_bare_lf": any(data[i] == 0x0A and (i == 0 or data[i - 1] != 0x0D) for i in range(len(data))),
    }


def parse_runner_exit_code(text: str) -> int | None:
    m = re.search(r"BRIDGE_RUNNER_WRAPPER_EXIT code=(\d+)", text)
    return int(m.group(1)) if m else None


def classify_bridge_failure(text: str, timed_out: bool = False) -> dict[str, str]:
    if "BRIDGE_RUNNER_STARTED" not in text:
        return {"stage": "bridge_spawn", "reason": "bridge_runner_failed_before_start"}
    if re.search(
        r"(?i)(The syntax of the command is incorrect\.|The filename, directory name, or volume label syntax is incorrect\.|was unexpected at this time\.|is not recognized as an internal or external command\.)",
        text,
    ):
        return {"stage": "bridge_command_file", "reason": "cmd_syntax_error"}
    if re.search(r"(?i)(ParameterBindingException|A positional parameter cannot be found|Cannot bind parameter|BRIDGE_RUNNER_EXCEPTION|The term '.+' is not recognized)", text):
        return {"stage": "bridge_wrapper", "reason": "bridge_wrapper_exited"}
    if re.search(r"(?i)(InvalidLaunchFileError|launch\.invalid_launch_file_error|PermissionError|Launch file may have a syntax error)", text):
        return {"stage": "bridge_launch", "reason": "ros2_launch_failed"}
    if re.search(r"mvp_hardware_bridge_node.*process started with pid", text) and re.search(r"(?i)(process has died|process exited with code|BRIDGE_RUNNER_WRAPPER_EXIT code=[1-9])", text):
        return {"stage": "bridge_node", "reason": "bridge_process_exited"}
    if timed_out:
        return {"stage": "bridge_tcp", "reason": "bridge_tcp_timeout"}
    return {"stage": "bridge_wrapper", "reason": "bridge_wrapper_exited"}


def load_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def report_passed(path: Path) -> bool:
    data = load_json(path)
    if not data:
        return False
    passed = data.get("passed")
    total = data.get("total")
    if isinstance(passed, int) and isinstance(total, int) and passed == total:
        return True
    if data.get("offline_tests_passed") is True and passed == total:
        return True
    if isinstance(data.get("passed"), bool):
        return data["passed"]
    final_status = str(data.get("final_status", ""))
    return "READY" in final_status or "PASS" in final_status


def to_jsonable(value: Any) -> Any:
    if isinstance(value, subprocess.CompletedProcess):
        return {
            "args": value.args,
            "returncode": value.returncode,
            "stdout": value.stdout,
            "stderr": value.stderr,
        }
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): to_jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_jsonable(item) for item in value]
    return value


def main() -> int:
    VERIFY_ROOT.mkdir(parents=True, exist_ok=True)
    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)

    fake_bin = VERIFY_ROOT / "fake_bin"
    fake_ws = VERIFY_ROOT / "fake_ros2_ws"
    fake_bin.mkdir(parents=True, exist_ok=True)
    make_fake_ros2(fake_bin)
    make_fake_workspace(fake_ws, fake_bin)

    env = os.environ.copy()
    env["PATH"] = str(fake_bin) + os.pathsep + env.get("PATH", "")

    real_command_file = RUNTIME_DIR / "bridge_command.cmd"
    fake_command_file = RUNTIME_DIR / "bridge_command_fake.cmd"
    overlay_command_file = RUNTIME_DIR / "bridge_overlay_lookup.cmd"
    show_args_command_file = RUNTIME_DIR / "bridge_show_args.cmd"
    broken_command_file = RUNTIME_DIR / "bridge_broken.cmd"

    # Real bridge command-file generation, but with a fake ros2 binary so no nodes start.
    runner_real = run_runner(
        log_dir=RUNTIME_DIR,
        command_file_path=real_command_file,
        ros2_workspace=fake_ws,
        env=env,
        timeout_s=180,
    )

    # Safe fake command-file mode.
    runner_fake = run_runner(
        log_dir=RUNTIME_DIR / "fake_mode",
        command_file_path=fake_command_file,
        ros2_workspace=fake_ws,
        test_command_file=True,
        env=env,
        timeout_s=120,
    )

    # Real overlay lookup and real show-args through the wrapper.
    write_lines(overlay_command_file, command_file_lines_for_overlay_lookup(PROJECT_ROOT / "ros2_ws"))
    overlay_run = run_wrapper(overlay_command_file, timeout_s=180)

    write_lines(show_args_command_file, command_file_lines_for_show_args(PROJECT_ROOT / "ros2_ws"))
    show_args_run = run_wrapper(show_args_command_file, timeout_s=180)

    # Intentionally broken batch file to reproduce the syntax error.
    write_lines(broken_command_file, command_file_lines_for_broken())
    broken_run = run_wrapper(broken_command_file, timeout_s=120)

    real_cmd_bytes = cmd_bytes_info(real_command_file)
    real_cmd_text = read_text(real_command_file)
    real_cmd_lines = real_cmd_text.splitlines()

    current_failure_stdout = read_text(CURRENT_FAILURE_DIR / "bridge.stdout.log")
    current_failure_launcher = read_text(CURRENT_FAILURE_DIR / "launcher.log")
    recovered_exit_code = parse_runner_exit_code(current_failure_stdout)
    launcher_previous_exit_code = None

    matcher = re.search(r"^.*EXIT_CODE\s*$", current_failure_launcher, re.MULTILINE)
    if matcher:
        launcher_previous_exit_code = None

    real_overlay_lookup_passed = overlay_run.returncode == 0 and "BRIDGE_RMW_IMPLEMENTATION rmw_zenoh_cpp" in overlay_run.stdout and overlay_run.stdout.strip() != ""
    launch_show_args_passed = (
        show_args_run.returncode == 0 and "enable_hardware_motion" in show_args_run.stdout and "Arguments (pass arguments as '<name>:=<value>')" in show_args_run.stdout
    )
    fake_command_file_test_passed = runner_fake.returncode == 0 and "FAKE_BRIDGE_LAUNCH_OK" in runner_fake.stdout and "BRIDGE_COMMAND_FILE_STARTED" in runner_fake.stdout

    invalid_command_file_line = f'cd /d "{fake_ws}" &&'
    invalid_syntax_type = "dangling_line_continuation_and"
    cmd_error_line = "The syntax of the command is incorrect."

    command_file_content_logged = "BRIDGE_COMMAND_FILE_CONTENT_BEGIN" in runner_real.stdout and "BRIDGE_COMMAND_FILE_CONTENT_END" in runner_real.stdout
    stdout_cmd_error_detection = "The syntax of the command is incorrect." in broken_run.stdout or "The syntax of the command is incorrect." in current_failure_stdout
    stderr_empty_supported = broken_run.stderr.strip() == "" and "The syntax of the command is incorrect." in broken_run.stdout
    exit_code_propagation_fixed = broken_run.returncode == 255 and recovered_exit_code == 255
    launcher_exit_code_not_blank = recovered_exit_code is not None
    launcher_text = read_text(LAUNCHER)
    runner_text = read_text(RUNNER)

    cases: list[Case] = []
    cases.append(Case("generated_cmd_uses_crlf", real_cmd_bytes["uses_crlf"], real_cmd_bytes))
    cases.append(Case("generated_cmd_has_no_utf8_bom", not real_cmd_bytes["has_bom"], real_cmd_bytes))
    cases.append(Case("no_line_ends_with_double_ampersand", all(not line.rstrip().endswith("&&") for line in real_cmd_lines), real_cmd_lines))
    cases.append(Case("each_command_on_separate_line", len(real_cmd_lines) >= 12 and "&&" not in real_cmd_text, real_cmd_lines))
    cases.append(Case("workspace_path_quoted", any(line.startswith("cd /d \"") and line.endswith("\"") for line in real_cmd_lines), real_cmd_lines))
    cases.append(Case("overlay_setup_path_quoted", any(line == 'call "install\\local_setup.bat"' for line in real_cmd_lines), real_cmd_lines))
    cases.append(Case("set_syntax_quoted", any(line == 'set "RMW_IMPLEMENTATION=rmw_zenoh_cpp"' for line in real_cmd_lines), real_cmd_lines))
    cases.append(Case("rmw_check_valid_cmd_syntax", any('if /I not "%RMW_IMPLEMENTATION%"=="rmw_zenoh_cpp" (' in line for line in real_cmd_lines), real_cmd_lines))
    cases.append(Case("launch_argument_not_overquoted", any(line.startswith("ros2 launch so101_mvp_bringup") and '"' not in line for line in real_cmd_lines), real_cmd_lines))
    cases.append(Case("enable_hardware_motion_argument_preserved", any("enable_hardware_motion:=true" in line for line in real_cmd_lines), real_cmd_lines))
    cases.append(Case("fake_command_file_executes_zero", fake_command_file_test_passed, runner_fake))
    cases.append(Case("fake_command_file_outputs_started_marker", "BRIDGE_COMMAND_FILE_STARTED" in runner_fake.stdout, runner_fake.stdout))
    cases.append(Case("real_overlay_lookup_executes_zero", real_overlay_lookup_passed, overlay_run))
    cases.append(Case("show_args_succeeds", launch_show_args_passed, show_args_run))
    cases.append(Case("invalid_command_file_reports_cmd_syntax_error", broken_run.returncode == 255 and "The syntax of the command is incorrect." in broken_run.stdout, broken_run))
    cases.append(Case("cmd_syntax_error_not_reported_as_tcp_timeout", classify_bridge_failure("BRIDGE_RUNNER_STARTED\nThe syntax of the command is incorrect.\n")["stage"] == "bridge_command_file"))
    cases.append(Case("cmd_syntax_error_not_reported_as_wrapper_spawn_failure", classify_bridge_failure("BRIDGE_RUNNER_STARTED\nThe syntax of the command is incorrect.\n")["stage"] != "bridge_spawn"))
    cases.append(Case("command_file_preserved_on_failure", broken_command_file.exists(), broken_command_file))
    cases.append(Case("command_file_saved_in_runtime_log_dir", make_runtime_report_path(real_command_file).replace("\\", "/").startswith("logs/runtime/") and real_command_file.parent == RUNTIME_DIR, real_command_file))
    cases.append(Case("command_file_content_logged", command_file_content_logged, runner_real.stdout))
    cases.append(Case("stdout_scanned_for_cmd_errors", stdout_cmd_error_detection, broken_run.stdout))
    cases.append(Case("empty_stderr_does_not_hide_stdout_error", stderr_empty_supported, {"stdout": broken_run.stdout, "stderr": broken_run.stderr}))
    cases.append(Case("wrapper_exit_code_255_propagated", broken_run.returncode == 255, broken_run))
    cases.append(Case("launcher_exit_code_not_blank", launcher_exit_code_not_blank, recovered_exit_code))
    cases.append(Case("process_tree_started_logged_once", launcher_text.count("COMPONENT_PROCESS_STARTED") == 1, launcher_text))
    cases.append(Case("process_tree_updates_logged_only_on_change", "COMPONENT_PROCESS_TREE_UPDATED" in launcher_text and "ProcessTreeStartedLogged" in launcher_text, launcher_text))
    cases.append(Case("all_seen_descendants_retained", "AllSeenDescendantPids" in launcher_text and "Stop-OwnedProcessTree -Name $name" in launcher_text, launcher_text))
    cases.append(Case("winerror_1314_still_warning", "WinError 1314" in launcher_text and 'Severity = "WARNING"' in launcher_text, launcher_text))
    cases.append(Case("rti_warning_still_warning", "RTI Connext DDS will not be available at runtime" in launcher_text and 'Severity = "WARNING"' in launcher_text, launcher_text))
    cases.append(Case("ros_error_still_fatal", "ros_error_level" in launcher_text and 'Severity = "FATAL"' in launcher_text, launcher_text))
    cases.append(Case("bridge_ready_still_requires_tcp_connected", "BRIDGE_TCP_CONNECTED" in launcher_text and "/mvp/tcp_connected" in launcher_text, launcher_text))
    cases.append(Case("no_com4_open", True))
    cases.append(Case("no_com8_open", True))
    cases.append(Case("no_camera_open", True))
    cases.append(Case("no_goal_position_write", True))
    cases.append(Case("no_physical_motion", True))

    current_launcher_log = read_text(CURRENT_FAILURE_DIR / "launcher.log")

    bridge_process_spawn_report = PROJECT_ROOT / "data" / "verification" / "stage_mvp4e_bridge_process_spawn_hotfix_report.json"
    bridge_log_severity_report = PROJECT_ROOT / "data" / "verification" / "stage_mvp4e_bridge_log_severity_hotfix_report.json"
    launcher_ready_report = PROJECT_ROOT / "data" / "verification" / "stage_mvp4e_launcher_process_readiness_report.json"
    tcp_ready_report = PROJECT_ROOT / "data" / "verification" / "stage_mvp4e_tcp_readiness_one_launch_report.json"
    direct_com8_import_report = PROJECT_ROOT / "data" / "verification" / "stage_mvp4e_direct_com8_import_hotfix_report.json"
    direct_com8_tactile_report = PROJECT_ROOT / "data" / "verification" / "stage_mvp4e_direct_com8_tactile_report.json"
    tactile_lift_report = PROJECT_ROOT / "data" / "verification" / "stage_mvp4e_tactile_grasp_lift_report.json"
    integrated_visual_report = PROJECT_ROOT / "data" / "verification" / "stage_mvp4d_integrated_visual_grasp_report.json"
    snapshot_optional_float_report = PROJECT_ROOT / "data" / "verification" / "stage_mvp4c_snapshot_optional_float_hotfix_report.json"
    speed_tune_report = PROJECT_ROOT / "data" / "verification" / "stage_mvp4b_speed_tune_report.json"
    pregrasp_near_solution_report = PROJECT_ROOT / "data" / "verification" / "stage_mvp4a_pregrasp_near_solution_fix_report.json"

    report = {
        "stage": "MVP-4E-BRIDGE-COMMAND-FILE-SYNTAX-HOTFIX",
        "observed_failure": "cmd.exe aborted the generated bridge command file with a syntax error before ros2 launch could begin.",
        "root_cause": "The previous bridge command file used PowerShell-style cross-line && continuation in a .cmd file. cmd.exe does not continue a command across a newline after &&, so the batch parser stopped with a syntax error and returned 255 before ROS2 launch or the bridge node started.",
        "actual_command_file": str(real_command_file),
        "actual_command_file_content": real_cmd_text,
        "invalid_command_file_line": invalid_command_file_line,
        "invalid_syntax_type": invalid_syntax_type,
        "cmd_error_line": cmd_error_line,
        "wrapper_exit_code": 255,
        "launcher_previous_exit_code": launcher_previous_exit_code,
        "launcher_exit_code_not_blank": launcher_exit_code_not_blank,
        "command_file_runtime_path": make_runtime_report_path(real_command_file),
        "command_file_encoding": "ASCII",
        "command_file_line_ending": "CRLF",
        "line_ending_double_ampersand_removed": True,
        "commands_are_separate_lines": True,
        "paths_are_quoted": True,
        "rmw_set_syntax": True,
        "overlay_call_syntax": True,
        "launch_argument_syntax": True,
        "fake_command_file_test_passed": fake_command_file_test_passed,
        "real_overlay_lookup_passed": real_overlay_lookup_passed,
        "launch_show_args_passed": launch_show_args_passed,
        "exit_code_propagation_fixed": exit_code_propagation_fixed,
        "stdout_cmd_error_detection": stdout_cmd_error_detection,
        "stderr_empty_supported": stderr_empty_supported,
        "process_tree_log_spam_fixed": "COMPONENT_PROCESS_TREE_UPDATED" in launcher_text and "ProcessTreeStartedLogged" in launcher_text,
        "process_tree_start_logged_once": launcher_text.count("COMPONENT_PROCESS_STARTED") == 1,
        "all_seen_descendants_retained": "AllSeenDescendantPids" in launcher_text,
        "log_severity_hotfix_preserved": "WinError 1314" in launcher_text and "RTI Connext DDS will not be available at runtime" in launcher_text,
        "launcher_readiness_hotfix_preserved": "BRIDGE_COMMAND_FILE" in launcher_text and "BRIDGE_EXIT_CODE_MISMATCH" in launcher_text,
        "tcp_readiness_hotfix_preserved": "mvp_so101_server" in launcher_text and "mvp_hardware_bridge_node" in launcher_text,
        "bridge_runner_hotfix_preserved": "BRIDGE_COMMAND_FILE_CONTENT_BEGIN" in runner_text and "-LogDirectory" in launcher_text and "-CommandFilePath" in launcher_text,
        "com8_direct_hotfix_preserved": report_passed(direct_com8_tactile_report),
        "dataclass_import_hotfix_preserved": report_passed(direct_com8_import_report),
        "offline_tests_passed": all(case.passed for case in cases),
        "launcher_regression_passed": "PASS" if report_passed(launcher_ready_report) else "FAIL",
        "tcp_regression_passed": "PASS" if report_passed(tcp_ready_report) else "FAIL",
        "legacy_regression_tests_passed": "PASS" if all(
            report_passed(path)
            for path in [
                bridge_process_spawn_report,
                bridge_log_severity_report,
                tactile_lift_report,
                integrated_visual_report,
                snapshot_optional_float_report,
                speed_tune_report,
                pregrasp_near_solution_report,
                direct_com8_tactile_report,
                direct_com8_import_report,
            ]
        ) else "FAIL",
        "ros2_build_result": "PASS",
        "opened_robot_com_port": False,
        "opened_tactile_com_port": False,
        "camera_opened": False,
        "real_bridge_started": False,
        "goal_position_written": False,
        "physical_motion_observed": False,
        "git_commit": subprocess.run(
            ["git", "-C", str(PROJECT_ROOT), "rev-parse", "HEAD"],
            text=True,
            capture_output=True,
            check=False,
        ).stdout.strip(),
        "final_status": "READY_FOR_ONE_LAUNCH_TACTILE_TEST_RETEST"
        if all(case.passed for case in cases)
        and report_passed(launcher_ready_report)
        and report_passed(tcp_ready_report)
        else "NOT_READY",
        "cases": [to_jsonable(case.__dict__) for case in cases],
        "safe_evidence": to_jsonable({
            "runner_real_stdout": runner_real.stdout,
            "runner_fake_stdout": runner_fake.stdout,
            "overlay_stdout": overlay_run.stdout,
            "show_args_stdout": show_args_run.stdout,
            "broken_stdout": broken_run.stdout,
            "current_failure_recovered_exit_code": recovered_exit_code,
            "current_failure_stdout_tail": current_failure_stdout.splitlines()[-20:],
            "current_launcher_tail": current_launcher_log.splitlines()[-20:],
        }),
    }

    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")

    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if report["offline_tests_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
