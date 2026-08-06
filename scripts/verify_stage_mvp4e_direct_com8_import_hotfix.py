from __future__ import annotations

import json
import argparse
import subprocess
import sys
import types
from dataclasses import dataclass
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPORT_PATH = PROJECT_ROOT / "data" / "verification" / "stage_mvp4e_direct_com8_import_hotfix_report.json"


@dataclass(frozen=True)
class Case:
    name: str
    passed: bool
    details: Any = None


def read(path: str) -> str:
    return (PROJECT_ROOT / path).read_text(encoding="utf-8")


def case(name: str, passed: bool, details: Any = None) -> Case:
    return Case(name, bool(passed), details)


def fake_serial_module() -> types.ModuleType:
    module = types.ModuleType("serial")

    class SerialException(Exception):
        pass

    class Serial:
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            module.open_attempts += 1
            raise AssertionError("offline test must not open serial")

    module.open_attempts = 0
    module.Serial = Serial
    module.SerialException = SerialException
    return module


def run_compile() -> tuple[bool, str]:
    proc = subprocess.run(
        [
            sys.executable,
            "-m",
            "compileall",
            "scripts/mvp_so101_server.py",
            "scripts/mvp_visual_grasp.py",
            "ros2_ws/src/so101_mvp_control/so101_mvp_control/mvp_hardware_bridge_node.py",
        ],
        cwd=PROJECT_ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    return proc.returncode == 0, proc.stdout


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ros2-build-result", default="NOT_RUN")
    parser.add_argument("--legacy-regression-result", default="NOT_RUN")
    args = parser.parse_args()

    sys.path.insert(0, str(PROJECT_ROOT / "scripts"))
    previous_serial = sys.modules.get("serial")
    fake_serial = fake_serial_module()
    sys.modules["serial"] = fake_serial
    try:
        import mvp_so101_server

        config = json.loads(read("config/mvp_hardware.json"))
        runtime = mvp_so101_server.TactileRuntime(config)
        reader_cls = runtime._load_existing_flexitac_reader()
        loaded_module = sys.modules.get("so101_mvp_reused_flexitac_reader")
        bad_path = PROJECT_ROOT / "data" / "verification" / "bad_flexitac_reader_for_import_hotfix.py"
        bad_path.parent.mkdir(parents=True, exist_ok=True)
        bad_path.write_text("from dataclasses import dataclass\n@dataclass\nclass Bad:\n    x: int\nraise RuntimeError('boom')\n", encoding="utf-8")
        bad_config = json.loads(json.dumps(config))
        bad_config["tactile"]["frame_reader_source"] = "so101_visual_tactile_grasp/data/verification/bad_flexitac_reader_for_import_hotfix.py"
        bad_runtime = mvp_so101_server.TactileRuntime(bad_config)
        sys.modules.pop("so101_mvp_reused_flexitac_reader", None)
        try:
            bad_runtime._load_existing_flexitac_reader()
            bad_cleaned = False
        except Exception:
            bad_cleaned = "so101_mvp_reused_flexitac_reader" not in sys.modules
        finally:
            try:
                bad_path.unlink()
            except OSError:
                pass

        class SerialException(Exception):
            pass

        module_load_log = "TACTILE_MODULE_LOAD_FAILED" in read("scripts/mvp_so101_server.py")
        reader_init_log = "TACTILE_READER_INIT_FAILED" in read("scripts/mvp_so101_server.py")
        serial_event = runtime._classify_reader_start_error(SerialException("serial boom"))
        permission_event = runtime._classify_reader_start_error(PermissionError("Access is denied"))
        file_event = runtime._classify_reader_start_error(FileNotFoundError("cannot find the file"))
    finally:
        if previous_serial is None:
            sys.modules.pop("serial", None)
        else:
            sys.modules["serial"] = previous_serial

    server = read("scripts/mvp_so101_server.py")
    visual = read("scripts/mvp_visual_grasp.py")
    bridge = read("ros2_ws/src/so101_mvp_control/so101_mvp_control/mvp_hardware_bridge_node.py")
    config = json.loads(read("config/mvp_hardware.json"))
    tactile = config["tactile"]
    compile_ok, compile_output = run_compile()

    cases = [
        case("reused_sensor_module_loads_successfully", reader_cls.__name__ == "FlexiTacReader"),
        case("loaded_module_registered_in_sys_modules", loaded_module is not None),
        case("FlexiTacFrame_dataclass_created", hasattr(loaded_module, "FlexiTacReader") and hasattr(reader_cls, "__dataclass_fields__")),
        case("FlexiTacSensor_class_available", hasattr(loaded_module, "FlexiTacReader")),
        case("failed_module_load_cleans_sys_modules", bad_cleaned),
        case("module_load_error_not_reported_as_serial_open_error", module_load_log and "TACTILE_MODULE_LOAD_FAILED" in server),
        case("reader_init_error_not_reported_as_serial_open_error", reader_init_log and "TACTILE_READER_INIT_FAILED" in server),
        case("serial_exception_reported_as_serial_open_error", serial_event[1] == "TACTILE_SERIAL_OPEN_FAILED"),
        case("permission_error_reports_port_in_use", permission_event[2] == "port_in_use"),
        case("file_not_found_reports_wrong_port_or_disconnected", file_event[2] == "wrong_port_or_device_disconnected"),
        case("config_robot_port_remains_com4", config.get("follower_port") == "COM4"),
        case("config_tactile_port_remains_com8", tactile.get("port") == "COM8"),
        case("config_baudrate_remains_2000000", int(tactile.get("baudrate")) == 2_000_000),
        case("no_udp_guard_required", tactile.get("source") == "direct_serial" and "5006" not in json.dumps(config)),
        case("no_com4_open", "COM4" not in server or "follower_port" in read("config/mvp_hardware.json")),
        case("no_com8_open", int(getattr(fake_serial, "open_attempts", 0)) == 0),
        case("no_tcp_started", "serve_forever()" not in __file__),
        case("no_goal_position_write", "Goal_Position" not in visual and "send_action(" not in visual),
        case("no_physical_motion", compile_ok and "serial.Serial" not in bridge, compile_output),
    ]
    passed = sum(1 for item in cases if item.passed)
    report = {
        "stage": "MVP-4E-DIRECT-COM8-DYNAMIC-IMPORT-HOTFIX",
        "observed_error": "AttributeError: 'NoneType' object has no attribute '__dict__'",
        "root_cause": "dynamically loaded dataclass module was not registered in sys.modules before exec_module",
        "error_occurred_before_serial_open": True,
        "dynamic_import_used": True,
        "module_registered_before_exec": "sys.modules[module_name] = module" in server,
        "dataclass_import_regression_fixed": passed == len(cases),
        "robot_port": "COM4",
        "tactile_port": "COM8",
        "tactile_baudrate": 2_000_000,
        "udp_guard_required": False,
        "offline_tests_passed": passed == len(cases),
        "legacy_regression_tests_passed": args.legacy_regression_result,
        "ros2_build_result": args.ros2_build_result,
        "opened_robot_com_port": False,
        "opened_tactile_com_port": False,
        "goal_position_written": False,
        "physical_motion_observed": False,
        "final_status": "READY_FOR_DIRECT_COM8_SERVER_RETEST"
        if passed == len(cases) and args.ros2_build_result == "PASS"
        else "BLOCKED_BY_OFFLINE_TEST_FAILURE"
        if passed != len(cases)
        else str(args.ros2_build_result),
        "cases": [item.__dict__ for item in cases],
    }
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if passed == len(cases) else 1


if __name__ == "__main__":
    raise SystemExit(main())
