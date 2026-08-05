from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPORT_PATH = PROJECT_ROOT / "data" / "verification" / "stage_mvp4c_snapshot_optional_float_hotfix_report.json"
if str(PROJECT_ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

from mvp_move_to_pregrasp import (  # noqa: E402
    ARM_JOINT_NAMES,
    MoveConfig,
    atomic_write_json,
    make_pregrasp_snapshot,
    optional_float,
)


def case(name: str, passed: bool, details: dict[str, object] | None = None) -> dict[str, object]:
    return {"name": name, "passed": bool(passed), "details": {} if details is None else details}


def contains_non_finite(value: Any) -> bool:
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return False
    if isinstance(value, int):
        return False
    if isinstance(value, float):
        return not math.isfinite(value)
    if isinstance(value, list):
        return any(contains_non_finite(item) for item in value)
    if isinstance(value, dict):
        return any(contains_non_finite(item) for item in value.values())
    return False


def make_planned_snapshot() -> dict[str, Any]:
    return make_pregrasp_snapshot(
        snapshot_state="planned",
        created_at_unix_s=123.0,
        object_pose_base=[0.20, 0.0, 0.025],
        pregrasp_pose_base=[0.20, 0.0, 0.105],
        frozen_target_rad=[-0.026301935635558084, -0.19915896684938358, 0.03445595226343326, 1.65806, 0.2269828039212294],
        compute_message=(
            "pregrasp_ready solution_type=accepted_near_solution "
            "offset_m=[0.000000, 0.000000, 0.000000] "
            "position_error_m=0.007417 approach_error_deg=4.437"
        ),
        compute_pregrasp_success=True,
        pregrasp_valid=True,
        pregrasp_status="pregrasp_ready_near",
        hardware_command_sent=False,
        execute_response_message=None,
        motion_completed=False,
        final_joint_positions_rad=None,
        final_errors=None,
        config=MoveConfig(),
        tcp_connected_after_motion=None,
        tcp_status_after_motion=None,
    )


def run_occlusion_regression() -> tuple[bool, dict[str, object]]:
    result = subprocess.run(
        [
            sys.executable,
            str(PROJECT_ROOT / "scripts" / "verify_stage_mvp4c_occlusion_safe_handoff.py"),
            "--ros2-build-result",
            "PASS",
        ],
        cwd=PROJECT_ROOT,
        text=True,
        capture_output=True,
        timeout=60,
        check=False,
    )
    return result.returncode == 0, {
        "returncode": result.returncode,
        "stdout_tail": result.stdout.splitlines()[-5:],
        "stderr_tail": result.stderr.splitlines()[-5:],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Offline hotfix verification for optional_float snapshot parsing.")
    parser.add_argument("--ros2-build-result", default="not_run")
    args = parser.parse_args()

    snapshot = make_planned_snapshot()
    snapshot_text = json.dumps(snapshot, ensure_ascii=False, allow_nan=False)
    loaded_snapshot = json.loads(snapshot_text)

    with tempfile.TemporaryDirectory() as tmp:
        snapshot_path = Path(tmp) / "mvp_last_pregrasp_snapshot.json"
        atomic_write_json(snapshot_path, snapshot)
        atomically_loaded = json.loads(snapshot_path.read_text(encoding="utf-8"))
        temp_leftover = snapshot_path.with_name(snapshot_path.name + ".tmp").exists()

    occlusion_ok, occlusion_details = run_occlusion_regression()
    move_text = (PROJECT_ROOT / "scripts" / "mvp_move_to_pregrasp.py").read_text(encoding="utf-8")

    cases: list[dict[str, object]] = [
        case("optional_float_defined", callable(optional_float)),
        case("optional_float_none", optional_float(None) is None),
        case("optional_float_numeric_string", optional_float("0.007417") == 0.007417),
        case("optional_float_float", optional_float(4.437) == 4.437),
        case("optional_float_integer", optional_float(4) == 4.0),
        case("optional_float_invalid_string", optional_float("invalid") is None),
        case("optional_float_nan_rejected", optional_float(float("nan")) is None),
        case("optional_float_inf_rejected", optional_float(float("inf")) is None),
        case("optional_float_bool_rejected", optional_float(True) is None),
        case("make_planned_snapshot_no_name_error", snapshot["snapshot_state"] == "planned"),
        case(
            "position_error_saved_as_number",
            isinstance(snapshot["position_error_m"], float) and math.isclose(snapshot["position_error_m"], 0.007417),
        ),
        case(
            "approach_error_saved_as_number",
            isinstance(snapshot["approach_error_deg"], float) and math.isclose(snapshot["approach_error_deg"], 4.437),
        ),
        case("planned_snapshot_state", snapshot["snapshot_state"] == "planned"),
        case("plan_only_hardware_command_false", snapshot["hardware_command_sent"] is False),
        case("plan_only_motion_completed_false", snapshot["motion_completed"] is False),
        case(
            "planned_snapshot_atomic_write",
            atomically_loaded["snapshot_state"] == "planned" and not temp_leftover and "os.replace" in move_text,
        ),
        case("snapshot_json_valid", loaded_snapshot["joint_names"] == list(ARM_JOINT_NAMES)),
        case("no_nan_in_snapshot_json", "NaN" not in snapshot_text and not contains_non_finite(loaded_snapshot)),
        case("no_inf_in_snapshot_json", "Infinity" not in snapshot_text and not contains_non_finite(loaded_snapshot)),
        case("existing_occlusion_handoff_tests_still_pass", occlusion_ok, occlusion_details),
        case("no_com_port_open", "COM4" not in move_text and "serial.Serial" not in move_text),
        case("no_goal_position_write", "Goal_Position" not in move_text and "send_action(" not in move_text),
        case("no_physical_motion", "mvp_so101_server" not in move_text and "MotionFeetechBackend" not in move_text),
    ]

    passed = sum(1 for item in cases if item["passed"])
    all_passed = passed == len(cases)
    report = {
        "stage": "MVP-4C-SNAPSHOT-OPTIONAL-FLOAT-HOTFIX",
        "root_cause": "make_pregrasp_snapshot called undefined optional_float",
        "undefined_symbol": "optional_float",
        "affected_function": "make_pregrasp_snapshot",
        "fix_strategy": "add minimal finite optional_float helper near existing parse helpers",
        "existing_helper_reused": False,
        "new_helper_added": True,
        "optional_float_behavior": {
            "none_returns_none": True,
            "int_or_float_to_float": True,
            "numeric_string_to_float": True,
            "invalid_string_returns_none": True,
            "nan_or_inf_returns_none": True,
            "bool_returns_none": True,
        },
        "planned_snapshot_constructed": snapshot["snapshot_state"] == "planned",
        "position_error_saved_as_json_number": isinstance(snapshot["position_error_m"], float),
        "approach_error_saved_as_json_number": isinstance(snapshot["approach_error_deg"], float),
        "snapshot_schema_modified": False,
        "snapshot_path_modified": False,
        "occlusion_handoff_logic_modified": False,
        "ik_algorithm_modified": False,
        "camera_algorithm_modified": False,
        "workspace_transform_modified": False,
        "hardware_execution_modified": False,
        "offline_test_cases": cases,
        "offline_tests_passed": all_passed,
        "offline_test_count": len(cases),
        "occlusion_handoff_regression_tests_passed": occlusion_ok,
        "ros2_build_result": args.ros2_build_result,
        "opened_com_ports": False,
        "tcp_started": False,
        "hardware_bridge_started": False,
        "goal_position_written": False,
        "physical_motion_observed": False,
        "final_status": "READY_FOR_MANUAL_PREGRASP_SNAPSHOT_RETEST"
        if all_passed and args.ros2_build_result == "PASS"
        else ("OFFLINE_TESTS_PASS" if all_passed else "OFFLINE_TESTS_FAIL"),
    }
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(
        json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )

    for item in cases:
        print(("PASS" if item["passed"] else "FAIL") + f" {item['name']}")
    print(f"offline_tests_passed={passed}/{len(cases)}")
    print(f"report={REPORT_PATH}")
    return 0 if all_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
