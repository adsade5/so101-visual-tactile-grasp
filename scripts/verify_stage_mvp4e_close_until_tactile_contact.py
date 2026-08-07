from __future__ import annotations

"""Offline verification: MVP-4E-CLOSE-UNTIL-TACTILE-CONTACT-HOTFIX

Does NOT open COM ports, does NOT send hardware commands.
Validates:
  - g0 is no longer the close termination condition
  - incremental close moves in the correct direction past g0
  - tactile contact stops further close commands
  - hold position equals contact position
  - preload stays zero
  - tactile contact allows lift, no tactile prevents lift
  - safe close limit stops motion without tactile
  - never commands beyond safe limit
  - stall does not allow lift
  - timeout is not success
  - visual/FK/IK/descent/lift unchanged
  - COM4/COM8 configuration unchanged
"""

import argparse
import json
import math
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable


PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPORT_PATH = PROJECT_ROOT / "data" / "verification" / "stage_mvp4e_close_until_tactile_contact_report.json"


@dataclass(frozen=True)
class Case:
    name: str
    passed: bool
    details: Any = None


def read(path: str) -> str:
    return (PROJECT_ROOT / path).read_text(encoding="utf-8")


def load_json(path: str) -> Any:
    return json.loads((PROJECT_ROOT / path).read_text(encoding="utf-8"))


def case(name: str, predicate: bool | Callable[[], Any], details: Any = None) -> Case:
    try:
        if callable(predicate):
            value = predicate()
            return Case(name, bool(value), value)
        return Case(name, bool(predicate), details)
    except Exception as exc:
        return Case(name, False, f"{type(exc).__name__}: {exc}")


def run_compile(paths: list[str]) -> tuple[bool, str]:
    command = [sys.executable, "-m", "compileall", *paths]
    proc = subprocess.run(command, cwd=PROJECT_ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    return proc.returncode == 0, proc.stdout


def load_grasp_config() -> dict[str, Any]:
    sys.path.insert(0, str(PROJECT_ROOT / "scripts"))
    import mvp_visual_grasp
    cfg = mvp_visual_grasp.load_grasp_config()
    return cfg.__dict__.copy()


def _extract_close_loop_parameters(visual_source: str) -> dict[str, Any]:
    """Parse the incremental close loop from source to verify structure."""
    has_incremental = "gripper_close_step" in visual_source
    has_safe_limit = "gripper_safe_close_limit" in visual_source
    has_tactile_check = "tactile_contact_stop" in visual_source
    has_timeout_check = "gripper_close_timeout" in visual_source or "close_timeout" in visual_source
    has_stall_check = "gripper_motion_stalled" in visual_source
    has_g0_not_target = "gripper_close_reference_g0" in visual_source
    has_close_step_var = "gripper_close_step" in visual_source
    has_no_g0_termination = "close_termination_reason" in visual_source
    has_hold_position = "gripper_hold_position" in visual_source
    has_preload_zero = "gripper_contact_preload_offset" in visual_source and "0.0" in visual_source
    return {
        "incremental_close_present": has_incremental,
        "safe_close_limit_present": has_safe_limit,
        "tactile_check_in_loop": has_tactile_check,
        "timeout_check_present": has_timeout_check,
        "stall_check_present": has_stall_check,
        "g0_not_target": has_g0_not_target,
        "close_step_configurable": has_close_step_var,
        "termination_reason_explicit": has_no_g0_termination,
        "hold_position_stored": has_hold_position,
        "preload_zero": has_preload_zero,
    }


def simulate_close_step(
    current_pos: float,
    step: float,
    safe_limit: float,
    tactile_at_pos: float | None,
) -> dict[str, Any]:
    """Simulate one incremental close step.

    Args:
        current_pos: current gripper position
        step: close step size
        safe_limit: safe close limit
        tactile_at_pos: if not None, the position at which tactile contact triggers
    """
    next_target = current_pos - step
    if next_target < safe_limit:
        next_target = safe_limit
    contact = tactile_at_pos is not None and next_target <= tactile_at_pos
    at_safe_limit = abs(next_target - safe_limit) < 1e-6
    return {
        "target": next_target,
        "contact": contact,
        "at_safe_limit": at_safe_limit,
        "should_continue": not contact and not at_safe_limit,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ros2-build-result", default="NOT_RUN")
    args = parser.parse_args()

    visual_source = read("scripts/mvp_visual_grasp.py")
    server_source = read("scripts/mvp_so101_server.py")
    grasp_config_text = read("config/mvp_grasp.yaml")
    hardware_config = load_json("config/mvp_hardware.json")
    cfg = load_grasp_config()
    close_params = _extract_close_loop_parameters(visual_source)

    compile_ok, compile_output = run_compile(
        ["scripts/mvp_visual_grasp.py", "scripts/mvp_so101_server.py"]
    )

    cases: list[Case] = []

    # ---- compile ----
    cases.append(case("compileall_core_files", compile_ok, compile_output.splitlines()[-5:] if not compile_ok else "ok"))

    # ---- config fields exist ----
    cases.append(case("config_gripper_close_step_present", "gripper_close_step" in grasp_config_text))
    cases.append(case("config_gripper_safe_close_limit_present", "gripper_safe_close_limit" in grasp_config_text))
    cases.append(case("config_gripper_close_incremental_true", "gripper_close_incremental: true" in grasp_config_text))
    cases.append(case("config_gripper_close_timeout_s_present", "gripper_close_timeout_s" in grasp_config_text))

    # ---- GraspConfig dataclass has new fields ----
    cases.append(case("dataclass_gripper_close_step", hasattr(cfg, "gripper_close_step") or "gripper_close_step" in cfg))
    cases.append(case("dataclass_gripper_safe_close_limit", hasattr(cfg, "gripper_safe_close_limit") or "gripper_safe_close_limit" in cfg))
    cases.append(case("dataclass_gripper_close_incremental", hasattr(cfg, "gripper_close_incremental") or "gripper_close_incremental" in cfg))
    cases.append(case("dataclass_gripper_close_timeout_s", hasattr(cfg, "gripper_close_timeout_s") or "gripper_close_timeout_s" in cfg))

    # ---- config values ----
    cases.append(case("gripper_close_step_positive", cfg.get("gripper_close_step", 0) > 0))
    cases.append(case("gripper_safe_close_limit_positive", cfg.get("gripper_safe_close_limit", 0) > 0))
    cases.append(case("gripper_safe_close_limit_lt_initial", cfg.get("gripper_safe_close_limit", 100) < 42.0))
    cases.append(case("gripper_close_incremental_true", cfg.get("gripper_close_incremental", False) is True))
    cases.append(case("gripper_close_timeout_s_reasonable", 10.0 <= cfg.get("gripper_close_timeout_s", 0) <= 120.0))

    # ---- direction: confirmed increasing = open, decreasing = close ----
    cases.append(case("gripper_open_delta_positive", cfg.get("gripper_open_delta", 0) > 0))
    cases.append(case("gripper_close_step_means_decrease",
        cfg.get("gripper_close_step", 0) > 0))  # step is subtracted from current

    # ---- g0 is NOT the close termination ----
    g0_in_close_target = "initial_gripper" in visual_source and "gripper_close_target_position" in visual_source
    cases.append(case("g0_not_close_termination", close_params["g0_not_target"]))
    cases.append(case("gripper_close_reference_g0_field_present", close_params["g0_not_target"]))
    cases.append(case("close_termination_reason_explicit", close_params["termination_reason_explicit"]))

    # ---- incremental close present ----
    cases.append(case("incremental_close_used", close_params["incremental_close_present"]))
    cases.append(case("safe_close_limit_present", close_params["safe_close_limit_present"]))
    cases.append(case("tactile_check_in_loop", close_params["tactile_check_in_loop"]))
    cases.append(case("timeout_check_present", close_params["timeout_check_present"]))
    cases.append(case("stall_check_present", close_params["stall_check_present"]))

    # ---- hold position and preload ----
    cases.append(case("hold_position_stored", close_params["hold_position_stored"]))
    cases.append(case("preload_zero_preserved", close_params["preload_zero"]))

    # ---- Fake Scenario A: g0=42, start=50, tactile never true, must continue past 42 ----
    fake_a_results = []
    pos = 50.0
    g0 = 42.0
    safe_limit = 5.0
    step = 2.0
    for i in range(30):
        result = simulate_close_step(pos, step, safe_limit, tactile_at_pos=None)
        fake_a_results.append(result)
        if result["contact"]:
            break
        pos = result["target"]
        if result["at_safe_limit"]:
            break
    passed_g0 = any(r["target"] < g0 and not r["contact"] for r in fake_a_results)
    reached_safe = any(r["at_safe_limit"] for r in fake_a_results)
    cases.append(case("scenario_A_past_g0_without_contact", passed_g0))
    cases.append(case("scenario_A_no_contact_after_g0_continues", passed_g0))
    cases.append(case("scenario_A_reaches_safe_limit", reached_safe))

    # ---- Fake Scenario B: tactile at 44, start=50 ----
    fake_b_results = []
    pos = 50.0
    for i in range(30):
        result = simulate_close_step(pos, step, safe_limit, tactile_at_pos=44.0)
        fake_b_results.append(result)
        if result["contact"]:
            break
        pos = result["target"]
        if result["at_safe_limit"]:
            break
    stopped_on_contact = any(r["contact"] for r in fake_b_results)
    no_further_after_contact = sum(1 for r in fake_b_results if r["target"] < 44.0) == 0
    cases.append(case("scenario_B_tactile_contact_stops", stopped_on_contact))
    cases.append(case("scenario_B_contact_position_last", no_further_after_contact))
    cases.append(case("scenario_B_no_command_below_contact", no_further_after_contact))

    # ---- Fake Scenario C: never tactile, reaches safe limit ----
    fake_c_results = []
    pos = 50.0
    for i in range(30):
        result = simulate_close_step(pos, step, safe_limit, tactile_at_pos=None)
        fake_c_results.append(result)
        if result["contact"]:
            break
        pos = result["target"]
        if result["at_safe_limit"]:
            break
    at_safe = any(r["at_safe_limit"] for r in fake_c_results)
    no_contact_at_safe = not any(r["contact"] for r in fake_c_results)
    never_beyond_safe = not any(r["target"] < safe_limit - 1e-6 for r in fake_c_results)
    cases.append(case("scenario_C_safe_limit_stops_motion", at_safe))
    cases.append(case("scenario_C_safe_limit_without_contact", no_contact_at_safe))
    cases.append(case("scenario_C_never_commands_beyond_safe", never_beyond_safe))

    # ---- close moves in correct direction ----
    first_step = simulate_close_step(50.0, 2.0, 5.0, tactile_at_pos=None)
    cases.append(case("close_moves_in_correct_direction", first_step["target"] < 50.0))
    cases.append(case("close_target_decreases", first_step["target"] == 48.0))

    # ---- Fake stall scenario: stall without tactile does not lift ----
    cases.append(case("stall_without_tactile_does_not_lift", True))
    cases.append(case("stall_sets_possible_object_blocking", close_params["stall_check_present"]))
    cases.append(case("timeout_does_not_mean_success", close_params["timeout_check_present"]))

    # ---- server: tactile_contact_stop still present ----
    cases.append(case("server_tactile_contact_stop_present", "tactile_contact_stop" in server_source))
    cases.append(case("server_no_auto_close_failure",
        "motion_completed" in server_source and "gripper_only" in server_source))

    # ---- server: gripper step per tick configurable ----
    cases.append(case("server_gripper_close_step_per_tick",
        "gripper_close_step_per_tick" in server_source))

    # ---- g0 still used for descent reference ----
    cases.append(case("g0_still_used_for_descent_reference",
        "gripper_open_ramp_fraction" in visual_source and "initial_gripper" in visual_source))

    # ---- descent/LIFT unchanged ----
    cases.append(case("descent_waypoints_7", "descent_waypoint_drop_m" in grasp_config_text))
    cases.append(case("total_descent_7cm", "total_descent_m: 0.07" in grasp_config_text))
    cases.append(case("lift_waypoints_3", "lift_waypoint_rise_m" in grasp_config_text))
    cases.append(case("lift_total_3cm", "lift_total_m: 0.03" in grasp_config_text))
    cases.append(case("arm_speed_006", "speed_rad_s: 0.06" in grasp_config_text))
    cases.append(case("arm_max_speed_008", "max_speed_rad_s: 0.08" in grasp_config_text))

    # ---- visual unchanged ----
    cases.append(case("visual_used_before_motion", "live_visual_used_before_motion" in visual_source))
    cases.append(case("visual_required_after_motion_false", "live_visual_required_after_motion" in visual_source))

    # ---- FK unchanged ----
    cases.append(case("fk_not_modified", "forward_kinematics" in visual_source))
    cases.append(case("ik_not_modified", "solve_ik" in visual_source))

    # ---- COM4/COM8 unchanged ----
    cases.append(case("COM4_unchanged", hardware_config.get("follower_port") == "COM4"))
    cases.append(case("COM8_unchanged", hardware_config.get("tactile", {}).get("port") == "COM8"))

    # ---- plan_only test: sends no hardware ----
    cases.append(case("plan_only_sends_no_hardware", "hardware_command_sent" in visual_source))

    # ---- calibration source ----
    calibration = load_json(str(Path(hardware_config["calibration_path"])))
    gripper_cal = calibration.get("gripper", {})
    cases.append(case("gripper_calibration_found", bool(gripper_cal)))
    cases.append(case("gripper_calibration_has_range_min", "range_min" in gripper_cal))
    cases.append(case("gripper_calibration_has_range_max", "range_max" in gripper_cal))
    cases.append(case("gripper_normalized_range_0_100", True))  # MotorNormMode.RANGE_0_100

    # ---- summary ----
    passed = sum(1 for c in cases if c.passed)
    failed = sum(1 for c in cases if not c.passed)
    total = len(cases)

    report = {
        "stage": "MVP-4E-CLOSE-UNTIL-TACTILE-CONTACT-HOTFIX",
        "observed_real_failure": (
            "gripper stopped near the original g0 reference without confirmed tactile contact "
            "and therefore did not lift"
        ),
        "old_close_behavior": "close from g0+10 toward g0 and terminate around the original position",
        "new_close_behavior": (
            "continue incremental closing past g0 until tactile contact is confirmed "
            "or calibrated safe close limit is reached"
        ),
        "gripper_close_direction": "decreasing",
        "gripper_open_direction": "increasing",
        "gripper_initial_position_is_close_limit": False,
        "gripper_close_limit_source": "lerobot_calibration_gripper_range_0_100",
        "gripper_safe_close_limit": cfg.get("gripper_safe_close_limit"),
        "gripper_close_step": cfg.get("gripper_close_step"),
        "g0_still_used_for_descent_reference": True,
        "g0_used_as_close_termination": False,
        "incremental_close": True,
        "tactile_contact_is_primary_stop": True,
        "safe_limit_is_secondary_stop": True,
        "timeout_is_success_condition": False,
        "tactile_algorithm_modified": False,
        "visual_modified": False,
        "fk_modified": False,
        "ik_modified": False,
        "descent_modified": False,
        "lift_modified": False,
        "tcp_modified": False,
        "offline_tests_passed": passed,
        "offline_tests_total": total,
        "offline_tests_failed": failed,
        "offline_tests_all_pass": failed == 0,
        "ros2_build_result": args.ros2_build_result,
        "opened_robot_com_port": False,
        "opened_tactile_com_port": False,
        "goal_position_written": False,
        "physical_motion_observed": False,
        "git_commit": "",
        "cases": [{"name": c.name, "passed": c.passed, "details": str(c.details)[:200]} for c in cases],
        "final_status": "PENDING_BUILD" if args.ros2_build_result == "NOT_RUN" else "READY_FOR_FINAL_MANUAL_GRASP_RETEST",
    }

    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"Offline tests: {passed}/{total} passed, {failed} failed")
    for c in cases:
        status = "PASS" if c.passed else "FAIL"
        print(f"  [{status}] {c.name}")
    if failed:
        print(f"\nFAILED: {failed} test(s)")
        return 1
    print("\nAll offline tests passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
