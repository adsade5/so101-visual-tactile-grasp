from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from lerobot_server.mvp_hardware_executor import MvpSo101HardwareExecutor


DEFAULT_CONFIG = PROJECT_ROOT / "config" / "mvp_hardware.json"
DEFAULT_STATE = PROJECT_ROOT / "data" / "verification" / "mvp3a_current_state.json"
DEFAULT_PLAN = PROJECT_ROOT / "data" / "verification" / "mvp3a_wrist_roll_2deg_plan.json"


def write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def load_state(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="MVP-3A SO-101 hardware readiness helper.")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--config-check", action="store_true")
    parser.add_argument("--read-state", action="store_true")
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--state-file", type=Path, default=DEFAULT_STATE)
    parser.add_argument("--plan-file", type=Path, default=DEFAULT_PLAN)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--enable-hardware-motion", action="store_true")
    parser.add_argument("--confirm", default="")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    executor = MvpSo101HardwareExecutor(args.config)

    modes = [args.config_check, args.read_state, args.plan_only, args.execute]
    if sum(1 for item in modes if item) != 1:
        print("Choose exactly one mode: --config-check, --read-state, --plan-only, or --execute")
        return 2

    if args.config_check:
        payload = {
            "mode": "config_check",
            **executor.check_static_config(),
            "api_audit": executor.audit_lerobot_api(),
            "goal_position_written": False,
            "motion_command_sent": False,
        }
        print(json.dumps(payload, indent=2))
        return 0

    if args.read_state:
        payload = {
            "mode": "read_state",
            "config_path": str(args.config),
            "read_only": True,
            **executor.read_state_read_only(),
        }
        write_json(DEFAULT_STATE, payload)
        print(json.dumps(payload, indent=2))
        return 0 if payload.get("success") else 3

    if args.plan_only:
        if not args.state_file.is_file():
            print(f"State file not found: {args.state_file}")
            return 4
        state = load_state(args.state_file)
        if not state.get("success"):
            print(f"State file is not a successful read-only snapshot: {args.state_file}")
            return 5
        plan = executor.build_wrist_roll_test_plan(
            {
                name: float(value)
                for name, value in state["joint_positions_rad"].items()
            },
            float(state["gripper_value"]),
        )
        payload = {
            "mode": "plan_only",
            "config_path": str(args.config),
            "state_file": str(args.state_file),
            "opens_serial_port": False,
            "goal_position_written": False,
            "motion_command_sent": False,
            **plan,
        }
        write_json(args.plan_file, payload)
        print(json.dumps(payload, indent=2))
        return 0

    if args.execute:
        if not args.enable_hardware_motion or args.confirm != "SMALL_WRIST_ROLL_2DEG":
            print("Refusing execution: require --enable-hardware-motion and --confirm SMALL_WRIST_ROLL_2DEG")
            return 10
        if not args.plan_file.is_file():
            print(f"Plan file not found: {args.plan_file}")
            return 11
        plan = json.loads(args.plan_file.read_text(encoding="utf-8"))
        result = executor.execute_plan(
            plan,
            enable_hardware_motion=args.enable_hardware_motion,
            confirm=args.confirm,
        )
        print(json.dumps(result, indent=2))
        return 0 if result.get("success") else 12

    return 2


if __name__ == "__main__":
    raise SystemExit(main())
