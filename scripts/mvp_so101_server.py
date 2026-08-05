from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def load_config(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as file:
        value = json.load(file)
    if not isinstance(value, dict):
        raise ValueError(f"Expected JSON object in {path}")
    return value


def run_dry_run(config_path: Path) -> int:
    config = load_config(config_path)
    hardware_enabled = bool(config.get("hardware_enabled", False))
    print("MVP SO101 server dry run")
    print(f"config={config_path}")
    print(f"host={config.get('host')}")
    print(f"port={config.get('port')}")
    print(f"calibration_path={config.get('calibration_path')}")
    print(f"hardware_enabled={str(hardware_enabled).lower()}")
    print("no serial port opened")
    print("no motor command sent")
    return 0 if not hardware_enabled else 2


def main() -> int:
    parser = argparse.ArgumentParser(
        description="SO-101 MVP LeRobot-side server placeholder."
    )
    parser.add_argument(
        "--config",
        required=True,
        type=Path,
        help="Path to config/mvp_hardware.json.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Load config and print safety status without opening hardware.",
    )
    args = parser.parse_args()

    if args.dry_run:
        return run_dry_run(args.config)

    parser.error("Stage MVP-0 only supports --dry-run")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())

