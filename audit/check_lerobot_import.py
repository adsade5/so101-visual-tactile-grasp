#!/usr/bin/env python
"""Read-only LeRobot import and module availability probe."""

from __future__ import annotations

import argparse
import importlib
import importlib.metadata
import json
import pkgutil
import sys
import traceback
from pathlib import Path


KEYWORDS = ("so101", "so_101", "so100", "so_follower", "feetech", "tactile", "servo")


def try_import(module_name: str) -> dict:
    result = {"module": module_name, "ok": False}
    try:
        module = importlib.import_module(module_name)
        result.update({"ok": True, "file": getattr(module, "__file__", None)})
    except Exception:
        result["error"] = traceback.format_exc()
    return result


def distribution_version(name: str) -> str | None:
    try:
        return importlib.metadata.version(name)
    except Exception:
        return None


def walk_lerobot_modules(limit: int = 800) -> tuple[list[str], list[str]]:
    modules: list[str] = []
    errors: list[str] = []
    try:
        root = importlib.import_module("lerobot")
    except Exception:
        return modules, errors

    paths = getattr(root, "__path__", None)
    if not paths:
        return modules, errors

    def onerror(name: str) -> None:
        errors.append(name)

    try:
        for index, module_info in enumerate(pkgutil.walk_packages(paths, prefix="lerobot.", onerror=onerror)):
            if index >= limit:
                break
            name = module_info.name
            if any(keyword in name.lower() for keyword in KEYWORDS):
                modules.append(name)
    except Exception:
        errors.append(traceback.format_exc())
    return modules, errors


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-dir", default=None)
    parser.add_argument("--old-extension-src", default=None)
    args = parser.parse_args()

    if args.old_extension_src:
        sys.path.insert(0, str(Path(args.old_extension_src).resolve()))

    data = {
        "python_executable": sys.executable,
        "python_version": sys.version,
        "ok": False,
        "lerobot_version": distribution_version("lerobot"),
        "feetech_versions": {
            "feetech-servo-sdk": distribution_version("feetech-servo-sdk"),
            "feetech-servo-sdk-python": distribution_version("feetech-servo-sdk-python"),
        },
        "pyserial_version": distribution_version("pyserial"),
    }

    lerobot_probe = try_import("lerobot")
    data["lerobot_import"] = lerobot_probe
    data["ok"] = lerobot_probe["ok"]
    matching_modules, walk_errors = walk_lerobot_modules() if data["ok"] else ([], [])
    data["matching_modules"] = matching_modules
    data["module_walk_errors"] = walk_errors

    candidate_modules = [
        "lerobot.robots.so101_follower",
        "lerobot.robots.so101_follower.config_so101_follower",
        "lerobot.robots.so100_follower",
        "lerobot.robots.so_follower",
        "lerobot.robots.so_follower.so_follower",
        "lerobot.motors.feetech",
        "lerobot.motors.feetech.feetech",
        "lerobot.sensors",
        "lerobot.sensors.tactile_sensor",
        "lerobot.robots.so_tactile_follower.tactile_udp_sender",
        "lerobot.robots.so_tactile_follower.tactile_guard",
        "lerobot.robots.so_tactile_follower.config_so_tactile_follower",
        "lerobot.robots.so_tactile_follower.so_tactile_follower",
    ]
    data["candidate_imports"] = {name: try_import(name) for name in candidate_modules}
    data["note"] = (
        "This probe imports modules only. It does not instantiate robot objects, "
        "does not call connect/calibrate/teleop/send_action, and does not open serial ports."
    )

    text = json.dumps(data, indent=2, ensure_ascii=False)
    print(text)
    if args.results_dir:
        out_dir = Path(args.results_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        (out_dir / "lerobot_import.json").write_text(text, encoding="utf-8")
    return 0 if data["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
