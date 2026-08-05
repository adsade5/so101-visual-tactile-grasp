#!/usr/bin/env python
"""Stage -1 Python runtime probe.

This script is intentionally read-only. It does not open serial ports,
does not connect to robot hardware, and does not send network control
messages.
"""

from __future__ import annotations

import argparse
import importlib
import importlib.metadata
import json
import os
import platform
import subprocess
import sys
import traceback
from pathlib import Path


def import_probe(module_name: str) -> dict:
    result = {"module": module_name, "ok": False}
    try:
        module = importlib.import_module(module_name)
        result.update(
            {
                "ok": True,
                "file": getattr(module, "__file__", None),
                "version": getattr(module, "__version__", None),
            }
        )
    except Exception:
        result["error"] = traceback.format_exc()
    return result


def package_version(name: str) -> str | None:
    try:
        return importlib.metadata.version(name)
    except importlib.metadata.PackageNotFoundError:
        return None
    except Exception:
        return None


def pip_command(args: list[str]) -> dict:
    try:
        completed = subprocess.run(
            [sys.executable, "-m", "pip", *args],
            text=True,
            capture_output=True,
            timeout=30,
        )
        return {
            "returncode": completed.returncode,
            "stdout": completed.stdout,
            "stderr": completed.stderr,
        }
    except Exception:
        return {"returncode": None, "error": traceback.format_exc()}


def list_serial_ports() -> dict:
    result = {"ok": False, "ports": []}
    try:
        from serial.tools import list_ports

        result["ok"] = True
        for port in list_ports.comports():
            result["ports"].append(
                {
                    "device": port.device,
                    "description": port.description,
                    "hwid": port.hwid,
                }
            )
    except Exception:
        result["error"] = traceback.format_exc()
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-dir", default=None)
    args = parser.parse_args()

    safe_env = {
        key: value
        for key, value in os.environ.items()
        if any(token in key.upper() for token in ("ROS", "AMENT", "COLCON", "RMW", "CONDA", "PYTHON"))
        and not any(secret in key.upper() for secret in ("TOKEN", "SECRET", "PASSWORD", "KEY"))
    }

    data = {
        "script": str(Path(__file__).resolve()),
        "python_executable": sys.executable,
        "python_version": sys.version,
        "platform": platform.platform(),
        "machine": platform.machine(),
        "cwd": str(Path.cwd()),
        "sys_path": sys.path,
        "environment": safe_env,
        "imports": {
            name: import_probe(name)
            for name in ("numpy", "cv2", "serial", "rclpy", "std_msgs", "lerobot")
        },
        "package_versions": {
            name: package_version(name)
            for name in ("numpy", "opencv-python", "opencv-contrib-python", "pyserial", "rclpy", "lerobot", "torch")
        },
        "serial_ports_read_only": list_serial_ports(),
        "pip_show_lerobot": pip_command(["show", "lerobot"]),
        "pip_list_selected": pip_command(["list"]),
    }

    text = json.dumps(data, indent=2, ensure_ascii=False)
    print(text)

    if args.results_dir:
        out_dir = Path(args.results_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        (out_dir / "python_runtime.json").write_text(text, encoding="utf-8")

    blocking_imports = ["cv2", "serial"]
    return 0 if all(data["imports"][name]["ok"] for name in blocking_imports) else 2


if __name__ == "__main__":
    raise SystemExit(main())
