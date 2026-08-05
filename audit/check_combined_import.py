#!/usr/bin/env python
"""Combined ROS2/LeRobot/OpenCV/pyserial import probe."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path


PROBE_CODE = r"""
import importlib
import json
import sys
import traceback

order = sys.argv[1].split(",")
result = {
    "python_executable": sys.executable,
    "python_version": sys.version,
    "order": order,
    "ok": True,
    "imports": [],
}
for name in order:
    item = {"module": name, "ok": False}
    try:
        module = importlib.import_module(name)
        item.update({
            "ok": True,
            "file": getattr(module, "__file__", None),
            "version": getattr(module, "__version__", None),
        })
    except Exception:
        item["error"] = traceback.format_exc()
        result["ok"] = False
    result["imports"].append(item)
print(json.dumps(result, indent=2, ensure_ascii=False))
sys.exit(0 if result["ok"] else 2)
"""


def run_order(order: list[str], timeout: int) -> dict:
    try:
        completed = subprocess.run(
            [sys.executable, "-c", PROBE_CODE, ",".join(order)],
            text=True,
            capture_output=True,
            timeout=timeout,
        )
        parsed = None
        try:
            parsed = json.loads(completed.stdout)
        except Exception:
            parsed = None
        return {
            "order": order,
            "returncode": completed.returncode,
            "stdout": completed.stdout,
            "stderr": completed.stderr,
            "parsed": parsed,
        }
    except Exception as exc:
        return {
            "order": order,
            "returncode": None,
            "error": repr(exc),
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-dir", default=None)
    parser.add_argument("--timeout", type=int, default=45)
    args = parser.parse_args()

    orders = [
        ["rclpy", "lerobot", "cv2", "serial"],
        ["lerobot", "rclpy", "cv2", "serial"],
    ]
    data = {
        "python_executable": sys.executable,
        "python_version": sys.version,
        "orders": [run_order(order, args.timeout) for order in orders],
        "interpretation": (
            "Failures are reported verbatim. This script only imports packages "
            "and does not create robot or serial objects."
        ),
    }
    data["ok"] = all(item.get("returncode") == 0 for item in data["orders"])

    text = json.dumps(data, indent=2, ensure_ascii=False)
    print(text)
    if args.results_dir:
        out_dir = Path(args.results_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        (out_dir / "combined_import.json").write_text(text, encoding="utf-8")
    return 0 if data["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
