#!/usr/bin/env python
"""Read-only ROS2 Python import probe."""

from __future__ import annotations

import argparse
import importlib
import json
import sys
import traceback
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-dir", default=None)
    args = parser.parse_args()

    data = {
        "python_executable": sys.executable,
        "python_version": sys.version,
        "ok": False,
    }
    try:
        import rclpy
        from std_msgs.msg import String

        data.update(
            {
                "ok": True,
                "rclpy_path": getattr(rclpy, "__file__", None),
                "std_msgs_path": getattr(importlib.import_module("std_msgs"), "__file__", None),
                "String_type": str(String),
            }
        )
    except Exception:
        data["error"] = traceback.format_exc()

    text = json.dumps(data, indent=2, ensure_ascii=False)
    print(text)
    if args.results_dir:
        out_dir = Path(args.results_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        (out_dir / "ros2_import.json").write_text(text, encoding="utf-8")
    return 0 if data["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
