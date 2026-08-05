#!/usr/bin/env python
"""Static compatibility scan for the legacy tactile project."""

from __future__ import annotations

import argparse
import ast
import json
import re
import xml.etree.ElementTree as ET
from pathlib import Path


PATTERNS = {
    "linux_api": [
        "termios",
        "fcntl",
        "pty",
        "tty",
        "os.setsid",
        "os.killpg",
        "signal.SIGCHLD",
        "select.epoll",
        "inotify",
        "udev",
        "pyudev",
    ],
    "linux_paths_commands": [
        "/dev/tty",
        "/dev/video",
        "chmod",
        "chown",
        "sudo",
        "apt-get",
        "apt",
        "#!/bin/bash",
        "#!/usr/bin/env bash",
        "source install/setup.bash",
        "setup.bash",
        "LD_LIBRARY_PATH",
        "DISPLAY",
        "X11",
    ],
    "serial": [
        "serial",
        "TactileSensor",
        "baud",
        "baud_rate",
        "timeout",
        "readline",
        "start_continuous_read",
        "disconnect",
        "COM",
        "/dev/tty",
    ],
    "socket": [
        "socket",
        "SOCK_DGRAM",
        "UDP",
        "udp",
        "bind",
        "sendto",
        "recvfrom",
        "setblocking",
        "settimeout",
        "0.0.0.0",
        "127.0.0.1",
        "localhost",
        "5005",
        "5006",
    ],
    "ros2": [
        "rclpy",
        "rclcpp",
        "create_publisher",
        "create_subscription",
        "declare_parameter",
        "launch_ros",
        "Float32MultiArray",
        "Bool",
        "Float32",
    ],
    "path_access": [
        "os.path",
        "Path(",
        "glob(",
        "/opt/ros",
        "${HOME}",
        "~",
        "\\",
    ],
}


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


def scan_patterns(root: Path) -> list[dict]:
    hits: list[dict] = []
    for path in root.rglob("*"):
        if not path.is_file():
            continue
        if path.suffix.lower() not in {".py", ".xml", ".txt", ".md", ".sh", ".cfg", ".toml", ".yml", ".yaml"} and path.name not in {"CMakeLists.txt"}:
            continue
        rel = str(path.relative_to(root))
        text = read_text(path)
        lines = text.splitlines()
        for category, patterns in PATTERNS.items():
            for line_no, line in enumerate(lines, start=1):
                lower_line = line.lower()
                for pattern in patterns:
                    if pattern.lower() in lower_line:
                        hits.append(
                            {
                                "category": category,
                                "pattern": pattern,
                                "file": rel,
                                "line": line_no,
                                "text": line.strip(),
                            }
                        )
    return hits


def parse_python_symbols(root: Path) -> dict[str, dict]:
    symbols: dict[str, dict] = {}
    for path in root.rglob("*.py"):
        rel = str(path.relative_to(root))
        item = {"classes": [], "functions": [], "imports": []}
        try:
            tree = ast.parse(read_text(path))
        except SyntaxError as exc:
            item["syntax_error"] = str(exc)
            symbols[rel] = item
            continue
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef):
                item["classes"].append({"name": node.name, "line": node.lineno})
            elif isinstance(node, ast.FunctionDef):
                item["functions"].append({"name": node.name, "line": node.lineno})
            elif isinstance(node, ast.Import):
                item["imports"].extend(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom):
                module = node.module or ""
                item["imports"].append(module)
        symbols[rel] = item
    return symbols


def parse_package_xml(path: Path) -> dict:
    data = {"path": str(path), "ok": False}
    try:
        root = ET.fromstring(read_text(path))
        data["ok"] = True
        data["name"] = root.findtext("name")
        data["build_type"] = None
        export = root.find("export")
        if export is not None:
            data["build_type"] = export.findtext("build_type")
        deps = []
        for tag in ("depend", "build_depend", "exec_depend", "test_depend"):
            deps.extend({"type": tag, "name": elem.text} for elem in root.findall(tag))
        data["dependencies"] = deps
    except Exception as exc:
        data["error"] = repr(exc)
    return data


def parse_setup_py(path: Path) -> dict:
    text = read_text(path)
    entries = re.findall(r"['\"]([^'\"]+)\s*=\s*([^:'\"]+):([^'\"]+)['\"]", text)
    return {
        "path": str(path),
        "console_scripts": [
            {"name": name.strip(), "module": module.strip(), "function": func.strip()}
            for name, module, func in entries
        ],
    }


def parse_launch(path: Path) -> dict:
    text = read_text(path)
    return {
        "path": str(path),
        "nodes": [
            {
                "package": match.group("package"),
                "executable": match.group("executable"),
                "name": match.group("name"),
            }
            for match in re.finditer(
                r"package=['\"](?P<package>[^'\"]+)['\"].*?executable=['\"](?P<executable>[^'\"]+)['\"].*?name=['\"](?P<name>[^'\"]+)['\"]",
                text,
                flags=re.S,
            )
        ],
        "parameters": re.findall(r"['\"]([^'\"]+)['\"]\s*:\s*([^,\n}]+)", text),
    }


def classify_modules() -> list[dict]:
    return [
        {
            "file": "ros2/so101_flexitac_bridge/so101_flexitac_bridge/leflexitac_udp_bridge.py",
            "symbols": "LeFlexiTacUDPBridge, process_packet, poll_socket, guard_callback",
            "responsibility": "Receives LeFlexiTac UDP frames, validates packet header/payload, publishes /tactile/raw, forwards /tactile/contact_detected as UDP guard packets.",
            "classification": "B",
            "reason": "Core protocol is cross-platform Python sockets and NumPy. It binds UDP sockets in __init__, so import is safe but node construction opens localhost ports; port collision handling should be hardened for Windows.",
            "copy_to_new_project": "Yes, after parameter and lifecycle cleanup.",
            "keep_independent": "Yes, useful as a ROS2 bridge boundary.",
        },
        {
            "file": "ros2/so101_flexitac_bridge/so101_flexitac_bridge/tactile_contact_detector.py",
            "symbols": "TactileContactDetector, _calculate_contact_score, _update_no_contact_state, _update_contact_state",
            "responsibility": "Top-k contact score, taxel floor filtering, hysteresis thresholds, consecutive-frame contact/release state machine, /tactile/contact_score and /tactile/contact_detected publishing.",
            "classification": "A",
            "reason": "Pure Python/NumPy/rclpy logic with no hardware, serial, filesystem, or OS-specific calls.",
            "copy_to_new_project": "Yes.",
            "keep_independent": "Can be copied or kept as a ROS2 package dependency.",
        },
        {
            "file": "ros2/so101_flexitac_bridge/so101_flexitac_bridge/tactile_processor.py",
            "symbols": "TactileProcessor, tactile_callback",
            "responsibility": "Legacy threshold contact area features and /tactile/contact_state publishing.",
            "classification": "A",
            "reason": "Pure rclpy message processing; no platform-specific APIs.",
            "copy_to_new_project": "Optional, contact_detector is stronger for closed-loop guard use.",
            "keep_independent": "Optional.",
        },
        {
            "file": "ros2/so101_flexitac_bridge/so101_flexitac_bridge/tactile_visualizer.py",
            "symbols": "TactileVisualizer, render_latest_frame",
            "responsibility": "OpenCV heatmap GUI for tactile frames.",
            "classification": "B",
            "reason": "Computation is portable, but cv2.namedWindow/imshow depend on Windows GUI/OpenCV build and should stay optional/headless-safe.",
            "copy_to_new_project": "Yes, only as optional visualization tooling.",
            "keep_independent": "Optional.",
        },
        {
            "file": "lerobot_extension/src/lerobot/robots/so_tactile_follower/tactile_udp_sender.py",
            "symbols": "TactileUDPSender, from_environment, send, close",
            "responsibility": "Serializes tactile matrices into FTAC UDP packets from the LeRobot observation loop.",
            "classification": "A",
            "reason": "Uses localhost UDP and NumPy only; disabled by default through environment flag.",
            "copy_to_new_project": "Yes if using the same UDP protocol.",
            "keep_independent": "Can remain in LeRobot-side environment.",
        },
        {
            "file": "lerobot_extension/src/lerobot/robots/so_tactile_follower/tactile_guard.py",
            "symbols": "TactileGuardReceiver, TactileGuardStatus, poll, _parse_packet",
            "responsibility": "Receives GRIP UDP guard packets with non-blocking latest-state semantics.",
            "classification": "B",
            "reason": "Protocol is portable; construction binds a UDP socket, so integration should isolate construction from import and handle port conflicts/timeouts explicitly.",
            "copy_to_new_project": "Yes, but likely LeRobot-side only.",
            "keep_independent": "Yes.",
        },
        {
            "file": "lerobot_extension/src/lerobot/robots/so_tactile_follower/so_tactile_follower.py",
            "symbols": "SOTactileFollower, get_observation, _apply_tactile_guard, send_action, disconnect",
            "responsibility": "Extends SOFollower, initializes tactile sensors, sends observations over UDP, clips gripper actions after tactile contact.",
            "classification": "C",
            "reason": "The guard state machine is reusable, but class construction calls SOFollower.__init__ and may instantiate TactileSensor and start_continuous_read for configured sensors. This must not be imported/constructed blindly in Windows audits.",
            "copy_to_new_project": "No direct copy until hardware boundary is redesigned.",
            "keep_independent": "Prefer keeping in LeRobot-side hardware process.",
        },
        {
            "file": "lerobot_extension/src/lerobot/robots/so_tactile_follower/config_so_tactile_follower.py",
            "symbols": "TactileGuardConfig, SOTactileFollowerConfig",
            "responsibility": "LeRobot config dataclasses and default guard parameters.",
            "classification": "B",
            "reason": "Mostly portable config, but sample /dev/ttyUSB comments and imports depend on the installed LeRobot package layout.",
            "copy_to_new_project": "Use as reference, adapt COM port examples.",
            "keep_independent": "Yes, if LeRobot extension remains separate.",
        },
        {
            "file": "scripts/run_ros2.sh",
            "symbols": "shell script",
            "responsibility": "Sources Linux ROS2 setup and launches tactile system.",
            "classification": "D",
            "reason": "Bash and /opt/ros/humble paths are Linux-specific. Replace with a Windows PowerShell launch note if needed.",
            "copy_to_new_project": "No.",
            "keep_independent": "No.",
        },
    ]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("legacy_root")
    parser.add_argument("--results-dir", default=None)
    args = parser.parse_args()

    legacy_root = Path(args.legacy_root).resolve()
    data = {
        "legacy_root": str(legacy_root),
        "exists": legacy_root.exists(),
        "pattern_hits": [],
        "python_symbols": {},
        "ros2_packages": [],
        "setup_py": [],
        "launch_files": [],
        "module_classification": classify_modules(),
    }

    if not legacy_root.exists():
        data["error"] = "Legacy root does not exist."
    else:
        data["pattern_hits"] = scan_patterns(legacy_root)
        data["python_symbols"] = parse_python_symbols(legacy_root)
        for package_xml in legacy_root.rglob("package.xml"):
            data["ros2_packages"].append(parse_package_xml(package_xml))
        for setup_py in legacy_root.rglob("setup.py"):
            data["setup_py"].append(parse_setup_py(setup_py))
        for launch_py in legacy_root.rglob("*.launch.py"):
            data["launch_files"].append(parse_launch(launch_py))

    text = json.dumps(data, indent=2, ensure_ascii=False)
    print(text)
    if args.results_dir:
        out_dir = Path(args.results_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        (out_dir / "legacy_static_scan.json").write_text(text, encoding="utf-8")

        summary = [
            "# Legacy Static Scan Summary",
            "",
            f"Legacy root: `{legacy_root}`",
            "",
            "## ROS2 Packages",
        ]
        for package in data["ros2_packages"]:
            summary.append(f"- {package.get('name')}: build_type={package.get('build_type')}, path={package.get('path')}")
        summary.extend(["", "## Module Classifications"])
        for module in data["module_classification"]:
            summary.append(f"- {module['classification']} `{module['file']}`: {module['reason']}")
        summary.extend(["", "## Pattern Hits"])
        for hit in data["pattern_hits"]:
            summary.append(f"- {hit['category']} `{hit['pattern']}` at `{hit['file']}:{hit['line']}`: {hit['text']}")
        (out_dir / "legacy_static_scan.md").write_text("\n".join(summary) + "\n", encoding="utf-8")

    return 0 if data["exists"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
