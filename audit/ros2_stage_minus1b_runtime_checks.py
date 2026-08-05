import argparse
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path


def run_capture(args, log_path, timeout=30, env=None):
    with open(log_path, "w", encoding="utf-8", errors="replace") as out:
        out.write("+ " + " ".join(args) + "\n")
        out.flush()
        return subprocess.run(
            args,
            stdout=out,
            stderr=subprocess.STDOUT,
            timeout=timeout,
            env=env,
            text=True,
        ).returncode


def terminate(proc):
    if proc.poll() is None:
        if os.name == "nt":
            subprocess.run(
                ["taskkill", "/PID", str(proc.pid), "/T", "/F"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=10,
            )
            return
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)


def count_matches(path, needles):
    if not path.exists():
        return 0
    text = path.read_text(encoding="utf-8", errors="replace")
    return sum(1 for line in text.splitlines() if any(n in line for n in needles))


def official_demo(result_root):
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "88"
    talker_log = result_root / "official_talker.log"
    listener_log = result_root / "official_listener.log"
    with open(talker_log, "w", encoding="utf-8", errors="replace") as t_out, open(
        listener_log, "w", encoding="utf-8", errors="replace"
    ) as l_out:
        talker = subprocess.Popen(
            ["ros2", "run", "demo_nodes_cpp", "talker"],
            stdout=t_out,
            stderr=subprocess.STDOUT,
            env=env,
            text=True,
        )
        time.sleep(2.0)
        listener = subprocess.Popen(
            ["ros2", "run", "demo_nodes_py", "listener"],
            stdout=l_out,
            stderr=subprocess.STDOUT,
            env=env,
            text=True,
        )
        time.sleep(10.0)
        terminate(listener)
        terminate(talker)

    talker_count = count_matches(talker_log, ["Publishing", "Hello World"])
    listener_count = count_matches(listener_log, ["I heard", "Hello World"])
    print(f"official_demo talker_count={talker_count} listener_count={listener_count}")
    return 0 if talker_count >= 3 and listener_count >= 3 else 1


def echo_once(topic, out_path, timeout=30, env=None):
    with open(out_path, "w", encoding="utf-8", errors="replace") as out:
        proc = subprocess.run(
            ["ros2", "topic", "echo", topic, "--once"],
            stdout=out,
            stderr=subprocess.STDOUT,
            timeout=timeout,
            env=env,
            text=True,
        )
    return proc.returncode


def minimal_probes(result_root):
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "89"
    checks = []
    workspace = Path.cwd()

    py_log = result_root / "python_probe_node.log"
    py_exe = workspace / "install" / "lib" / "stage_minus1_python_probe" / "python_probe_node.exe"
    with open(py_log, "w", encoding="utf-8", errors="replace") as out:
        proc = subprocess.Popen(
            [str(py_exe)],
            stdout=out,
            stderr=subprocess.STDOUT,
            env=env,
            text=True,
        )
        time.sleep(8.0)
        with open(py_log, "a", encoding="utf-8", errors="replace") as out:
            out.write(f"\nprobe_poll_after_wait={proc.poll()}\n")
        py_echo = result_root / "python_probe_echo.log"
        rc = echo_once("/stage_minus1_python_probe", py_echo, env=env)
        terminate(proc)
    checks.append(rc == 0 and "stage_minus1_python_probe_ok" in py_echo.read_text(encoding="utf-8", errors="replace"))

    cpp_log = result_root / "cpp_probe_node.log"
    cpp_exe = workspace / "install" / "lib" / "stage_minus1_cpp_probe" / "cpp_probe_node.exe"
    with open(cpp_log, "w", encoding="utf-8", errors="replace") as out:
        proc = subprocess.Popen(
            [str(cpp_exe)],
            stdout=out,
            stderr=subprocess.STDOUT,
            env=env,
            text=True,
        )
        time.sleep(8.0)
        with open(cpp_log, "a", encoding="utf-8", errors="replace") as out:
            out.write(f"\nprobe_poll_after_wait={proc.poll()}\n")
            out.write("\n--- ros2 node list ---\n")
            out.flush()
            subprocess.run(["ros2", "node", "list"], stdout=out, stderr=subprocess.STDOUT, timeout=15, env=env, text=True)
            out.write("\n--- ros2 topic list -t ---\n")
            out.flush()
            subprocess.run(["ros2", "topic", "list", "-t"], stdout=out, stderr=subprocess.STDOUT, timeout=15, env=env, text=True)
        cpp_echo = result_root / "cpp_probe_echo.log"
        rc = echo_once("/stage_minus1_cpp_probe", cpp_echo, env=env)
        terminate(proc)
    checks.append(rc == 0 and "stage_minus1_cpp_probe_ok" in cpp_echo.read_text(encoding="utf-8", errors="replace"))

    print(f"minimal_probes python={checks[0]} cpp={checks[1]}")
    return 0 if all(checks) else 1


def tactile_check(result_root):
    exe_log = result_root / "tactile_pkg_executables.log"
    rc = run_capture(["ros2", "pkg", "executables", "so101_flexitac_bridge"], exe_log, timeout=30)
    if rc != 0:
        print("ros2 pkg executables failed")
        return rc
    text = exe_log.read_text(encoding="utf-8", errors="replace")
    expected = [
        "leflexitac_udp_bridge",
        "tactile_processor",
        "tactile_contact_detector",
        "tactile_visualizer",
    ]
    missing = [name for name in expected if name not in text]
    if missing:
        print("missing executables: " + ", ".join(missing))
        return 2

    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "90"
    node_log = result_root / "tactile_contact_detector_start.log"
    with open(node_log, "w", encoding="utf-8", errors="replace") as out:
        proc = subprocess.Popen(
            ["ros2", "run", "so101_flexitac_bridge", "tactile_contact_detector"],
            stdout=out,
            stderr=subprocess.STDOUT,
            env=env,
            text=True,
        )
        time.sleep(5.0)
        still_running = proc.poll() is None
        terminate(proc)
    print(f"tactile_contact_detector_started={still_running}")
    return 0 if still_running else 3


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--result-root", required=True)
    parser.add_argument("--official-demo", action="store_true")
    parser.add_argument("--minimal-probes", action="store_true")
    parser.add_argument("--tactile-check", action="store_true")
    args = parser.parse_args()
    result_root = Path(args.result_root)
    result_root.mkdir(parents=True, exist_ok=True)

    if args.official_demo:
        return official_demo(result_root)
    if args.minimal_probes:
        return minimal_probes(result_root)
    if args.tactile_check:
        return tactile_check(result_root)
    return 2


if __name__ == "__main__":
    sys.exit(main())
