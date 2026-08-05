from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import threading
import time
from pathlib import Path
from typing import Any, Protocol

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from lerobot_server.mvp_hardware_executor import (
    ALL_MOTOR_NAMES,
    ARM_JOINT_NAMES,
    MvpSo101HardwareExecutor,
    radians_to_degrees,
)


MAX_SPEED_RAD_S = 0.08
MOVE_CONFIRMATION = "MVP_MOVE"
STOP_BANNER = (
    "SERVER STOPPING\n"
    "NO FURTHER MOTION COMMANDS\n"
    "SERVO TORQUE NOT AUTOMATICALLY DISABLED\n"
    "POWER OFF SERVO SUPPLY MANUALLY WHEN FINISHED"
)


class HardwareBackend(Protocol):
    def connect(self) -> None: ...

    def close(self) -> None: ...

    def get_state(self) -> dict[str, Any]: ...

    def move_joints_sequential(
        self,
        target_rad: list[float],
        speed_rad_s: float,
        joint_order: list[int],
    ) -> dict[str, Any]: ...

    def stop(self) -> dict[str, Any]: ...


def load_config(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as file:
        value = json.load(file)
    if not isinstance(value, dict):
        raise ValueError(f"Expected JSON object in {path}")
    return value


def response(success: bool, reason: str, **extra: Any) -> dict[str, Any]:
    value = {"success": bool(success), "reason": reason}
    value.update(extra)
    return value


class ReadOnlyFeetechBackend:
    def __init__(self, config_path: Path) -> None:
        self.executor = MvpSo101HardwareExecutor(config_path)
        self.bus: Any | None = None
        self.connected = False
        self.stop_requested = False
        self.detected_usb_serial = ""
        self.opened_com_ports = False
        self.goal_position_write_count = 0
        self.torque_enable_write_count = 0
        self.torque_disable_write_count = 0

    def connect(self) -> None:
        port_check = self.executor.check_port_and_usb()
        if not port_check["success"]:
            raise RuntimeError(port_check["reason"])
        port_info = port_check.get("port_info") or {}
        self.detected_usb_serial = str(port_info.get("serial_number") or "")
        self.bus = self.executor.make_read_only_bus()
        self.bus.connect(handshake=False)
        self.connected = True
        self.opened_com_ports = True
        self._read_bus_state()

    def close(self) -> None:
        if self.bus is not None and self.connected:
            self.bus.disconnect(disable_torque=False)
        self.connected = False
        self.bus = None

    def _read_bus_state(self) -> tuple[dict[str, float], dict[str, float]]:
        if self.bus is None:
            raise RuntimeError("backend_not_connected")
        raw = self.bus.sync_read("Present_Position", ALL_MOTOR_NAMES, normalize=False, num_retry=3)
        calibrated = self.bus.sync_read("Present_Position", ALL_MOTOR_NAMES, normalize=True, num_retry=3)
        return (
            {name: float(raw[name]) for name in ALL_MOTOR_NAMES},
            {name: float(calibrated[name]) for name in ALL_MOTOR_NAMES},
        )

    def get_state(self) -> dict[str, Any]:
        raw, calibrated = self._read_bus_state()
        observation = {f"{name}.pos": float(calibrated[name]) for name in ALL_MOTOR_NAMES}
        joints_rad, gripper = self.executor.observation_to_internal_rad(observation)
        within = self.executor.all_targets_within_calibration(joints_rad, gripper)
        return response(
            True,
            "state_ok",
            joint_names=list(ARM_JOINT_NAMES),
            positions_rad=[float(joints_rad[name]) for name in ARM_JOINT_NAMES],
            gripper=float(gripper),
            within_calibration=bool(within),
            raw_lerobot_positions=raw,
            calibrated_lerobot_positions=calibrated,
        )

    def move_joints_sequential(
        self,
        target_rad: list[float],
        speed_rad_s: float,
        joint_order: list[int],
    ) -> dict[str, Any]:
        del target_rad, speed_rad_s, joint_order
        return response(False, "hardware_motion_disabled")

    def stop(self) -> dict[str, Any]:
        self.stop_requested = True
        return response(True, "stop_requested")

    def counters(self) -> dict[str, Any]:
        return {
            "opened_com_ports": bool(self.opened_com_ports),
            "detected_usb_serial": self.detected_usb_serial,
            "goal_position_write_count": self.goal_position_write_count,
            "torque_enable_write_count": self.torque_enable_write_count,
            "torque_disable_write_count": self.torque_disable_write_count,
        }


class MotionFeetechBackend(ReadOnlyFeetechBackend):
    def move_joints_sequential(
        self,
        target_rad: list[float],
        speed_rad_s: float,
        joint_order: list[int],
    ) -> dict[str, Any]:
        current_state = self.get_state()
        if not current_state["success"]:
            return current_state

        positions = [float(v) for v in current_state["positions_rad"]]
        self.stop_requested = False
        for index in joint_order:
            if self.stop_requested:
                return response(False, "stopped")
            target = float(target_rad[index])
            while abs(positions[index] - target) > 1.0e-6:
                if self.stop_requested:
                    return response(False, "stopped")
                delta = target - positions[index]
                step = math.copysign(min(abs(delta), speed_rad_s / 20.0), delta)
                positions[index] += step
                action = {f"{ARM_JOINT_NAMES[index]}.pos": radians_to_degrees(positions[index])}
                if self.bus is None:
                    return response(False, "backend_not_connected")
                self.bus.sync_write("Goal_Position", action, normalize=True)
                self.goal_position_write_count += 1
                time.sleep(1.0 / 20.0)
        return response(True, "motion_completed")


class MvpTcpServer:
    def __init__(
        self,
        *,
        host: str,
        port: int,
        backend: HardwareBackend,
        hardware_motion_enabled: bool = False,
    ) -> None:
        self.host = host
        self.port = int(port)
        self.backend = backend
        self.hardware_motion_enabled = bool(hardware_motion_enabled)
        self.stop_event = threading.Event()
        self.server_socket: socket.socket | None = None
        self.get_state_request_count = 0
        self.move_request_received_count = 0
        self.move_request_executed_count = 0

    def serve_forever(self) -> None:
        self.backend.connect()
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((self.host, self.port))
            sock.listen(1)
            sock.settimeout(0.5)
            self.server_socket = sock
            print("MVP SO101 TCP server starting", flush=True)
            print(f"server_mode={'motion_enabled' if self.hardware_motion_enabled else 'read_only'}", flush=True)
            print(f"hardware_motion_enabled={str(self.hardware_motion_enabled).lower()}", flush=True)
            print(f"host={self.host}", flush=True)
            print(f"port={self.port}", flush=True)
            executor = getattr(self.backend, "executor", None)
            if executor is not None:
                print(f"follower_port={executor.config.get('follower_port')}", flush=True)
                print(
                    f"expected_usb_serial_number={executor.config.get('expected_usb_serial_number')}",
                    flush=True,
                )
                print(f"calibration_path={executor.calibration_path}", flush=True)
            detected_usb_serial = getattr(self.backend, "detected_usb_serial", "")
            if detected_usb_serial:
                print(f"detected_usb_serial={detected_usb_serial}", flush=True)
            print(f"MVP SO101 TCP server listening on {self.host}:{self.port}", flush=True)
            try:
                while not self.stop_event.is_set():
                    try:
                        conn, addr = sock.accept()
                    except socket.timeout:
                        continue
                    with conn:
                        conn.settimeout(5.0)
                        self._serve_client(conn, addr)
            finally:
                self.backend.close()
                print("SERVER COUNTERS", flush=True)
                print(json.dumps(self.counters(), sort_keys=True), flush=True)
                print(STOP_BANNER, flush=True)

    def shutdown(self) -> None:
        self.stop_event.set()
        try:
            if self.server_socket is not None:
                self.server_socket.close()
        except OSError:
            pass

    def _serve_client(self, conn: socket.socket, addr: tuple[str, int]) -> None:
        del addr
        with conn.makefile("rwb") as stream:
            while not self.stop_event.is_set():
                line = stream.readline()
                if not line:
                    return
                result = self.handle_line(line.decode("utf-8", errors="replace"))
                stream.write((json.dumps(result, separators=(",", ":")) + "\n").encode("utf-8"))
                stream.flush()

    def handle_line(self, line: str) -> dict[str, Any]:
        try:
            request = json.loads(line)
        except json.JSONDecodeError:
            return response(False, "malformed_json")
        if not isinstance(request, dict):
            return response(False, "invalid_request")
        command = request.get("command")
        if command == "get_state":
            self.get_state_request_count += 1
            return self.backend.get_state()
        if command == "move_joints_sequential":
            return self._handle_move(request)
        if command == "stop":
            result = self.backend.stop()
            self.stop_event.set()
            return result
        return response(False, "unknown_command")

    def _handle_move(self, request: dict[str, Any]) -> dict[str, Any]:
        self.move_request_received_count += 1
        if not self.hardware_motion_enabled:
            return response(False, "hardware_motion_disabled")
        if request.get("confirm") != MOVE_CONFIRMATION:
            return response(False, "confirmation_missing")

        target = request.get("target_rad")
        if not isinstance(target, list) or len(target) != len(ARM_JOINT_NAMES):
            return response(False, "invalid_target_length")
        try:
            target_rad = [float(value) for value in target]
        except (TypeError, ValueError):
            return response(False, "non_finite_target")
        if not all(math.isfinite(value) for value in target_rad):
            return response(False, "non_finite_target")

        try:
            speed = float(request.get("speed_rad_s"))
        except (TypeError, ValueError):
            return response(False, "invalid_speed")
        if not math.isfinite(speed) or speed <= 0.0:
            return response(False, "invalid_speed")
        if speed > MAX_SPEED_RAD_S:
            return response(False, "speed_above_limit")

        order = request.get("joint_order")
        if not isinstance(order, list):
            return response(False, "invalid_joint_order")
        try:
            joint_order = [int(value) for value in order]
        except (TypeError, ValueError):
            return response(False, "invalid_joint_order")
        if sorted(joint_order) != list(range(len(ARM_JOINT_NAMES))):
            return response(False, "invalid_joint_order")

        targets = dict(zip(ARM_JOINT_NAMES, target_rad, strict=True))
        executor = getattr(self.backend, "executor", None)
        if executor is not None:
            for name, value in targets.items():
                if not executor.target_within_calibration(name, value):
                    return response(False, "calibration_out_of_range")

        result = self.backend.move_joints_sequential(target_rad, speed, joint_order)
        if result.get("success"):
            self.move_request_executed_count += 1
        return result

    def counters(self) -> dict[str, Any]:
        backend_counters = {}
        if hasattr(self.backend, "counters"):
            backend_counters = self.backend.counters()
        return {
            "get_state_request_count": self.get_state_request_count,
            "move_request_received_count": self.move_request_received_count,
            "move_request_executed_count": self.move_request_executed_count,
            **backend_counters,
        }


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
    parser = argparse.ArgumentParser(description="SO-101 MVP JSON Lines TCP server.")
    parser.add_argument("--config", required=True, type=Path, help="Path to config/mvp_hardware.json.")
    parser.add_argument("--dry-run", action="store_true", help="Load config without opening hardware.")
    parser.add_argument("--read-only", action="store_true", help="Force read-only mode.")
    parser.add_argument(
        "--enable-hardware-motion",
        action="store_true",
        help="Allow validated move_joints_sequential requests.",
    )
    args = parser.parse_args()

    if args.dry_run:
        return run_dry_run(args.config)

    config = load_config(args.config)
    host = str(config.get("host", "127.0.0.1"))
    port = int(config.get("port", 8770))
    motion_enabled = bool(args.enable_hardware_motion and not args.read_only)
    backend: HardwareBackend
    if motion_enabled:
        backend = MotionFeetechBackend(args.config)
    else:
        backend = ReadOnlyFeetechBackend(args.config)
    server = MvpTcpServer(host=host, port=port, backend=backend, hardware_motion_enabled=motion_enabled)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        server.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
