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
    LEROBOT_ACTION_KEYS,
    LEROBOT_POSITION_KEYS,
    MvpSo101HardwareExecutor,
    build_lerobot_action,
    extract_arm_state_from_lerobot_observation,
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
        gripper_target_pos: float | None = None,
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


def bool_text(value: bool) -> str:
    return str(bool(value)).lower()


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
        gripper_target_pos: float | None = None,
    ) -> dict[str, Any]:
        del target_rad, speed_rad_s, joint_order, gripper_target_pos
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
    def __init__(self, config_path: Path) -> None:
        super().__init__(config_path)
        self.robot: Any | None = None
        self.logged_observation_keys = False
        self.logged_action_keys = False

    def connect(self) -> None:
        port_check = self.executor.check_port_and_usb()
        if not port_check["success"]:
            raise RuntimeError(port_check["reason"])
        port_info = port_check.get("port_info") or {}
        self.detected_usb_serial = str(port_info.get("serial_number") or "")
        self.robot = self.executor.make_official_robot()
        try:
            self.robot.connect(calibrate=False)
        except TypeError:
            self.robot.connect()
        self.connected = True
        self.opened_com_ports = True

    def close(self) -> None:
        if self.robot is not None and self.connected:
            bus = getattr(self.robot, "bus", None)
            if bus is not None and hasattr(bus, "disconnect"):
                bus.disconnect(disable_torque=False)
            else:
                self.robot.disconnect()
        self.connected = False
        self.robot = None

    def get_state(self) -> dict[str, Any]:
        if self.robot is None:
            return response(False, "backend_not_connected")
        observation = self.robot.get_observation()
        self._log_observation_keys_once(observation)
        state = extract_arm_state_from_lerobot_observation(observation)
        joints_rad = state["joint_positions_rad"]
        gripper = state["gripper_value"]
        return response(
            True,
            "state_ok",
            joint_names=list(ARM_JOINT_NAMES),
            positions_rad=[float(joints_rad[name]) for name in ARM_JOINT_NAMES],
            gripper=float(gripper),
            within_calibration=bool(self.executor.all_targets_within_calibration(joints_rad, gripper)),
        )

    def move_joints_sequential(
        self,
        target_rad: list[float],
        speed_rad_s: float,
        joint_order: list[int],
        gripper_target_pos: float | None = None,
    ) -> dict[str, Any]:
        if self.robot is None:
            return response(False, "backend_not_connected")
        current_state = self.get_state()
        if not current_state["success"]:
            return current_state

        positions = {
            name: float(value)
            for name, value in zip(ARM_JOINT_NAMES, current_state["positions_rad"], strict=True)
        }
        target_positions = {
            name: float(value)
            for name, value in zip(ARM_JOINT_NAMES, target_rad, strict=True)
        }
        gripper = float(current_state["gripper"])
        target_gripper = gripper if gripper_target_pos is None else float(gripper_target_pos)
        if not math.isfinite(target_gripper):
            return response(False, "invalid_gripper_target_pos")
        if not self.executor.all_targets_within_calibration(target_positions, target_gripper):
            return response(False, "gripper_target_out_of_calibration_range")
        self.stop_requested = False
        threshold_rad = math.radians(float(self.executor.config["maximum_tracking_error_deg"]))
        bad_tracking_count = 0
        gripper_only_duration_s = float(self.executor.config.get("gripper_only_motion_duration_s", 2.0))
        total_arm_delta = sum(abs(target_positions[name] - positions[name]) for name in ARM_JOINT_NAMES)
        total_steps = 0
        for index in joint_order:
            joint_name = ARM_JOINT_NAMES[index]
            distance = abs(target_positions[joint_name] - positions[joint_name])
            if distance > 1.0e-6:
                total_steps += max(1, int(math.ceil(distance / (speed_rad_s / 20.0))))
        if total_arm_delta <= 1.0e-9 and abs(target_gripper - gripper) > 1.0e-9:
            total_steps = max(1, int(math.ceil(gripper_only_duration_s * 20.0)))
            joint_order = [0]
        gripper_step_index = 0
        for index in joint_order:
            joint_name = ARM_JOINT_NAMES[index]
            target = target_positions[joint_name]
            while abs(positions[joint_name] - target) > 1.0e-6 or (
                total_arm_delta <= 1.0e-9
                and abs(target_gripper - gripper) > 1.0e-9
                and gripper_step_index < total_steps
            ):
                delta = target - positions[joint_name]
                step = math.copysign(min(abs(delta), speed_rad_s / 20.0), delta)
                positions[joint_name] += step
                gripper_step_index += 1
                gripper_fraction = min(1.0, gripper_step_index / max(total_steps, 1))
                gripper_command = gripper + gripper_fraction * (target_gripper - gripper)
                if abs(positions[joint_name] - target) <= 1.0e-6:
                    positions[joint_name] = target
                action = build_lerobot_action(positions, gripper)
                if gripper_target_pos is not None:
                    action = build_lerobot_action(positions, gripper_command)
                self._log_action_keys_once(action)
                self.robot.send_action(action)
                self.goal_position_write_count += 1
                measured_observation = self.robot.get_observation()
                self._log_observation_keys_once(measured_observation)
                measured = extract_arm_state_from_lerobot_observation(measured_observation)[
                    "joint_positions_rad"
                ]
                tracking_error = abs(measured[joint_name] - positions[joint_name])
                bad_tracking_count = bad_tracking_count + 1 if tracking_error > threshold_rad else 0
                if bad_tracking_count >= 3:
                    return response(
                        False,
                        "tracking_error_exceeded",
                        active_joint_name=joint_name,
                        tracking_error_rad=float(tracking_error),
                    )
                time.sleep(1.0 / 20.0)
            positions[joint_name] = target
        final_state = self.get_state()
        if not final_state["success"]:
            return final_state
        return response(True, "motion_completed")

    def _log_observation_keys_once(self, observation: dict[str, Any]) -> None:
        if self.logged_observation_keys:
            return
        ordered = [LEROBOT_POSITION_KEYS[name] for name in ARM_JOINT_NAMES] + ["gripper.pos"]
        available = [key for key in ordered if key in observation]
        print(f"LEROBOT_OBSERVATION_KEYS={available}", flush=True)
        self.logged_observation_keys = True

    def _log_action_keys_once(self, action: dict[str, float]) -> None:
        if self.logged_action_keys:
            return
        print(f"LEROBOT_ACTION_KEYS={list(action.keys())}", flush=True)
        self.logged_action_keys = True


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
        self.next_client_id = 0
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
            self._print_startup()
            try:
                while not self.stop_event.is_set():
                    try:
                        conn, addr = sock.accept()
                    except socket.timeout:
                        continue
                    self.next_client_id += 1
                    client_id = self.next_client_id
                    print(f"TCP_CLIENT_CONNECTED id={client_id} address={addr}", flush=True)
                    try:
                        self._serve_client(conn, client_id)
                    finally:
                        try:
                            conn.close()
                        except OSError:
                            pass
                        print(f"TCP_CLIENT_DISCONNECTED id={client_id}", flush=True)
            finally:
                self.backend.close()
                print("SERVER COUNTERS", flush=True)
                print(json.dumps(self.counters(), sort_keys=True), flush=True)
                print(STOP_BANNER, flush=True)

    def _print_startup(self) -> None:
        print("MVP TCP SERVER STARTING", flush=True)
        print(f"server_host={self.host}", flush=True)
        print(f"server_port={self.port}", flush=True)
        print(
            f"server_mode={'motion_enabled' if self.hardware_motion_enabled else 'read_only'}",
            flush=True,
        )
        print(f"hardware_motion_enabled={bool_text(self.hardware_motion_enabled)}", flush=True)
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
        print(f"TCP_LISTENING host={self.host} port={self.port} single_client=true", flush=True)

    def shutdown(self) -> None:
        self.stop_event.set()
        try:
            if self.server_socket is not None:
                self.server_socket.close()
        except OSError:
            pass

    def _serve_client(self, conn: socket.socket, client_id: int) -> None:
        conn.settimeout(20.0)
        with conn.makefile("rwb") as stream:
            while not self.stop_event.is_set():
                try:
                    line = stream.readline()
                    if not line:
                        return
                    result = self.handle_line(line.decode("utf-8", errors="replace"), client_id)
                    payload = self.to_jsonable(result)
                    stream.write((json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8"))
                    stream.flush()
                    self._log_response(client_id, payload)
                except Exception as exc:
                    print(
                        f"TCP_CLIENT_ERROR id={client_id} type={type(exc).__name__} message={exc}",
                        flush=True,
                    )
                    return

    def handle_line(self, line: str, client_id: int = 0) -> dict[str, Any]:
        try:
            request = json.loads(line)
        except json.JSONDecodeError:
            self._log_request(None, client_id)
            return response(False, "malformed_json")
        if not isinstance(request, dict):
            self._log_request(None, client_id)
            return response(False, "invalid_request")
        self._log_request(request, client_id)
        command = request.get("command")
        if command == "get_state":
            self.get_state_request_count += 1
            try:
                return self.backend.get_state()
            except Exception as exc:
                return self._application_error(client_id, exc, "server_state_error")
        if command == "move_joints_sequential":
            return self._handle_move(request, client_id)
        if command == "stop":
            return response(False, "unsupported_command")
        return response(False, "unknown_command")

    def _log_request(self, request: dict[str, Any] | None, client_id: int) -> None:
        del request, client_id

    def _log_response(self, client_id: int, result: dict[str, Any]) -> None:
        del client_id, result

    def _application_error(self, client_id: int, exc: Exception, prefix: str) -> dict[str, Any]:
        message = f"{prefix}:{type(exc).__name__}:{exc}"[:300]
        print(
            f"TCP_APPLICATION_ERROR id={client_id} type={type(exc).__name__} message={exc}",
            flush=True,
        )
        return response(False, message)

    def _handle_move(self, request: dict[str, Any], client_id: int) -> dict[str, Any]:
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
            return response(False, "invalid_speed_rad_s")
        if not math.isfinite(speed) or speed <= 0.0:
            return response(False, "invalid_speed_rad_s")
        if speed > MAX_SPEED_RAD_S:
            return response(False, "invalid_speed_rad_s")

        gripper_target_pos = None
        if "gripper_target_pos" in request:
            try:
                gripper_target_pos = float(request.get("gripper_target_pos"))
            except (TypeError, ValueError):
                return response(False, "invalid_gripper_target_pos")
            if not math.isfinite(gripper_target_pos):
                return response(False, "invalid_gripper_target_pos")
            if not 0.0 <= gripper_target_pos <= 100.0:
                return response(False, "gripper_target_out_of_calibration_range")

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
            if gripper_target_pos is not None and not executor.all_targets_within_calibration(
                targets,
                gripper_target_pos,
            ):
                return response(False, "gripper_target_out_of_calibration_range")

        print(f"MOTION_INPUT_KEYS logical_joint_names={list(ARM_JOINT_NAMES)}", flush=True)
        print(f"MOTION_STARTED speed_rad_s={speed}", flush=True)
        started = time.monotonic()
        try:
            result = self.backend.move_joints_sequential(
                target_rad,
                speed,
                joint_order,
                gripper_target_pos,
            )
        except Exception as exc:
            return self._application_error(client_id, exc, "server_motion_error")
        duration = time.monotonic() - started
        if result.get("success"):
            self.move_request_executed_count += 1
            result.setdefault("reason", "motion_completed")
            result.setdefault("duration_s", duration)
            print(f"MOTION_COMPLETED duration_s={duration:.3f}", flush=True)
        return result

    @staticmethod
    def to_jsonable(value: Any) -> Any:
        if value is None or isinstance(value, (str, int, float, bool)):
            return value
        if isinstance(value, dict):
            return {str(key): MvpTcpServer.to_jsonable(item) for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [MvpTcpServer.to_jsonable(item) for item in value]
        if hasattr(value, "item"):
            return MvpTcpServer.to_jsonable(value.item())
        if hasattr(value, "tolist"):
            return MvpTcpServer.to_jsonable(value.tolist())
        return str(value)

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

    load_config(args.config)
    host = "127.0.0.1"
    port = 8770
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
