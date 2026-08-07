from __future__ import annotations

import argparse
import importlib.util
import json
import math
import socket
import struct
import sys
import threading
import time
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

import numpy as np

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
DEBUG_TACTILE = False
GUARD_PACKET_STRUCT = struct.Struct("!4sBBI")
GUARD_MAGIC = b"GRIP"
GUARD_VERSION = 1
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
        stop_gripper_on_tactile_contact: bool = False,
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


@dataclass(frozen=True)
class TactileSnapshot:
    ready: bool
    contact_detected: bool
    contact_score: float
    state_age_s: float | None
    error: str | None
    status: str
    source: str
    port: str | None = None
    frame_count: int = 0

    def to_tcp_fields(self) -> dict[str, Any]:
        return {
            "tactile_ready": bool(self.ready),
            "tactile_contact_detected": bool(self.contact_detected),
            "tactile_contact_score": float(self.contact_score),
            "tactile_state_age_s": self.state_age_s,
            "tactile_error": self.error,
            "tactile_status": self.status,
            "tactile_source": self.source,
            "tactile_port": self.port,
            "tactile_frame_count": int(self.frame_count),
        }


class TactileUdpGuardReceiver:
    """Non-blocking receiver compatible with the old SO-101 FlexiTac UDP guard."""

    def __init__(self, host: str, port: int, timeout_s: float) -> None:
        self.host = host
        self.port = int(port)
        self.timeout_s = float(timeout_s)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind((self.host, self.port))
        self._socket.setblocking(False)
        self._contact = False
        self._sequence: int | None = None
        self._last_receive_time: float | None = None

    def poll(self) -> TactileSnapshot:
        newest_state: bool | None = None
        newest_sequence: int | None = None
        while True:
            try:
                packet, _ = self._socket.recvfrom(1024)
            except BlockingIOError:
                break
            parsed = self._parse_packet(packet)
            if parsed is None:
                continue
            newest_state, newest_sequence = parsed
        if newest_state is not None:
            self._contact = newest_state
            self._sequence = newest_sequence
            self._last_receive_time = time.monotonic()
        age = None if self._last_receive_time is None else time.monotonic() - self._last_receive_time
        ready = age is not None and age <= self.timeout_s
        status = "udp_guard_packet"
        if self._sequence is not None:
            status = f"{status}:sequence={self._sequence}"
        return TactileSnapshot(
            ready=bool(ready),
            contact_detected=bool(self._contact),
            contact_score=1.0 if self._contact else 0.0,
            state_age_s=None if age is None else float(age),
            error=None if ready else "tactile_state_unavailable_or_stale",
            status=status,
            source=f"udp_guard:{self.host}:{self.port}",
        )

    def _parse_packet(self, packet: bytes) -> tuple[bool, int] | None:
        if len(packet) != GUARD_PACKET_STRUCT.size:
            return None
        magic, version, state, sequence = GUARD_PACKET_STRUCT.unpack(packet)
        if magic != GUARD_MAGIC or version != GUARD_VERSION or state not in (0, 1):
            return None
        return bool(state), int(sequence)

    def close(self) -> None:
        self._socket.close()


class TactileRuntime:
    def __init__(self, config: dict[str, Any]) -> None:
        tactile_config = config.get("tactile", {})
        if not isinstance(tactile_config, dict):
            tactile_config = {}
        self.enabled = bool(tactile_config.get("enabled", True))
        self.source = str(tactile_config.get("source", "direct_serial"))
        self.port = str(tactile_config.get("port", "COM8"))
        self.baudrate = int(tactile_config.get("baudrate", 2_000_000))
        self.rows = int(tactile_config.get("rows", 12))
        self.cols = int(tactile_config.get("cols", 32))
        self.baseline_frames = int(tactile_config.get("baseline_frames", 30))
        self.serial_timeout_s = float(tactile_config.get("serial_timeout_s", 0.05))
        self.state_max_age_s = float(tactile_config.get("state_max_age_s", 0.25))
        self.top_k = int(tactile_config.get("top_k", 20))
        self.contact_on_threshold = float(tactile_config.get("contact_on_threshold", 40.0))
        self.contact_off_threshold = float(tactile_config.get("contact_off_threshold", 30.0))
        self.contact_confirm_frames = int(tactile_config.get("contact_confirm_frames", 3))
        self.release_confirm_frames = int(tactile_config.get("release_confirm_frames", 5))
        self.reader_source = str(tactile_config.get("frame_reader_source", "Evo-RL/src/lerobot/utils/flexitac_reader.py"))
        self.contact_logic_source = str(tactile_config.get("contact_logic_source", "Evo-RL/src/lerobot/utils/episode_success_source.py"))
        self.reader: Any | None = None
        self.monitor_thread: threading.Thread | None = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.error: str | None = None
        self.ready = False
        self.contact_detected = False
        self.contact_score = 0.0
        self.last_update_monotonic_s: float | None = None
        self.frame_count = 0
        self.above_count = 0
        self.below_count = 0
        self.last_logged_contact: bool | None = None

    def connect(self) -> None:
        if not self.enabled:
            self.error = "tactile_disabled"
            return
        print(
            f"TACTILE_CONFIG source={self.source} port={self.port} baudrate={self.baudrate} rows={self.rows} cols={self.cols}",
            flush=True,
        )
        if self.source != "direct_serial":
            self.error = f"unsupported_tactile_source:{self.source}"
            return
        if self.port.upper() != "COM8":
            self.error = f"tactile_port_must_be_COM8:configured={self.port}"
            print(
                f"TACTILE_READER_INIT_FAILED port={self.port} error_type=ValueError error={self.error} stage=reader_init",
                flush=True,
            )
            return
        try:
            reader_cls = self._load_existing_flexitac_reader()
            print("TACTILE_MODULE_LOADED", flush=True)
        except Exception as exc:
            self.reader = None
            self.error = f"tactile_module_load_failed:{type(exc).__name__}:{exc}"
            print(
                f"TACTILE_MODULE_LOAD_FAILED module_path={self._reader_path()} error_type={type(exc).__name__} error={exc} stage=module_load",
                flush=True,
            )
            self._print_debug_traceback()
            return
        try:
            self.reader = reader_cls(
                port=self.port,
                baud=self.baudrate,
                rows=self.rows,
                cols=self.cols,
                baseline_frames=self.baseline_frames,
            )
            print(f"TACTILE_READER_INITIALIZED port={self.port}", flush=True)
        except Exception as exc:
            self.reader = None
            self.error = f"tactile_reader_init_failed:{type(exc).__name__}:{exc}"
            print(
                f"TACTILE_READER_INIT_FAILED port={self.port} error_type={type(exc).__name__} error={exc} stage=reader_init",
                flush=True,
            )
            self._print_debug_traceback()
            return
        try:
            print(f"TACTILE_SERIAL_OPENING port={self.port}", flush=True)
            print("DO_NOT_TOUCH_FLEXITAC_DURING_BASELINE", flush=True)
            print(f"TACTILE_BASELINE_STARTED frames={self.baseline_frames}", flush=True)
            self.reader.start()
            print(f"TACTILE_SERIAL_OPENED port={self.port}", flush=True)
            print("TACTILE_BASELINE_COMPLETED", flush=True)
            self.stop_event.clear()
            self.monitor_thread = threading.Thread(
                target=self._monitor_loop,
                name="mvp-flexitac-contact-monitor",
                daemon=True,
            )
            self.monitor_thread.start()
            print("TACTILE_READER_STARTED", flush=True)
            self.error = None
            with self.lock:
                self.ready = True
            print("TACTILE_READY true", flush=True)
        except Exception as exc:
            stage, event, possible_reason = self._classify_reader_start_error(exc)
            self.reader = None
            self.error = f"{stage}:{type(exc).__name__}:{exc}"
            possible_text = "" if possible_reason is None else f" possible_reason={possible_reason}"
            print(
                f"{event} port={self.port} error_type={type(exc).__name__} error={exc} stage={stage}{possible_text}",
                flush=True,
            )
            self._print_debug_traceback()

    def close(self) -> None:
        self.stop_event.set()
        if self.monitor_thread is not None:
            self.monitor_thread.join(timeout=1.0)
        self.monitor_thread = None
        if self.reader is not None:
            self.reader.close()
        self.reader = None
        with self.lock:
            self.ready = False

    def snapshot(self) -> TactileSnapshot:
        with self.lock:
            age = None if self.last_update_monotonic_s is None else time.monotonic() - self.last_update_monotonic_s
            ready = bool(self.ready and age is not None and age <= self.state_max_age_s)
            contact = bool(self.contact_detected)
            score = float(self.contact_score)
            frame_count = int(self.frame_count)
            error = self.error
        if not ready and error is None:
            error = "tactile_state_unavailable_or_stale"
        status = (
            f"source={self.source};port={self.port};ready={bool(ready)};contact={bool(contact)};"
            f"score={score:.3f};age_s={'' if age is None else f'{age:.3f}'};error={'' if error is None else error};"
            f"frame_count={frame_count}"
        )
        return TactileSnapshot(
            ready=ready,
            contact_detected=contact,
            contact_score=score,
            state_age_s=None if age is None else float(age),
            error=error,
            status=status,
            source=self.source,
            port=self.port,
            frame_count=frame_count,
        )

    def _reader_path(self) -> Path:
        return (PROJECT_ROOT.parents[0] / self.reader_source).resolve()

    def _load_existing_flexitac_reader(self) -> type:
        reader_path = self._reader_path()
        if not reader_path.is_file():
            raise RuntimeError(f"existing_flexitac_reader_not_found:{reader_path}")
        module_name = "so101_mvp_reused_flexitac_reader"
        spec = importlib.util.spec_from_file_location(module_name, reader_path)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"existing_flexitac_reader_import_failed:{reader_path}")
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        try:
            spec.loader.exec_module(module)
        except Exception:
            sys.modules.pop(module_name, None)
            raise
        reader_cls = getattr(module, "FlexiTacReader", None)
        if reader_cls is None:
            raise RuntimeError("FlexiTacReader class missing from existing source")
        return reader_cls

    def _classify_reader_start_error(self, exc: Exception) -> tuple[str, str, str | None]:
        root = exc.__cause__ or exc
        text = str(root).lower()
        type_name = type(root).__name__
        if type_name in {"PermissionError"} or "access is denied" in text or "permission" in text:
            return "serial_open", "TACTILE_SERIAL_OPEN_FAILED", "port_in_use"
        if type_name in {"FileNotFoundError"} or "cannot find" in text or "could not open port" in text:
            return "serial_open", "TACTILE_SERIAL_OPEN_FAILED", "wrong_port_or_device_disconnected"
        if "serial" in type_name.lower() or "serial" in text:
            return "serial_open", "TACTILE_SERIAL_OPEN_FAILED", "serial_configuration_or_driver_error"
        if isinstance(root, TimeoutError) or "baseline" in text or "timed out waiting for a complete flexitac frame" in text:
            return "baseline", "TACTILE_BASELINE_FAILED", None
        return "reader_start", "TACTILE_READER_START_FAILED", None

    def _print_debug_traceback(self) -> None:
        if DEBUG_TACTILE:
            traceback.print_exc()

    def _monitor_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                if self.reader is None:
                    time.sleep(0.01)
                    continue
                delta = self.reader.get_latest_delta()
                if delta is None:
                    time.sleep(0.01)
                    continue
                score = self._calculate_top_k_score(delta)
                changed = self._update_contact_state(score)
                with self.lock:
                    self.contact_score = score
                    self.last_update_monotonic_s = time.monotonic()
                    self.frame_count += 1
                    contact = self.contact_detected
                    self.error = None
                if changed or self.last_logged_contact is None:
                    print(f"TACTILE_CONTACT_CHANGED contact={str(contact).lower()} score={score:.2f}", flush=True)
                    self.last_logged_contact = contact
                time.sleep(0.005)
            except Exception as exc:
                with self.lock:
                    self.error = f"tactile_reader_failed:{type(exc).__name__}:{exc}"
                    self.ready = False
                return

    def _calculate_top_k_score(self, delta: np.ndarray) -> float:
        flat = np.asarray(delta, dtype=np.float32).reshape(-1)
        top_k = min(self.top_k, flat.size)
        top_values = np.partition(flat, -top_k)[-top_k:]
        return float(top_values.mean())

    def _update_contact_state(self, score: float) -> bool:
        changed = False
        with self.lock:
            if not self.contact_detected:
                if score >= self.contact_on_threshold:
                    self.above_count += 1
                else:
                    self.above_count = 0
                if self.above_count >= self.contact_confirm_frames:
                    self.contact_detected = True
                    self.above_count = 0
                    self.below_count = 0
                    changed = True
            else:
                if score <= self.contact_off_threshold:
                    self.below_count += 1
                else:
                    self.below_count = 0
                if self.below_count >= self.release_confirm_frames:
                    self.contact_detected = False
                    self.above_count = 0
                    self.below_count = 0
                    changed = True
        return changed


class ReadOnlyFeetechBackend:
    def __init__(self, config_path: Path) -> None:
        self.executor = MvpSo101HardwareExecutor(config_path)
        self.config = load_config(config_path)
        self.tactile = TactileRuntime(self.config)
        self.bus: Any | None = None
        self.connected = False
        self.stop_requested = False
        self.detected_usb_serial = ""
        self.opened_com_ports = False
        self.goal_position_write_count = 0
        self.torque_enable_write_count = 0
        self.torque_disable_write_count = 0

    def connect(self) -> None:
        self.tactile.connect()
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
        self.tactile.close()

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
            **self.tactile.snapshot().to_tcp_fields(),
        )

    def move_joints_sequential(
        self,
        target_rad: list[float],
        speed_rad_s: float,
        joint_order: list[int],
        gripper_target_pos: float | None = None,
        stop_gripper_on_tactile_contact: bool = False,
    ) -> dict[str, Any]:
        del target_rad, speed_rad_s, joint_order, gripper_target_pos, stop_gripper_on_tactile_contact
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
            "tactile_source": self.tactile.source,
            "tactile_port": self.tactile.port,
            "tactile_baudrate": self.tactile.baudrate,
            "tactile_frame_count": self.tactile.snapshot().frame_count,
        }


class MotionFeetechBackend(ReadOnlyFeetechBackend):
    def __init__(self, config_path: Path) -> None:
        super().__init__(config_path)
        self.robot: Any | None = None
        self.logged_observation_keys = False
        self.logged_action_keys = False

    def connect(self) -> None:
        self.tactile.connect()
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
        self.tactile.close()

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
            **self.tactile.snapshot().to_tcp_fields(),
        )

    def move_joints_sequential(
        self,
        target_rad: list[float],
        speed_rad_s: float,
        joint_order: list[int],
        gripper_target_pos: float | None = None,
        stop_gripper_on_tactile_contact: bool = False,
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
                if gripper_target_pos is not None and stop_gripper_on_tactile_contact:
                    tactile = self.tactile.snapshot()
                    if not tactile.ready:
                        return response(
                            False,
                            "tactile_unavailable_during_gripper_close",
                            gripper_stop_triggered=False,
                            gripper_stop_position=float(gripper_command),
                            **tactile.to_tcp_fields(),
                        )
                    if tactile.contact_detected:
                        hold_action = build_lerobot_action(positions, gripper_command)
                        self.robot.send_action(hold_action)
                        self.goal_position_write_count += 1
                        return response(
                            True,
                            "tactile_contact_stop",
                            gripper_stop_triggered=True,
                            gripper_stop_position=float(gripper_command),
                            gripper_target_pos=float(target_gripper),
                            gripper_contact_preload_offset=0.0,
                            **tactile.to_tcp_fields(),
                        )
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
        if gripper_target_pos is not None and stop_gripper_on_tactile_contact:
            return response(
                False,
                "gripper_closed_without_tactile_contact",
                gripper_stop_triggered=False,
                gripper_stop_position=float(target_gripper),
                **self.tactile.snapshot().to_tcp_fields(),
            )
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
        self.active_client_id: int | None = None
        self.active_client_lock = threading.Lock()
        self.client_threads: list[threading.Thread] = []

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
                    with self.active_client_lock:
                        active_client_id = self.active_client_id
                        if active_client_id is None:
                            self.active_client_id = client_id
                    if active_client_id is not None:
                        print(
                            f"TCP_CLIENT_REJECTED id={client_id} peer={addr} reason=active_client_exists active_id={active_client_id}",
                            flush=True,
                        )
                        conn.close()
                        continue
                    print(f"TCP_CLIENT_CONNECTED id={client_id} peer={addr}", flush=True)
                    thread = threading.Thread(
                        target=self._client_thread,
                        args=(conn, client_id),
                        daemon=True,
                    )
                    self.client_threads.append(thread)
                    thread.start()
            finally:
                for thread in self.client_threads:
                    thread.join(timeout=1.0)
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
        print(f"TCP_SERVER_LISTENING host={self.host} port={self.port}", flush=True)
        print(f"TCP_LISTENING host={self.host} port={self.port} single_client=true", flush=True)

    def shutdown(self) -> None:
        self.stop_event.set()
        try:
            if self.server_socket is not None:
                self.server_socket.close()
        except OSError:
            pass

    def _client_thread(self, conn: socket.socket, client_id: int) -> None:
        disconnect_reason = "normal"
        try:
            disconnect_reason = self._serve_client(conn, client_id)
        finally:
            try:
                conn.close()
            except OSError:
                pass
            with self.active_client_lock:
                if self.active_client_id == client_id:
                    self.active_client_id = None
            print(
                f"TCP_CLIENT_DISCONNECTED id={client_id} reason={disconnect_reason}",
                flush=True,
            )

    def _serve_client(self, conn: socket.socket, client_id: int) -> str:
        conn.settimeout(None)
        with conn.makefile("rwb") as stream:
            while not self.stop_event.is_set():
                try:
                    line = stream.readline()
                    if not line:
                        print(f"TCP_CLIENT_EOF id={client_id}", flush=True)
                        return "eof"
                    result = self.handle_line(line.decode("utf-8", errors="replace"), client_id)
                    payload = self.to_jsonable(result)
                    stream.write((json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8"))
                    stream.flush()
                    self._log_response(client_id, payload)
                except Exception as exc:
                    print(
                        f"TCP_SERVER_ERROR error_type={type(exc).__name__} error={exc}",
                        flush=True,
                    )
                    return f"{type(exc).__name__}:{exc}"
        return "server_stop"

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
                result = self.backend.get_state()
                result.setdefault("command", "get_state")
                return result
            except Exception as exc:
                return self._application_error(client_id, exc, "server_state_error")
        if command == "move_joints_sequential":
            return self._handle_move(request, client_id)
        if command == "stop":
            return response(False, "unsupported_command")
        return response(False, "unknown_command")

    def _log_request(self, request: dict[str, Any] | None, client_id: int) -> None:
        command = "malformed" if request is None else str(request.get("command", "missing"))
        print(f"TCP_REQUEST_RECEIVED command={command} id={client_id}", flush=True)

    def _log_response(self, client_id: int, result: dict[str, Any]) -> None:
        command = str(result.get("command", "unknown"))
        reason = str(result.get("reason", "missing_reason"))
        print(
            f"TCP_RESPONSE_SENT command={command} id={client_id} success={bool_text(bool(result.get('success', False)))} reason={reason}",
            flush=True,
        )

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
                bool(request.get("stop_gripper_on_tactile_contact", False)),
            )
        except Exception as exc:
            return self._application_error(client_id, exc, "server_motion_error")
        duration = time.monotonic() - started
        result.setdefault("command", "move_joints_sequential")
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
    parser.add_argument(
        "--debug-tactile",
        action="store_true",
        help="Print full tactile initialization traceback on failure.",
    )
    args = parser.parse_args()
    global DEBUG_TACTILE
    DEBUG_TACTILE = bool(args.debug_tactile)

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
