from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import math
import socket
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from xml.etree import ElementTree as ET


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MAPPING = PROJECT_ROOT / "config" / "real_joint_state_mapping.json"
REPORT_DIR = PROJECT_ROOT / "data" / "verification"
PREFLIGHT_REPORT = REPORT_DIR / "stage_2d3b_bus_preflight_report.json"
PREFLIGHT_LOG = REPORT_DIR / "stage_2d3b_bus_preflight.log"
MAPPING_DUMP = REPORT_DIR / "stage_2d3b_joint_mapping_dump.json"
MAPPING_REPORT = REPORT_DIR / "stage_2d3b_joint_mapping_report.json"
MAPPING_LOG = REPORT_DIR / "stage_2d3b_joint_mapping.log"
CALIBRATION_SUMMARY = REPORT_DIR / "my_follower_calibration_summary.json"
URDF_PATH = PROJECT_ROOT / "data" / "robot_model" / "so101" / "so101_new_calib.urdf"
EXPECTED_URDF_SHA256 = "3a65d2d35e68a8d2f0c2cc176d19b884506543c93ba72980145b80abe276022c"
LEROBOT_SRC = PROJECT_ROOT.parents[0] / "repos" / "lerobot" / "src"
TELEOP_CONFIG = (
    PROJECT_ROOT.parents[0]
    / "repos"
    / "lerobot"
    / "so101_sparse_tactile"
    / "configs"
    / "tactile_teleop_force_close.yaml"
)
ARM_JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]
OPTIONAL_GRIPPER_NAME = "gripper"
DEFAULT_MOTOR_MODEL = "sts3215"
DEFAULT_BAUDRATE = 1_000_000
DEFAULT_PROTOCOL_VERSION = 0
FEETECH_RESOLUTION = 4096


@dataclass(frozen=True)
class ReadOnlyConfig:
    follower_port: str
    calibration_path: Path
    bind_host: str
    bind_port: int
    rate_hz: float
    mapping_status: str
    baudrate: int
    protocol_version: int
    motor_model: str
    read_retry_count: int
    reconnect_backoff_s: float
    maximum_consecutive_read_failures: int
    fault_policy: str


def iso_timestamp() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat()


def ensure_report_dir() -> None:
    REPORT_DIR.mkdir(parents=True, exist_ok=True)


def log_json(payload: dict[str, Any]) -> None:
    ensure_report_dir()
    line = json.dumps(payload, ensure_ascii=False, allow_nan=False)
    print(line, flush=True)
    with PREFLIGHT_LOG.open("a", encoding="utf-8") as file:
        file.write(line + "\n")


def log_mapping(payload: dict[str, Any]) -> None:
    ensure_report_dir()
    line = json.dumps(payload, ensure_ascii=False, allow_nan=False)
    print(line, flush=True)
    with MAPPING_LOG.open("a", encoding="utf-8") as file:
        file.write(line + "\n")


def load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"Expected JSON object in {path}")
    return value


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_urdf_joint_limits() -> dict[str, dict[str, float]]:
    tree = ET.parse(URDF_PATH)
    root = tree.getroot()
    limits: dict[str, dict[str, float]] = {}
    for joint in root.findall("joint"):
        name = joint.attrib.get("name")
        if name not in ARM_JOINT_NAMES:
            continue
        limit = joint.find("limit")
        if limit is None:
            continue
        limits[name] = {
            "lower": float(limit.attrib["lower"]),
            "upper": float(limit.attrib["upper"]),
        }
    missing = [name for name in ARM_JOINT_NAMES if name not in limits]
    if missing:
        raise ValueError(f"URDF missing joint limits for: {missing}")
    return limits


def parse_scale(value: Any) -> float:
    text = str(value)
    if text == "pi/180":
        return math.pi / 180.0
    return float(value)


def load_joint_mapping(path: Path = DEFAULT_MAPPING) -> dict[str, dict[str, float]]:
    raw = load_json(path)
    mapping: dict[str, dict[str, float]] = {}
    for item in raw.get("joint_mappings", []):
        if not isinstance(item, dict):
            continue
        target_name = str(item["target_name"])
        mapping[target_name] = {
            "source_name": str(item["source_name"]),
            "sign": float(item.get("sign", 1.0)),
            "offset_rad": float(item.get("offset_rad", 0.0)),
            "scale": parse_scale(item.get("scale", "pi/180")),
        }
    missing = [name for name in ARM_JOINT_NAMES if name not in mapping]
    if missing:
        raise ValueError(f"Mapping missing joints: {missing}")
    return mapping


def raw_encoder_to_uncalibrated_degrees(raw_value: float) -> float:
    midpoint = (FEETECH_RESOLUTION - 1) / 2.0
    return (float(raw_value) - midpoint) * 360.0 / (FEETECH_RESOLUTION - 1)


def build_mapping_diagnostics(
    *,
    raw_values: dict[str, Any],
    calibrated_degrees: dict[str, Any],
    mapping_path: Path = DEFAULT_MAPPING,
) -> dict[str, Any]:
    joint_limits = load_urdf_joint_limits()
    mapping = load_joint_mapping(mapping_path)
    source_positions = {
        name: float(calibrated_degrees[name])
        for name in ARM_JOINT_NAMES
    }
    raw_encoder_values = {
        name: float(raw_values[name])
        for name in ARM_JOINT_NAMES
    }
    motor_degrees_before_calibration = {
        name: raw_encoder_to_uncalibrated_degrees(raw_encoder_values[name])
        for name in ARM_JOINT_NAMES
    }
    mapped_positions_rad: dict[str, float] = {}
    per_joint_validation: dict[str, dict[str, Any]] = {}
    violating_joints: list[str] = []
    for name in ARM_JOINT_NAMES:
        joint_mapping = mapping[name]
        source_value = source_positions[str(joint_mapping["source_name"])]
        mapped_rad = (
            joint_mapping["sign"] * source_value * joint_mapping["scale"]
            + joint_mapping["offset_rad"]
        )
        lower = joint_limits[name]["lower"]
        upper = joint_limits[name]["upper"]
        lower_margin = mapped_rad - lower
        upper_margin = upper - mapped_rad
        within_limits = bool(lower_margin >= -1.0e-10 and upper_margin >= -1.0e-10)
        if not within_limits:
            violating_joints.append(name)
        mapped_positions_rad[name] = mapped_rad
        per_joint_validation[name] = {
            "source_value": source_value,
            "source_unit": "lerobot_calibrated_degrees",
            "raw_encoder_value": raw_encoder_values[name],
            "motor_degrees_before_calibration": motor_degrees_before_calibration[name],
            "mapped_rad": mapped_rad,
            "urdf_lower_rad": lower,
            "urdf_upper_rad": upper,
            "lower_margin_rad": lower_margin,
            "upper_margin_rad": upper_margin,
            "within_limits": within_limits,
            "sign": joint_mapping["sign"],
            "offset_rad": joint_mapping["offset_rad"],
            "scale": joint_mapping["scale"],
        }
    mapped_joint_state_valid = len(violating_joints) == 0
    return {
        "raw_encoder_values": raw_encoder_values,
        "motor_degrees_before_calibration": motor_degrees_before_calibration,
        "lerobot_calibrated_degrees": source_positions,
        "source_positions": source_positions,
        "source_unit": "lerobot_calibrated_degrees",
        "calibration_applied": True,
        "mapped_positions_rad": mapped_positions_rad,
        "positions_rad": [mapped_positions_rad[name] for name in ARM_JOINT_NAMES],
        "per_joint_validation": per_joint_validation,
        "violating_joints": violating_joints,
        "mapped_joint_state_valid": mapped_joint_state_valid,
        "mapping_failure_reason": (
            None if mapped_joint_state_valid else "current_joint_state_out_of_bounds"
        ),
    }


def write_report(report: dict[str, Any]) -> None:
    ensure_report_dir()
    PREFLIGHT_REPORT.write_text(
        json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False),
        encoding="utf-8",
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read SO-101 follower Present_Position registers and expose them as "
            "localhost JSON Lines. This script is intentionally read-only."
        )
    )
    parser.add_argument("--mapping", type=Path, default=DEFAULT_MAPPING)
    parser.add_argument("--follower-port", default=None)
    parser.add_argument("--calibration-path", type=Path, default=None)
    parser.add_argument("--bind-host", default="127.0.0.1")
    parser.add_argument("--bind-port", type=int, default=8766)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--allow-unlisted-port", action="store_true")
    parser.add_argument("--list-ports", action="store_true")
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--dump-once", action="store_true")
    return parser.parse_args()


def build_config(args: argparse.Namespace) -> ReadOnlyConfig:
    mapping = load_json(args.mapping)
    calibration_path = args.calibration_path or Path(str(mapping["calibration_path"]))
    follower_port = str(args.follower_port or mapping["follower_port"])
    rate_hz = float(args.rate_hz)
    if not math.isfinite(rate_hz) or rate_hz <= 0.0:
        raise ValueError("--rate-hz must be finite and positive")
    return ReadOnlyConfig(
        follower_port=follower_port,
        calibration_path=calibration_path,
        bind_host=str(args.bind_host),
        bind_port=int(args.bind_port),
        rate_hz=rate_hz,
        mapping_status=str(mapping.get("status", "unknown")),
        baudrate=int(mapping.get("baudrate", DEFAULT_BAUDRATE)),
        protocol_version=int(mapping.get("protocol_version", DEFAULT_PROTOCOL_VERSION)),
        motor_model=str(mapping.get("motor_model", DEFAULT_MOTOR_MODEL)),
        read_retry_count=int(mapping.get("read_retry_count", 3)),
        reconnect_backoff_s=float(mapping.get("reconnect_backoff_s", 1.0)),
        maximum_consecutive_read_failures=int(
            mapping.get("maximum_consecutive_read_failures", 5)
        ),
        fault_policy=str(mapping.get("fault_policy", "stay_alive_fault")),
    )


def serial_ports() -> list[dict[str, Any]]:
    import serial.tools.list_ports as list_ports

    ports: list[dict[str, Any]] = []
    for port in list_ports.comports():
        ports.append(
            {
                "device": str(port.device),
                "description": str(port.description),
                "hwid": str(port.hwid),
                "vid": port.vid,
                "pid": port.pid,
                "serial_number": port.serial_number,
                "manufacturer": port.manufacturer,
                "product": port.product,
                "location": port.location,
            }
        )
    return ports


def port_names(ports: list[dict[str, Any]]) -> list[str]:
    return [str(port["device"]) for port in ports]


def find_port_info(ports: list[dict[str, Any]], device: str) -> dict[str, Any] | None:
    for port in ports:
        if str(port["device"]).upper() == device.upper():
            return port
    return None


def classify_serial_ports(ports: list[dict[str, Any]], configured_follower_port: str) -> dict[str, Any]:
    configured = find_port_info(ports, configured_follower_port)
    ch340 = [port for port in ports if "CH340" in str(port.get("description", ""))]
    ch343 = [port for port in ports if "CH343" in str(port.get("description", ""))]
    return {
        "configured_follower_port_info": configured,
        "possible_so101_usb_serial_ports": [
            port for port in ports if port.get("vid") == 0x1A86
        ],
        "possible_flexitac_ports": ch340,
        "possible_so101_bus_ports": ch343,
        "leader_port_candidates": [],
        "follower_port_candidates": [configured] if configured else [],
        "classification_basis": (
            "Configured follower_port is used only when present. CH343/CH340 "
            "classification is diagnostic, not proof of Leader/Follower identity."
        ),
    }


def print_list_ports(config: ReadOnlyConfig) -> int:
    ports = serial_ports()
    payload = {
        "status": "SERIAL_PORT_LIST",
        "timestamp": iso_timestamp(),
        "current_serial_ports": ports,
        "configured_follower_port": config.follower_port,
        "configured_port_exists": config.follower_port in port_names(ports),
        "port_classification": classify_serial_ports(ports, config.follower_port),
        "opened_motor_bus": False,
        "motion_command_sent": False,
    }
    log_json(payload)
    return 0


def import_lerobot_bus_types() -> tuple[Any, Any, Any, Any]:
    if str(LEROBOT_SRC) not in sys.path:
        sys.path.insert(0, str(LEROBOT_SRC))
    from lerobot.motors.feetech.feetech import FeetechMotorsBus
    from lerobot.motors.motors_bus import Motor, MotorCalibration, MotorNormMode

    return FeetechMotorsBus, Motor, MotorCalibration, MotorNormMode


def load_lerobot_calibration(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"Calibration file not found: {path}")
    raw = load_json(path)
    required_names = list(ARM_JOINT_NAMES)
    missing = [name for name in required_names if name not in raw]
    if missing:
        raise ValueError(f"Calibration file missing arm joints: {missing}")

    _, _, MotorCalibration, _ = import_lerobot_bus_types()
    calibration: dict[str, Any] = {}
    for name, item in raw.items():
        if name not in required_names and name != OPTIONAL_GRIPPER_NAME:
            continue
        calibration[name] = MotorCalibration(
            id=int(item["id"]),
            drive_mode=int(item["drive_mode"]),
            homing_offset=int(item["homing_offset"]),
            range_min=int(item["range_min"]),
            range_max=int(item["range_max"]),
        )
    return calibration


def write_calibration_summary(path: Path) -> dict[str, Any]:
    raw = load_json(path)
    motor_names = list(raw)
    summary = {
        "timestamp": iso_timestamp(),
        "calibration_path": str(path),
        "calibration_format_version": raw.get("version") if isinstance(raw, dict) else None,
        "motor_names": motor_names,
        "motors": {},
        "expected_arm_joint_names": list(ARM_JOINT_NAMES),
        "matches_expected_arm_joint_names": all(name in raw for name in ARM_JOINT_NAMES),
        "contains_gripper": OPTIONAL_GRIPPER_NAME in raw,
        "robot_id_basis": (
            "LeRobot SOFollower Robot.calibration_fpath resolves id=my_follower "
            "to this my_follower.json path when calibration_dir is default."
        ),
    }
    for name, item in raw.items():
        if not isinstance(item, dict):
            continue
        summary["motors"][name] = {
            "id": int(item["id"]),
            "homing_offset": int(item["homing_offset"]),
            "range_min": int(item["range_min"]),
            "range_max": int(item["range_max"]),
            "drive_mode": int(item["drive_mode"]),
        }
    ensure_report_dir()
    CALIBRATION_SUMMARY.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False),
        encoding="utf-8",
    )
    return summary


def motor_config(config: ReadOnlyConfig, include_gripper: bool = True) -> dict[str, Any]:
    _, Motor, _, MotorNormMode = import_lerobot_bus_types()
    motors = {
        name: Motor(index + 1, config.motor_model, MotorNormMode.DEGREES)
        for index, name in enumerate(ARM_JOINT_NAMES)
    }
    if include_gripper:
        motors[OPTIONAL_GRIPPER_NAME] = Motor(
            6,
            config.motor_model,
            MotorNormMode.RANGE_0_100,
        )
    return motors


def make_bus(config: ReadOnlyConfig, include_gripper: bool = True) -> Any:
    FeetechMotorsBus, _, _, _ = import_lerobot_bus_types()
    calibration = load_lerobot_calibration(config.calibration_path)
    motors = motor_config(config, include_gripper=include_gripper)
    return FeetechMotorsBus(
        port=config.follower_port,
        motors=motors,
        calibration={name: calibration[name] for name in motors if name in calibration},
        protocol_version=config.protocol_version,
    )


def exception_reason(error: BaseException) -> str:
    text = repr(error)
    lowered = text.lower()
    if "no status packet" in lowered:
        return "no_status_packet"
    if "could not connect" in lowered or "failed to open port" in lowered:
        return "serial_port_open_failed"
    if "access is denied" in lowered or "permission" in lowered:
        return "serial_port_busy_or_permission_denied"
    if "timeout" in lowered:
        return "read_timeout"
    return "motor_bus_error"


def base_report(config: ReadOnlyConfig) -> dict[str, Any]:
    ports = serial_ports()
    configured = find_port_info(ports, config.follower_port)
    motors = motor_config(config, include_gripper=True)
    required_ids = [motors[name].id for name in ARM_JOINT_NAMES]
    optional_ids = [motors[OPTIONAL_GRIPPER_NAME].id]
    calibration = load_json(config.calibration_path) if config.calibration_path.is_file() else {}
    return {
        "timestamp": iso_timestamp(),
        "current_serial_ports": ports,
        "configured_follower_port": config.follower_port,
        "configured_port_exists": configured is not None,
        "selected_port_basis": (
            "Configured follower_port from real_joint_state_mapping.json; "
            "motor identity is confirmed only if read-only preflight passes."
        ),
        "selected_port_serial_number": (
            configured.get("serial_number") if configured else None
        ),
        "port_classification": classify_serial_ports(ports, config.follower_port),
        "calibration_path": str(config.calibration_path),
        "calibration_exists": config.calibration_path.is_file(),
        "bus_class": "lerobot.motors.feetech.feetech.FeetechMotorsBus",
        "connect_method": "FeetechMotorsBus.connect(handshake=False)",
        "disconnect_method": "FeetechMotorsBus.disconnect(disable_torque=False)",
        "motor_model": config.motor_model,
        "baudrate": config.baudrate,
        "protocol": config.protocol_version,
        "required_motor_ids": required_ids,
        "optional_motor_ids": optional_ids,
        "joint_source_names": list(ARM_JOINT_NAMES),
        "includes_gripper_id_6": OPTIONAL_GRIPPER_NAME in motors,
        "calibration_motor_ids": {
            name: int(item["id"])
            for name, item in calibration.items()
            if isinstance(item, dict) and "id" in item
        },
        "sync_read_parameters": {
            "data_name": "Present_Position",
            "motors": list(ARM_JOINT_NAMES),
            "normalize": True,
            "num_retry": config.read_retry_count,
        },
        "responsive_motor_ids": [],
        "per_motor_ping_results": {},
        "per_motor_present_position_results": {},
        "bulk_sync_read_result": {
            "passed": False,
            "raw_values": None,
            "calibrated_values": None,
            "error": None,
        },
        "failure_reason": None,
        "suggested_checks": [],
        "torque_enable_written": False,
        "torque_disable_written": False,
        "goal_position_written": False,
        "motion_parameters_written": False,
        "motion_command_sent": False,
        "observed_physical_motion": False,
        "opened_com_ports": False,
        "final_status": "READ_ONLY_PREFLIGHT_NOT_RUN",
    }


def suggested_checks(reason: str, responsive_ids: list[int]) -> list[str]:
    if not responsive_ids:
        return [
            "follower_bus_power",
            "wrong_serial_port",
            "baudrate_or_protocol_mismatch",
            "motor_id_mismatch",
        ]
    if reason == "missing_required_motor_id":
        return ["motor_id_mismatch", "daisy_chain_connection", "follower_bus_power"]
    return ["follower_bus_power", "wrong_serial_port", "baudrate_or_protocol_mismatch"]


def preflight(config: ReadOnlyConfig, allow_unlisted_port: bool) -> tuple[int, dict[str, Any]]:
    if PREFLIGHT_LOG.exists():
        PREFLIGHT_LOG.unlink()
    report = base_report(config)
    log_json({"status": "CONFIG_LOADED", "follower_port": config.follower_port})

    if not report["configured_port_exists"] and not allow_unlisted_port:
        report["failure_reason"] = "configured_port_missing"
        report["suggested_checks"] = ["wrong_serial_port", "device_not_connected"]
        report["final_status"] = "READ_ONLY_PREFLIGHT_FAIL"
        write_report(report)
        log_json(preflight_failure_payload(report))
        return 3, report
    log_json({"status": "SERIAL_PORT_VERIFIED", "follower_port": config.follower_port})

    bus = make_bus(config, include_gripper=True)
    try:
        bus.connect(handshake=False)
        bus.set_baudrate(config.baudrate)
        report["opened_com_ports"] = True
    except Exception as error:
        report["failure_reason"] = exception_reason(error)
        report["suggested_checks"] = suggested_checks(report["failure_reason"], [])
        report["final_status"] = "READ_ONLY_PREFLIGHT_FAIL"
        write_report(report)
        log_json(preflight_failure_payload(report))
        return 4, report

    try:
        responsive_ids: list[int] = []
        for name, motor in bus.motors.items():
            motor_id = int(motor.id)
            ping_result = {
                "motor_name": name,
                "motor_id": motor_id,
                "ping_passed": False,
                "model_number": None,
                "error": None,
            }
            try:
                model_number = bus.ping(motor_id, num_retry=config.read_retry_count)
                ping_result["model_number"] = model_number
                ping_result["ping_passed"] = model_number is not None
                if model_number is not None:
                    responsive_ids.append(motor_id)
            except Exception as error:
                ping_result["error"] = repr(error)
            report["per_motor_ping_results"][str(motor_id)] = ping_result

        report["responsive_motor_ids"] = responsive_ids
        required_ids = set(report["required_motor_ids"])
        if required_ids.issubset(set(responsive_ids)):
            log_json(
                {
                    "status": "MOTOR_PING_PASS",
                    "responsive_motor_ids": responsive_ids,
                    "required_motor_ids": report["required_motor_ids"],
                }
            )

        for name, motor in bus.motors.items():
            motor_id = int(motor.id)
            if motor_id not in responsive_ids:
                report["per_motor_present_position_results"][str(motor_id)] = {
                    "motor_name": name,
                    "motor_id": motor_id,
                    "present_position_read_passed": False,
                    "raw_value": None,
                    "calibrated_value": None,
                    "error": "ping_failed_or_no_response",
                }
                continue
            result = {
                "motor_name": name,
                "motor_id": motor_id,
                "present_position_read_passed": False,
                "raw_value": None,
                "calibrated_value": None,
                "error": None,
            }
            try:
                result["raw_value"] = bus.read(
                    "Present_Position",
                    name,
                    normalize=False,
                    num_retry=config.read_retry_count,
                )
                result["calibrated_value"] = bus.read(
                    "Present_Position",
                    name,
                    normalize=True,
                    num_retry=config.read_retry_count,
                )
                result["present_position_read_passed"] = bool(
                    math.isfinite(float(result["calibrated_value"]))
                )
            except Exception as error:
                result["error"] = repr(error)
            report["per_motor_present_position_results"][str(motor_id)] = result

        present_required_ok = all(
            report["per_motor_present_position_results"]
            .get(str(motor_id), {})
            .get("present_position_read_passed")
            for motor_id in report["required_motor_ids"]
        )
        if present_required_ok:
            log_json({"status": "PRESENT_POSITION_READ_PASS"})

        try:
            raw_values = bus.sync_read(
                "Present_Position",
                ARM_JOINT_NAMES,
                normalize=False,
                num_retry=config.read_retry_count,
            )
            calibrated_values = bus.sync_read(
                "Present_Position",
                ARM_JOINT_NAMES,
                normalize=True,
                num_retry=config.read_retry_count,
            )
            report["bulk_sync_read_result"] = {
                "passed": True,
                "raw_values": {name: float(value) for name, value in raw_values.items()},
                "calibrated_values": {
                    name: float(value) for name, value in calibrated_values.items()
                },
                "error": None,
            }
        except Exception as error:
            report["bulk_sync_read_result"] = {
                "passed": False,
                "raw_values": None,
                "calibrated_values": None,
                "error": repr(error),
            }

        missing_ids = sorted(required_ids - set(responsive_ids))
        if missing_ids and not responsive_ids:
            report["failure_reason"] = "all_required_motor_ids_unresponsive"
        elif missing_ids:
            report["failure_reason"] = "missing_required_motor_id"
        elif not present_required_ok:
            report["failure_reason"] = "present_position_read_failed"
        elif not report["bulk_sync_read_result"]["passed"]:
            report["failure_reason"] = exception_reason(
                RuntimeError(str(report["bulk_sync_read_result"]["error"]))
            )
        else:
            report["final_status"] = "READ_ONLY_PREFLIGHT_PASS"
            write_report(report)
            log_json(
                {
                    "status": "READ_ONLY_PREFLIGHT_PASS",
                    "follower_port": config.follower_port,
                    "bus_class": report["bus_class"],
                    "baudrate": config.baudrate,
                    "protocol": config.protocol_version,
                    "responsive_motor_ids": responsive_ids,
                    "required_motor_ids": report["required_motor_ids"],
                    "present_positions_read": True,
                    "torque_enable_written": False,
                    "goal_position_written": False,
                    "motion_command_sent": False,
                }
            )
            return 0, report

        report["suggested_checks"] = suggested_checks(
            str(report["failure_reason"]),
            responsive_ids,
        )
        report["final_status"] = "READ_ONLY_PREFLIGHT_FAIL"
        write_report(report)
        log_json(preflight_failure_payload(report))
        return 5, report
    finally:
        if getattr(bus, "is_connected", False):
            bus.disconnect(disable_torque=False)
        log_json({"status": "SHUTDOWN", "disconnect_disables_torque": False})


def preflight_failure_payload(report: dict[str, Any]) -> dict[str, Any]:
    return {
        "status": "READ_ONLY_PREFLIGHT_FAIL",
        "reason": report["failure_reason"],
        "follower_port": report["configured_follower_port"],
        "responsive_motor_ids": report["responsive_motor_ids"],
        "required_motor_ids": report["required_motor_ids"],
        "suggested_checks": report["suggested_checks"],
        "torque_enable_written": False,
        "goal_position_written": False,
        "motion_command_sent": False,
    }


def status_payload(config: ReadOnlyConfig, status: str, **values: Any) -> dict[str, Any]:
    payload = {
        "type": "joint_state_status",
        "status": status,
        "timestamp_monotonic_s": time.monotonic(),
        "follower_port": config.follower_port,
        "read_only": True,
        "torque_enable_written": False,
        "torque_disable_written": False,
        "goal_position_written": False,
        "motion_parameters_written": False,
        "motion_command_sent": False,
    }
    payload.update(values)
    return payload


def read_joint_frame(bus: Any, config: ReadOnlyConfig) -> dict[str, Any]:
    raw_values = bus.sync_read(
        "Present_Position",
        ARM_JOINT_NAMES,
        normalize=False,
        num_retry=config.read_retry_count,
    )
    calibrated_degrees = bus.sync_read(
        "Present_Position",
        ARM_JOINT_NAMES,
        normalize=True,
        num_retry=config.read_retry_count,
    )
    diagnostics = build_mapping_diagnostics(
        raw_values=raw_values,
        calibrated_degrees=calibrated_degrees,
    )
    return {
        "motor_bus_read_valid": True,
        "read_register": "Present_Position",
        **diagnostics,
    }


def make_joint_state_payload(
    config: ReadOnlyConfig,
    sequence: int,
    frame: dict[str, Any],
) -> dict[str, Any]:
    return {
        "type": "joint_state",
        "sequence": sequence,
        "timestamp_monotonic_s": time.monotonic(),
        "joint_names": list(ARM_JOINT_NAMES),
        "source": "real_so101_follower_read_only",
        "read_only": True,
        "torque_state_changed": False,
        "torque_enable_written": False,
        "torque_disable_written": False,
        "goal_position_written": False,
        "motion_parameters_written": False,
        "motion_command_sent": False,
        "follower_port": config.follower_port,
        "calibration_path": str(config.calibration_path),
        "mapping_status": config.mapping_status,
        "motor_read_requests_sent": sequence + 1,
        **frame,
    }


def write_joint_mapping_report(
    *,
    config: ReadOnlyConfig,
    final_status: str,
    root_cause: str,
    frame: dict[str, Any] | None,
    preflight_report: dict[str, Any] | None,
    mapping_modified: bool,
) -> dict[str, Any]:
    urdf_sha = sha256_file(URDF_PATH)
    report = {
        "timestamp": iso_timestamp(),
        "final_status": final_status,
        "root_cause": root_cause,
        "tcp_connected": None,
        "motor_bus_streaming": bool(frame and frame.get("motor_bus_read_valid")),
        "calibration_applied": bool(frame and frame.get("calibration_applied")),
        "calibration_path": str(config.calibration_path),
        "raw_encoder_values": None if frame is None else frame.get("raw_encoder_values"),
        "motor_degrees_before_calibration": (
            None if frame is None else frame.get("motor_degrees_before_calibration")
        ),
        "calibrated_lerobot_values": (
            None if frame is None else frame.get("lerobot_calibrated_degrees")
        ),
        "source_unit": None if frame is None else frame.get("source_unit"),
        "mapped_urdf_rad": None if frame is None else frame.get("mapped_positions_rad"),
        "per_joint_validation": (
            None if frame is None else frame.get("per_joint_validation")
        ),
        "violating_joints": [] if frame is None else frame.get("violating_joints", []),
        "sign_offset_scale": load_joint_mapping(),
        "mapping_modified": mapping_modified,
        "mapping_modification_basis": (
            "No mapping change was made; diagnostics do not by themselves prove "
            "URDF zero/direction mismatch versus current physical pose."
        ),
        "final_real_joint_state_valid": bool(
            frame and frame.get("mapped_joint_state_valid")
        ),
        "preflight": preflight_report,
        "safety": {
            "read_only": True,
            "torque_state_changed": False,
            "torque_enable_written": False,
            "goal_position_written": False,
            "motion_parameters_written": False,
            "motion_command_sent": False,
            "shadow_execution_started": False,
            "controller_command_topics_published": [],
        },
        "urdf": {
            "path": str(URDF_PATH),
            "sha256": urdf_sha,
            "expected_sha256": EXPECTED_URDF_SHA256,
            "matches_expected": urdf_sha == EXPECTED_URDF_SHA256,
        },
    }
    ensure_report_dir()
    MAPPING_REPORT.write_text(
        json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False),
        encoding="utf-8",
    )
    return report


def dump_once(config: ReadOnlyConfig, allow_unlisted_port: bool) -> int:
    if MAPPING_LOG.exists():
        MAPPING_LOG.unlink()
    preflight_code, preflight_report = preflight(config, allow_unlisted_port)
    write_calibration_summary(config.calibration_path)
    if preflight_code != 0:
        report = write_joint_mapping_report(
            config=config,
            final_status="FAIL_PREFLIGHT",
            root_cause=str(preflight_report.get("failure_reason")),
            frame=None,
            preflight_report=preflight_report,
            mapping_modified=False,
        )
        MAPPING_DUMP.write_text(
            json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False),
            encoding="utf-8",
        )
        log_mapping(
            {
                "status": "DUMP_ONCE_FAIL",
                "reason": preflight_report.get("failure_reason"),
                "report": str(MAPPING_REPORT),
            }
        )
        return preflight_code

    bus = make_bus(config, include_gripper=True)
    frame: dict[str, Any] | None = None
    try:
        bus.connect(handshake=False)
        bus.set_baudrate(config.baudrate)
        frame = read_joint_frame(bus, config)
        payload = make_joint_state_payload(config, 0, frame)
        dump = {
            "status": "DUMP_ONCE_PASS",
            "timestamp": iso_timestamp(),
            "payload": payload,
            "torque_enable_written": False,
            "goal_position_written": False,
            "motion_parameters_written": False,
            "motion_command_sent": False,
        }
        MAPPING_DUMP.write_text(
            json.dumps(dump, indent=2, ensure_ascii=False, allow_nan=False),
            encoding="utf-8",
        )
        root_cause = (
            "mapped_joint_state_valid"
            if frame.get("mapped_joint_state_valid")
            else "current_joint_state_out_of_bounds"
        )
        write_joint_mapping_report(
            config=config,
            final_status="PASS" if frame.get("mapped_joint_state_valid") else "FAIL",
            root_cause=root_cause,
            frame=frame,
            preflight_report=preflight_report,
            mapping_modified=False,
        )
        log_mapping(
            {
                "status": "DUMP_ONCE_PASS",
                "mapped_joint_state_valid": frame.get("mapped_joint_state_valid"),
                "violating_joints": frame.get("violating_joints"),
                "dump": str(MAPPING_DUMP),
                "report": str(MAPPING_REPORT),
            }
        )
        return 0 if frame.get("mapped_joint_state_valid") else 7
    finally:
        if getattr(bus, "is_connected", False):
            bus.disconnect(disable_torque=False)


def send_json_line(client: socket.socket, payload: dict[str, Any]) -> None:
    client.sendall(json.dumps(payload, ensure_ascii=False, allow_nan=False).encode("utf-8"))
    client.sendall(b"\n")


def stream_client(
    client: socket.socket,
    config: ReadOnlyConfig,
    sequence_start: int,
) -> tuple[int, int, str | None]:
    bus = make_bus(config, include_gripper=True)
    sequence = sequence_start
    period_s = 1.0 / config.rate_hz
    consecutive_failures = 0
    try:
        bus.connect(handshake=False)
        bus.set_baudrate(config.baudrate)
        streaming_logged = False
        while True:
            try:
                frame = read_joint_frame(bus, config)
                consecutive_failures = 0
            except Exception as error:
                consecutive_failures += 1
                reason = exception_reason(error)
                send_json_line(
                    client,
                    status_payload(
                        config,
                        "INVALID",
                        reason="motor_bus_read_failed",
                        error=repr(error),
                        classified_reason=reason,
                        consecutive_read_failures=consecutive_failures,
                    ),
                )
                log_json(
                    {
                        "status": "MOTOR_BUS_READ_RETRY",
                        "reason": reason,
                        "error": repr(error),
                        "consecutive_read_failures": consecutive_failures,
                    }
                )
                if consecutive_failures >= config.maximum_consecutive_read_failures:
                    send_json_line(
                        client,
                        status_payload(
                            config,
                            "INVALID",
                            reason="motor_bus_fault",
                            classified_reason=reason,
                            consecutive_read_failures=consecutive_failures,
                        ),
                    )
                    log_json(
                        {
                            "status": "MOTOR_BUS_FAULT",
                            "reason": reason,
                            "consecutive_read_failures": consecutive_failures,
                        }
                    )
                    return sequence, consecutive_failures, reason
                time.sleep(config.reconnect_backoff_s)
                continue

            positions_rad = frame["positions_rad"]
            if not all(math.isfinite(float(value)) for value in positions_rad):
                send_json_line(
                    client,
                    status_payload(
                        config,
                        "INVALID",
                        reason="non_finite_joint_state",
                    ),
                )
                return sequence, consecutive_failures, "non_finite_joint_state"
            payload = make_joint_state_payload(config, sequence, frame)
            send_json_line(client, payload)
            if not streaming_logged:
                log_json(
                    {
                        "status": "READ_ONLY_STREAMING",
                        "motor_bus_read_valid": True,
                        "mapped_joint_state_valid": frame["mapped_joint_state_valid"],
                        "mapping_failure_reason": frame["mapping_failure_reason"],
                        "violating_joints": frame["violating_joints"],
                    }
                )
                streaming_logged = True
            sequence += 1
            elapsed = time.monotonic() - float(payload["timestamp_monotonic_s"])
            time.sleep(max(0.0, period_s - elapsed))
    finally:
        if getattr(bus, "is_connected", False):
            bus.disconnect(disable_torque=False)


def run_server(config: ReadOnlyConfig, allow_unlisted_port: bool) -> int:
    code, report = preflight(config, allow_unlisted_port)
    if code != 0:
        return code
    log_json(
        {
            "status": "LISTENING_READ_ONLY",
            "bind": f"{config.bind_host}:{config.bind_port}",
            "preflight_report": str(PREFLIGHT_REPORT),
        }
    )
    sequence = 0
    consecutive_faults = 0
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((config.bind_host, config.bind_port))
        server.listen(1)
        while True:
            try:
                client, address = server.accept()
            except KeyboardInterrupt:
                log_json({"status": "SHUTDOWN"})
                return 0
            with client:
                log_json({"status": "CLIENT_CONNECTED", "client": str(address)})
                try:
                    sequence, failures, reason = stream_client(client, config, sequence)
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    failures = 0
                    reason = None
                log_json({"status": "CLIENT_DISCONNECTED", "reason": reason})
                if failures:
                    consecutive_faults += 1
                    if config.fault_policy == "exit_nonzero":
                        return 6
                    if consecutive_faults >= config.maximum_consecutive_read_failures:
                        log_json(
                            {
                                "status": "MOTOR_BUS_FAULT",
                                "reason": reason,
                                "fault_policy": config.fault_policy,
                            }
                        )
                        time.sleep(config.reconnect_backoff_s)


def main() -> int:
    args = parse_args()
    config = build_config(args)
    if args.list_ports:
        return print_list_ports(config)
    if args.preflight_only:
        code, _ = preflight(config, bool(args.allow_unlisted_port))
        return code
    if args.dump_once:
        return dump_once(config, bool(args.allow_unlisted_port))
    return run_server(config, bool(args.allow_unlisted_port))


if __name__ == "__main__":
    raise SystemExit(main())
