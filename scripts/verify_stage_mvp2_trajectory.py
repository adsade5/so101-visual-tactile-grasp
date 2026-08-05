from __future__ import annotations

import csv
import hashlib
import json
import math
import struct
import sys
import zlib
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
ROS2_SRC = PROJECT_ROOT / "ros2_ws" / "src"
sys.path.insert(0, str(ROS2_SRC / "so101_mvp_kinematics"))
sys.path.insert(0, str(ROS2_SRC / "so101_mvp_control"))

from so101_mvp_control.simple_trajectory import (
    TrajectoryPoint,
    generate_sequential_joint_trajectory,
    generate_single_joint_profile,
)
from so101_mvp_control.trajectory_validation import validate_trajectory
from so101_mvp_kinematics.fk import forward_kinematics
from so101_mvp_kinematics.ik import solve_ik
from so101_mvp_kinematics.joint_limits import joints_within_limits
from so101_mvp_kinematics.model import BASE_LINK, JOINT_NAMES, TIP_LINK, So101KinematicModel


URDF_PATH = PROJECT_ROOT / "data" / "robot_model" / "so101" / "so101_new_calib.urdf"
REPORT_PATH = PROJECT_ROOT / "data" / "verification" / "stage_mvp2_trajectory_report.json"
LOG_PATH = PROJECT_ROOT / "data" / "verification" / "stage_mvp2_trajectory.log"
OUTPUT_DIR = PROJECT_ROOT / "data" / "verification" / "mvp2_trajectory"
EXPECTED_URDF_SHA256 = "3a65d2d35e68a8d2f0c2cc176d19b884506543c93BA72980145B80ABE276022C".lower()

CONTROL_RATE_HZ = 20.0
FIXED_SPEED = 0.08
MAX_SPEED = 0.10
FIXED_ACCELERATION = 0.20
MAX_ACCELERATION = 0.25
MINIMUM_MOTION = 0.001
SETTLE_BETWEEN_JOINTS = 0.20
SETTLE_BETWEEN_STAGES = 0.50
MAX_TOTAL_DURATION = 120.0
JOINT_ORDER = JOINT_NAMES.copy()

REFERENCE_Q = np.asarray([0.0, -0.35, 0.35, 1.22, 0.0], dtype=np.float64)
DESIRED_APPROACH = np.asarray([0.0, 0.0, -1.0], dtype=np.float64)
TOOL_AXIS = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
OBJECT_POSITION = np.asarray([0.18, -0.04, 0.025], dtype=np.float64)
PREGRASP_TARGET = np.asarray([0.18, -0.04, 0.080], dtype=np.float64)
DESCEND_TARGET = np.asarray([0.18, -0.04, 0.055], dtype=np.float64)
LIFT_TARGET = np.asarray([0.18, -0.04, 0.080], dtype=np.float64)
PLACE_TARGET = np.asarray([0.20, -0.08, 0.080], dtype=np.float64)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git_branch() -> str:
    import subprocess

    return subprocess.check_output(
        ["git", "branch", "--show-current"],
        cwd=PROJECT_ROOT,
        text=True,
    ).strip()


def listf(values: np.ndarray | list[float]) -> list[float]:
    return [float(value) for value in np.asarray(values, dtype=np.float64).tolist()]


def build_trajectory_dict(
    points: list[TrajectoryPoint],
    start: np.ndarray,
    target: np.ndarray,
    segments: list[dict[str, object]],
) -> dict[str, object]:
    return {
        "joint_names": JOINT_NAMES.copy(),
        "joint_order": JOINT_ORDER.copy(),
        "start_positions_rad": listf(start),
        "target_positions_rad": listf(target),
        "points": points,
        "segments": segments,
        "motion_stage": "full_pick_place_offline",
    }


def add_static_pause(
    points: list[TrajectoryPoint],
    duration_s: float,
    motion_stage: str,
) -> None:
    if duration_s <= 0.0:
        return
    dt = 1.0 / CONTROL_RATE_HZ
    start_time = points[-1].time_s
    positions = points[-1].positions_rad.copy()
    zero = [0.0 for _ in JOINT_NAMES]
    steps = int(math.floor(duration_s / dt))
    times = [round(index * dt, 12) for index in range(1, steps + 1)]
    if not times or not math.isclose(times[-1], duration_s, abs_tol=1.0e-12):
        times.append(duration_s)
    for local_time in times:
        points.append(
            TrajectoryPoint(
                time_s=float(start_time + local_time),
                positions_rad=positions.copy(),
                velocities_rad_s=zero.copy(),
                accelerations_rad_s2=zero.copy(),
                active_joint_index=None,
                active_joint_name=None,
                phase="stage_settle",
                motion_stage=motion_stage,
            )
        )


def append_segment(
    full_points: list[TrajectoryPoint],
    full_segments: list[dict[str, object]],
    trajectory: dict[str, object],
) -> None:
    points = list(trajectory["points"])
    if not full_points:
        full_points.extend(points)
    else:
        full_points.extend(points[1:])
    full_segments.extend(list(trajectory["segments"]))


def solve_demo_ik(model: So101KinematicModel) -> tuple[list[dict[str, object]], dict[str, np.ndarray]]:
    targets = [
        ("pregrasp", PREGRASP_TARGET),
        ("descend", DESCEND_TARGET),
        ("lift", LIFT_TARGET),
        ("place", PLACE_TARGET),
    ]
    seed = REFERENCE_Q.copy()
    results: list[dict[str, object]] = []
    solutions: dict[str, np.ndarray] = {}
    for name, target in targets:
        result = solve_ik(
            model,
            target,
            seed,
            desired_approach_base=DESIRED_APPROACH,
            tool_approach_axis_local=TOOL_AXIS,
            max_iterations=200,
            damping=0.05,
            maximum_step_rad=0.10,
            position_tolerance_m=0.002,
            approach_tolerance_deg=5.0,
            orientation_weight=0.25,
        )
        success = bool(result.get("success"))
        if success:
            seed = np.asarray(result["joint_positions_rad"], dtype=np.float64)
            solutions[name] = seed.copy()
        results.append(
            {
                "name": name,
                "target_position_m": listf(target),
                **result,
            }
        )
    return results, solutions


def generate_demo_trajectory(
    model: So101KinematicModel,
    solutions: dict[str, np.ndarray],
) -> tuple[dict[str, object], list[dict[str, object]]]:
    stages = [
        ("reference_to_pregrasp", REFERENCE_Q, solutions["pregrasp"]),
        ("pregrasp_to_descend", solutions["pregrasp"], solutions["descend"]),
        ("descend_to_lift", solutions["descend"], solutions["lift"]),
        ("lift_to_place", solutions["lift"], solutions["place"]),
    ]
    full_points: list[TrajectoryPoint] = []
    full_segments: list[dict[str, object]] = []
    segment_results: list[dict[str, object]] = []
    start_time = 0.0

    for index, (name, start, target) in enumerate(stages):
        trajectory = generate_sequential_joint_trajectory(
            start,
            target,
            JOINT_NAMES,
            JOINT_ORDER,
            FIXED_SPEED,
            FIXED_ACCELERATION,
            CONTROL_RATE_HZ,
            MINIMUM_MOTION,
            SETTLE_BETWEEN_JOINTS,
            motion_stage=name,
            start_time_s=start_time,
        )
        validation = validate_trajectory(
            trajectory,
            model,
            MAX_SPEED,
            MAX_ACCELERATION,
            MAX_TOTAL_DURATION,
        )
        append_segment(full_points, full_segments, trajectory)
        segment_results.append(
            {
                "name": name,
                "point_count": len(trajectory["points"]),
                "duration_s": float(trajectory["points"][-1].time_s - trajectory["points"][0].time_s),
                "validation": validation,
                "joint_segments": trajectory["segments"],
            }
        )
        start_time = full_points[-1].time_s
        if index < len(stages) - 1:
            add_static_pause(full_points, SETTLE_BETWEEN_STAGES, f"{name}_to_next")
            start_time = full_points[-1].time_s

    full_trajectory = build_trajectory_dict(
        full_points,
        REFERENCE_Q,
        solutions["place"],
        full_segments,
    )
    return full_trajectory, segment_results


def write_trajectory_csv(path: Path, points: list[TrajectoryPoint]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    header = ["time_s"]
    header += [f"{name}_position_rad" for name in JOINT_NAMES]
    header += [f"{name}_velocity_rad_s" for name in JOINT_NAMES]
    header += [f"{name}_acceleration_rad_s2" for name in JOINT_NAMES]
    header += ["active_joint_name", "phase", "motion_stage"]
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(header)
        for point in points:
            writer.writerow(
                [f"{point.time_s:.9f}"]
                + [f"{value:.12f}" for value in point.positions_rad]
                + [f"{value:.12f}" for value in point.velocities_rad_s]
                + [f"{value:.12f}" for value in point.accelerations_rad_s2]
                + [point.active_joint_name or "", point.phase, point.motion_stage]
            )


def write_tcp_path_csv(path: Path, model: So101KinematicModel, points: list[TrajectoryPoint]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(["time_s", "tcp_x_m", "tcp_y_m", "tcp_z_m", "motion_stage", "active_joint_name"])
        for point in points:
            fk = forward_kinematics(model, np.asarray(point.positions_rad, dtype=np.float64))
            position = np.asarray(fk["position_m"], dtype=np.float64)
            row = {
                "time_s": point.time_s,
                "tcp_x_m": float(position[0]),
                "tcp_y_m": float(position[1]),
                "tcp_z_m": float(position[2]),
                "motion_stage": point.motion_stage,
                "active_joint_name": point.active_joint_name or "",
            }
            rows.append(row)
            writer.writerow(
                [
                    f"{point.time_s:.9f}",
                    f"{position[0]:.12f}",
                    f"{position[1]:.12f}",
                    f"{position[2]:.12f}",
                    point.motion_stage,
                    point.active_joint_name or "",
                ]
            )
    return rows


def write_plots(points: list[TrajectoryPoint], tcp_rows: list[dict[str, object]]) -> dict[str, str]:
    times = np.asarray([point.time_s for point in points], dtype=np.float64)
    positions = np.asarray([point.positions_rad for point in points], dtype=np.float64)
    velocities = np.asarray([point.velocities_rad_s for point in points], dtype=np.float64)
    accelerations = np.asarray([point.accelerations_rad_s2 for point in points], dtype=np.float64)
    tcp_times = np.asarray([row["time_s"] for row in tcp_rows], dtype=np.float64)
    tcp_xyz = np.asarray(
        [[row["tcp_x_m"], row["tcp_y_m"], row["tcp_z_m"]] for row in tcp_rows],
        dtype=np.float64,
    )

    paths = {
        "joint_positions": OUTPUT_DIR / "mvp2_joint_positions.png",
        "joint_velocities": OUTPUT_DIR / "mvp2_joint_velocities.png",
        "joint_accelerations": OUTPUT_DIR / "mvp2_joint_accelerations.png",
        "tcp_path": OUTPUT_DIR / "mvp2_tcp_path.png",
    }

    _plot_series_png(times, positions, paths["joint_positions"])
    _plot_series_png(times, velocities, paths["joint_velocities"])
    _plot_series_png(times, accelerations, paths["joint_accelerations"])
    _plot_series_png(tcp_times, tcp_xyz, paths["tcp_path"])

    return {key: str(value) for key, value in paths.items()}


def _write_png(path: Path, pixels: bytearray, width: int, height: int) -> None:
    def chunk(name: bytes, data: bytes) -> bytes:
        return (
            struct.pack(">I", len(data))
            + name
            + data
            + struct.pack(">I", zlib.crc32(name + data) & 0xFFFFFFFF)
        )

    rows = bytearray()
    stride = width * 3
    for y in range(height):
        rows.append(0)
        rows.extend(pixels[y * stride : (y + 1) * stride])
    payload = zlib.compress(bytes(rows), level=9)
    png = (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", payload)
        + chunk(b"IEND", b"")
    )
    path.write_bytes(png)


def _put_pixel(pixels: bytearray, width: int, height: int, x: int, y: int, color: tuple[int, int, int]) -> None:
    if x < 0 or x >= width or y < 0 or y >= height:
        return
    index = (y * width + x) * 3
    pixels[index : index + 3] = bytes(color)


def _draw_line(
    pixels: bytearray,
    width: int,
    height: int,
    x0: int,
    y0: int,
    x1: int,
    y1: int,
    color: tuple[int, int, int],
) -> None:
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    while True:
        _put_pixel(pixels, width, height, x0, y0, color)
        _put_pixel(pixels, width, height, x0 + 1, y0, color)
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy


def _plot_series_png(times: np.ndarray, values: np.ndarray, path: Path) -> None:
    width = 1200
    height = 600
    left = 70
    right = 30
    top = 30
    bottom = 55
    pixels = bytearray([255] * width * height * 3)
    colors = [
        (31, 119, 180),
        (214, 39, 40),
        (44, 160, 44),
        (255, 127, 14),
        (148, 103, 189),
    ]

    xmin = float(np.min(times))
    xmax = float(np.max(times))
    ymin = float(np.min(values))
    ymax = float(np.max(values))
    if math.isclose(xmin, xmax):
        xmax = xmin + 1.0
    if math.isclose(ymin, ymax):
        ymin -= 1.0
        ymax += 1.0
    margin = 0.05 * (ymax - ymin)
    ymin -= margin
    ymax += margin

    plot_w = width - left - right
    plot_h = height - top - bottom

    for i in range(6):
        x = left + int(round(plot_w * i / 5.0))
        _draw_line(pixels, width, height, x, top, x, top + plot_h, (225, 225, 225))
    for i in range(6):
        y = top + int(round(plot_h * i / 5.0))
        _draw_line(pixels, width, height, left, y, left + plot_w, y, (225, 225, 225))
    _draw_line(pixels, width, height, left, top, left, top + plot_h, (0, 0, 0))
    _draw_line(pixels, width, height, left, top + plot_h, left + plot_w, top + plot_h, (0, 0, 0))

    def map_x(value: float) -> int:
        return left + int(round((value - xmin) / (xmax - xmin) * plot_w))

    def map_y(value: float) -> int:
        return top + plot_h - int(round((value - ymin) / (ymax - ymin) * plot_h))

    columns = values.shape[1]
    for column in range(columns):
        color = colors[column % len(colors)]
        previous: tuple[int, int] | None = None
        for time_value, y_value in zip(times.tolist(), values[:, column].tolist()):
            point = (map_x(float(time_value)), map_y(float(y_value)))
            if previous is not None:
                _draw_line(pixels, width, height, previous[0], previous[1], point[0], point[1], color)
            previous = point
        legend_x = left + 20 + column * 70
        legend_y = height - 25
        _draw_line(pixels, width, height, legend_x, legend_y, legend_x + 40, legend_y, color)

    _write_png(path, pixels, width, height)


def monotonic(samples: list[dict[str, object]], direction: float) -> bool:
    positions = [float(sample["position_rad"]) for sample in samples]
    if direction >= 0.0:
        return all(after >= before - 1.0e-12 for before, after in zip(positions, positions[1:]))
    return all(after <= before + 1.0e-12 for before, after in zip(positions, positions[1:]))


def run_tests(model: So101KinematicModel, full_trajectory: dict[str, object]) -> list[dict[str, object]]:
    tests: list[dict[str, object]] = []

    positive = generate_single_joint_profile(0.0, 0.20, FIXED_SPEED, FIXED_ACCELERATION, CONTROL_RATE_HZ, MINIMUM_MOTION)
    tests.append(
        {
            "name": "positive_single_joint",
            "success": positive["profile_type"] == "trapezoidal"
            and monotonic(positive["samples"], 1.0)
            and float(positive["samples"][0]["velocity_rad_s"]) == 0.0
            and float(positive["samples"][-1]["velocity_rad_s"]) == 0.0,
            "profile_type": positive["profile_type"],
        }
    )

    negative = generate_single_joint_profile(0.10, -0.08, FIXED_SPEED, FIXED_ACCELERATION, CONTROL_RATE_HZ, MINIMUM_MOTION)
    tests.append(
        {
            "name": "negative_single_joint",
            "success": negative["profile_type"] == "trapezoidal" and monotonic(negative["samples"], -1.0),
            "profile_type": negative["profile_type"],
        }
    )

    short = generate_single_joint_profile(0.0, 0.01, FIXED_SPEED, FIXED_ACCELERATION, CONTROL_RATE_HZ, MINIMUM_MOTION)
    tests.append(
        {
            "name": "short_triangular_profile",
            "success": short["profile_type"] == "triangular" and monotonic(short["samples"], 1.0),
            "profile_type": short["profile_type"],
        }
    )

    zero = generate_single_joint_profile(0.0, 0.0, FIXED_SPEED, FIXED_ACCELERATION, CONTROL_RATE_HZ, MINIMUM_MOTION)
    tests.append(
        {
            "name": "zero_motion",
            "success": zero["profile_type"] == "static"
            and len(zero["samples"]) == 2
            and all(float(sample["velocity_rad_s"]) == 0.0 for sample in zero["samples"]),
            "profile_type": zero["profile_type"],
        }
    )

    validation = validate_trajectory(full_trajectory, model, MAX_SPEED, MAX_ACCELERATION, MAX_TOTAL_DURATION)
    tests.extend(
        [
            {
                "name": "sequential_only_one_joint_moves",
                "success": validation.get("maximum_simultaneously_moving_joints") <= 1,
                "observed": validation.get("maximum_simultaneously_moving_joints"),
            },
            {
                "name": "exact_endpoint",
                "success": bool(validation.get("exact_final_target")),
            },
            {
                "name": "speed_limit",
                "success": validation.get("maximum_velocity_rad_s_observed") <= MAX_SPEED,
                "observed": validation.get("maximum_velocity_rad_s_observed"),
            },
            {
                "name": "acceleration_limit",
                "success": validation.get("maximum_acceleration_rad_s2_observed") <= MAX_ACCELERATION,
                "observed": validation.get("maximum_acceleration_rad_s2_observed"),
            },
            {
                "name": "time_increasing",
                "success": bool(validation.get("time_strictly_increasing")),
            },
        ]
    )

    invalid_start = dict(full_trajectory)
    invalid_start["start_positions_rad"] = listf(model.upper_limits + 0.2)
    invalid_start["points"] = [
        TrajectoryPoint(0.0, invalid_start["start_positions_rad"], [0.0] * 5, [0.0] * 5, None, None, "start"),
        TrajectoryPoint(0.1, invalid_start["target_positions_rad"], [0.0] * 5, [0.0] * 5, None, None, "end"),
    ]
    result = validate_trajectory(invalid_start, model, MAX_SPEED, MAX_ACCELERATION, MAX_TOTAL_DURATION)
    tests.append({"name": "start_out_of_limits", "success": result["reason"] == "invalid_start_joint_limits", "reason": result["reason"]})

    invalid_target = dict(full_trajectory)
    invalid_target["target_positions_rad"] = listf(model.upper_limits + 0.2)
    invalid_target["points"] = [
        TrajectoryPoint(0.0, invalid_target["start_positions_rad"], [0.0] * 5, [0.0] * 5, None, None, "start"),
        TrajectoryPoint(0.1, invalid_target["target_positions_rad"], [0.0] * 5, [0.0] * 5, None, None, "end"),
    ]
    result = validate_trajectory(invalid_target, model, MAX_SPEED, MAX_ACCELERATION, MAX_TOTAL_DURATION)
    tests.append({"name": "target_out_of_limits", "success": result["reason"] == "invalid_target_joint_limits", "reason": result["reason"]})

    try:
        generate_sequential_joint_trajectory([math.nan] * 5, REFERENCE_Q, JOINT_NAMES, JOINT_ORDER, FIXED_SPEED, FIXED_ACCELERATION, CONTROL_RATE_HZ)
        non_finite_success = False
        non_finite_reason = "unexpected_success"
    except ValueError as exc:
        non_finite_success = True
        non_finite_reason = str(exc)
    tests.append({"name": "non_finite_input", "success": non_finite_success, "reason": non_finite_reason})

    repeat_a = generate_sequential_joint_trajectory(
        REFERENCE_Q,
        np.asarray(full_trajectory["target_positions_rad"], dtype=np.float64),
        JOINT_NAMES,
        JOINT_ORDER,
        FIXED_SPEED,
        FIXED_ACCELERATION,
        CONTROL_RATE_HZ,
        MINIMUM_MOTION,
        SETTLE_BETWEEN_JOINTS,
        "repeat",
    )
    repeat_b = generate_sequential_joint_trajectory(
        REFERENCE_Q,
        np.asarray(full_trajectory["target_positions_rad"], dtype=np.float64),
        JOINT_NAMES,
        JOINT_ORDER,
        FIXED_SPEED,
        FIXED_ACCELERATION,
        CONTROL_RATE_HZ,
        MINIMUM_MOTION,
        SETTLE_BETWEEN_JOINTS,
        "repeat",
    )
    deterministic = json.dumps(
        [[point.time_s, point.positions_rad, point.velocities_rad_s, point.accelerations_rad_s2, point.phase] for point in repeat_a["points"]],
        sort_keys=True,
    ) == json.dumps(
        [[point.time_s, point.positions_rad, point.velocities_rad_s, point.accelerations_rad_s2, point.phase] for point in repeat_b["points"]],
        sort_keys=True,
    )
    tests.append({"name": "deterministic_repeat", "success": deterministic})
    tests.append({"name": "full_pick_place_offline", "success": bool(validation["success"]), "reason": validation["reason"]})

    return tests


def main() -> int:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    urdf_hash = sha256_file(URDF_PATH)
    model = So101KinematicModel(URDF_PATH, BASE_LINK, TIP_LINK, JOINT_NAMES)

    ik_results, solutions = solve_demo_ik(model)
    ik_success = all(bool(result.get("success")) for result in ik_results)
    if ik_success:
        full_trajectory, segment_results = generate_demo_trajectory(model, solutions)
    else:
        full_trajectory = build_trajectory_dict([], REFERENCE_Q, REFERENCE_Q, [])
        segment_results = []

    validation = (
        validate_trajectory(full_trajectory, model, MAX_SPEED, MAX_ACCELERATION, MAX_TOTAL_DURATION)
        if ik_success
        else {"success": False, "reason": "ik_failure"}
    )

    csv_path = OUTPUT_DIR / "mvp2_full_trajectory.csv"
    tcp_csv_path = OUTPUT_DIR / "mvp2_tcp_path.csv"
    plot_paths: dict[str, str] = {}
    tcp_rows: list[dict[str, object]] = []
    if ik_success:
        write_trajectory_csv(csv_path, list(full_trajectory["points"]))
        tcp_rows = write_tcp_path_csv(tcp_csv_path, model, list(full_trajectory["points"]))
        plot_paths = write_plots(list(full_trajectory["points"]), tcp_rows)

    test_cases = run_tests(model, full_trajectory) if ik_success else []
    all_tests_success = all(bool(test["success"]) for test in test_cases)
    all_segments_success = all(bool(segment["validation"]["success"]) for segment in segment_results)
    all_points_within_limits = bool(validation.get("all_points_within_urdf_limits", False))
    exact_final_target = bool(validation.get("exact_final_target", False))
    final_status = "PASS"
    if not (
        urdf_hash == EXPECTED_URDF_SHA256
        and ik_success
        and bool(validation.get("success"))
        and all_tests_success
        and all_segments_success
        and all_points_within_limits
        and exact_final_target
    ):
        final_status = "FAIL"

    report = {
        "stage": "MVP-2",
        "git_branch": git_branch(),
        "urdf_sha256": urdf_hash,
        "joint_names": JOINT_NAMES,
        "motion_mode": "sequential_joint",
        "control_rate_hz": CONTROL_RATE_HZ,
        "fixed_speed_rad_s": FIXED_SPEED,
        "fixed_acceleration_rad_s2": FIXED_ACCELERATION,
        "joint_order": JOINT_ORDER,
        "reference_joint_positions_rad": listf(REFERENCE_Q),
        "pregrasp_target_position_m": listf(PREGRASP_TARGET),
        "descend_target_position_m": listf(DESCEND_TARGET),
        "lift_target_position_m": listf(LIFT_TARGET),
        "place_target_position_m": listf(PLACE_TARGET),
        "ik_results": ik_results,
        "segment_results": segment_results,
        "total_point_count": int(len(full_trajectory.get("points", []))),
        "total_duration_s": float(validation.get("total_duration_s", 0.0)),
        "maximum_velocity_rad_s_observed": float(validation.get("maximum_velocity_rad_s_observed", math.inf)),
        "maximum_acceleration_rad_s2_observed": float(validation.get("maximum_acceleration_rad_s2_observed", math.inf)),
        "maximum_simultaneously_moving_joints": int(validation.get("maximum_simultaneously_moving_joints", 999)),
        "time_strictly_increasing": bool(validation.get("time_strictly_increasing", False)),
        "all_points_within_urdf_limits": all_points_within_limits,
        "exact_final_target": exact_final_target,
        "csv_path": str(csv_path),
        "tcp_path_csv": str(tcp_csv_path),
        "plot_paths": plot_paths,
        "test_cases": test_cases,
        "opened_com_ports": False,
        "torque_enable_written": False,
        "torque_disable_written": False,
        "goal_position_written": False,
        "motion_command_sent": False,
        "final_status": final_status,
    }
    REPORT_PATH.write_text(json.dumps(report, indent=2), encoding="utf-8")

    lines = [
        "Stage MVP-2 sequential trajectory verification",
        f"urdf_sha256={urdf_hash}",
        f"ik_success={ik_success}",
        f"total_point_count={report['total_point_count']}",
        f"total_duration_s={report['total_duration_s']:.9f}",
        f"maximum_velocity_rad_s_observed={report['maximum_velocity_rad_s_observed']:.12f}",
        f"maximum_acceleration_rad_s2_observed={report['maximum_acceleration_rad_s2_observed']:.12f}",
        f"maximum_simultaneously_moving_joints={report['maximum_simultaneously_moving_joints']}",
        f"all_tests_success={all_tests_success}",
        f"final_status={final_status}",
        f"report={REPORT_PATH}",
    ]
    LOG_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))
    return 0 if final_status == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
