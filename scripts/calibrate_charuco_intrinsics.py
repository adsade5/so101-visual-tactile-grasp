from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]

BOARD_CONFIG_PATH = (
    PROJECT_ROOT
    / "data"
    / "calibration"
    / "charuco_board_5x7_actual.json"
)

CAPTURES_ROOT = (
    PROJECT_ROOT
    / "data"
    / "calibration"
    / "captures"
)

CALIBRATION_ROOT = (
    PROJECT_ROOT
    / "data"
    / "calibration"
)

MIN_CHARUCO_CORNERS = 12
MIN_VALID_IMAGES = 15


def load_board_config() -> dict[str, Any]:
    if not BOARD_CONFIG_PATH.is_file():
        raise FileNotFoundError(
            f"Board config not found: {BOARD_CONFIG_PATH}"
        )

    with BOARD_CONFIG_PATH.open(
        "r",
        encoding="utf-8",
    ) as file:
        config = json.load(file)

    required = {
        "dictionary",
        "squares_x",
        "squares_y",
        "square_length_m",
        "marker_length_m",
    }

    missing = required - config.keys()

    if missing:
        raise ValueError(
            f"Board config missing fields: {sorted(missing)}"
        )

    if config["dictionary"] != "DICT_4X4_50":
        raise ValueError(
            "This calibration script currently supports "
            "DICT_4X4_50 only"
        )

    if int(config["squares_x"]) != 7:
        raise ValueError(
            "Expected squares_x=7 for the physical board"
        )

    if int(config["squares_y"]) != 5:
        raise ValueError(
            "Expected squares_y=5 for the physical board"
        )

    square_length = float(config["square_length_m"])
    marker_length = float(config["marker_length_m"])

    if square_length <= 0 or marker_length <= 0:
        raise ValueError(
            "Board dimensions must be positive"
        )

    if marker_length >= square_length:
        raise ValueError(
            "marker_length must be smaller than square_length"
        )

    return config


def create_board(
    config: dict[str, Any],
) -> tuple[
    cv2.aruco.CharucoBoard,
    cv2.aruco.CharucoDetector,
]:
    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_4X4_50
    )

    board = cv2.aruco.CharucoBoard(
        (
            int(config["squares_x"]),
            int(config["squares_y"]),
        ),
        float(config["square_length_m"]),
        float(config["marker_length_m"]),
        dictionary,
    )

    detector = cv2.aruco.CharucoDetector(board)

    return board, detector


def resolve_capture_session(
    session_argument: str | None,
) -> Path:
    if session_argument:
        session_path = Path(session_argument)

        if not session_path.is_absolute():
            session_path = PROJECT_ROOT / session_path

        session_path = session_path.resolve()

        if not session_path.is_dir():
            raise FileNotFoundError(
                f"Capture session does not exist: {session_path}"
            )

        return session_path

    sessions = sorted(
        (
            path
            for path in CAPTURES_ROOT.glob("session_*")
            if path.is_dir()
        ),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )

    if not sessions:
        raise FileNotFoundError(
            f"No capture sessions found in {CAPTURES_ROOT}"
        )

    return sessions[0]


def calculate_per_view_error(
    object_points: np.ndarray,
    image_points: np.ndarray,
    rotation_vector: np.ndarray,
    translation_vector: np.ndarray,
    camera_matrix: np.ndarray,
    distortion_coefficients: np.ndarray,
) -> float:
    projected_points, _ = cv2.projectPoints(
        object_points,
        rotation_vector,
        translation_vector,
        camera_matrix,
        distortion_coefficients,
    )

    observed = np.asarray(
        image_points,
        dtype=np.float64,
    ).reshape(-1, 2)

    projected = np.asarray(
        projected_points,
        dtype=np.float64,
    ).reshape(-1, 2)

    differences = observed - projected
    squared_distances = np.sum(
        differences * differences,
        axis=1,
    )

    return float(
        math.sqrt(float(np.mean(squared_distances)))
    )


def matrix_as_list(
    matrix: np.ndarray,
) -> list[float]:
    return [
        float(value)
        for value in np.asarray(matrix).reshape(-1)
    ]


def format_yaml_list(values: list[float]) -> str:
    return ", ".join(
        f"{value:.12g}"
        for value in values
    )


def write_intrinsics_yaml(
    output_path: Path,
    *,
    image_width: int,
    image_height: int,
    camera_matrix: np.ndarray,
    distortion_coefficients: np.ndarray,
    new_camera_matrix: np.ndarray,
    roi: tuple[int, int, int, int],
    rms_error: float,
    mean_per_view_error: float,
    maximum_per_view_error: float,
    valid_image_count: int,
    board_config: dict[str, Any],
    capture_session: Path,
) -> None:
    camera_data = matrix_as_list(camera_matrix)
    distortion_data = matrix_as_list(
        distortion_coefficients
    )
    new_camera_data = matrix_as_list(
        new_camera_matrix
    )

    yaml_text = f"""# SO-101 workspace camera intrinsic calibration
# Generated from ChArUco images.

camera_name: workspace_camera

image_width: {image_width}
image_height: {image_height}

camera_matrix:
  rows: 3
  cols: 3
  data: [{format_yaml_list(camera_data)}]

distortion_model: plumb_bob

distortion_coefficients:
  rows: 1
  cols: {len(distortion_data)}
  data: [{format_yaml_list(distortion_data)}]

optimal_new_camera_matrix:
  rows: 3
  cols: 3
  data: [{format_yaml_list(new_camera_data)}]

valid_roi:
  x: {roi[0]}
  y: {roi[1]}
  width: {roi[2]}
  height: {roi[3]}

calibration_quality:
  rms_reprojection_error_px: {rms_error:.12g}
  mean_per_view_error_px: {mean_per_view_error:.12g}
  maximum_per_view_error_px: {maximum_per_view_error:.12g}
  valid_image_count: {valid_image_count}

capture:
  session: "{capture_session.as_posix()}"
  camera_index: 1
  backend: "dshow"

board:
  type: "charuco"
  dictionary: "{board_config['dictionary']}"
  squares_x: {int(board_config['squares_x'])}
  squares_y: {int(board_config['squares_y'])}
  square_length_m: {float(board_config['square_length_m']):.12g}
  marker_length_m: {float(board_config['marker_length_m']):.12g}
"""

    output_path.write_text(
        yaml_text,
        encoding="utf-8",
    )


def save_undistortion_comparison(
    image_path: Path,
    output_path: Path,
    camera_matrix: np.ndarray,
    distortion_coefficients: np.ndarray,
    new_camera_matrix: np.ndarray,
    label: str,
) -> None:
    image = cv2.imread(
        str(image_path),
        cv2.IMREAD_COLOR,
    )

    if image is None:
        return

    undistorted = cv2.undistort(
        image,
        camera_matrix,
        distortion_coefficients,
        None,
        new_camera_matrix,
    )

    original_display = image.copy()
    undistorted_display = undistorted.copy()

    cv2.putText(
        original_display,
        "Original",
        (15, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )

    cv2.putText(
        undistorted_display,
        "Undistorted",
        (15, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )

    comparison = np.hstack(
        [original_display, undistorted_display]
    )

    cv2.putText(
        comparison,
        label,
        (15, comparison.shape[0] - 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )

    cv2.imwrite(
        str(output_path),
        comparison,
    )


def classify_quality(
    rms_error: float,
    maximum_per_view_error: float,
) -> str:
    # 这是本项目的工程验收阈值，不是OpenCV强制标准。
    if rms_error <= 1.0 and maximum_per_view_error <= 1.5:
        return "PASS"

    if rms_error <= 1.5 and maximum_per_view_error <= 2.5:
        return "REVIEW"

    return "FAIL"


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Calibrate SO-101 workspace camera intrinsics "
            "from a ChArUco capture session."
        )
    )

    parser.add_argument(
        "--session",
        default=None,
        help=(
            "Capture session directory. When omitted, "
            "the newest session_* directory is used."
        ),
    )

    args = parser.parse_args()

    board_config = load_board_config()
    board, detector = create_board(board_config)
    session_path = resolve_capture_session(args.session)

    image_paths = sorted(
        session_path.glob("charuco_*.png")
    )

    if not image_paths:
        print(
            f"FAIL: no charuco_*.png images in {session_path}"
        )
        return 1

    output_directory = (
        CALIBRATION_ROOT
        / f"results_{session_path.name}"
    )

    debug_directory = (
        output_directory
        / "detections"
    )

    comparison_directory = (
        output_directory
        / "undistortion"
    )

    debug_directory.mkdir(
        parents=True,
        exist_ok=True,
    )

    comparison_directory.mkdir(
        parents=True,
        exist_ok=True,
    )

    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    accepted_records: list[dict[str, Any]] = []
    all_records: list[dict[str, Any]] = []

    image_size: tuple[int, int] | None = None

    print(f"Capture session: {session_path}")
    print(f"Input images: {len(image_paths)}")
    print(
        "Board: "
        f"{board_config['squares_x']}x"
        f"{board_config['squares_y']}, "
        f"square={float(board_config['square_length_m']) * 1000:.2f} mm, "
        f"marker={float(board_config['marker_length_m']) * 1000:.2f} mm"
    )
    print()

    for image_path in image_paths:
        grayscale = cv2.imread(
            str(image_path),
            cv2.IMREAD_GRAYSCALE,
        )

        if grayscale is None:
            all_records.append(
                {
                    "filename": image_path.name,
                    "accepted": False,
                    "reason": "image_load_failed",
                }
            )
            print(
                f"REJECT {image_path.name}: load failed"
            )
            continue

        current_size = (
            grayscale.shape[1],
            grayscale.shape[0],
        )

        if image_size is None:
            image_size = current_size
        elif current_size != image_size:
            all_records.append(
                {
                    "filename": image_path.name,
                    "accepted": False,
                    "reason": (
                        "image_size_mismatch"
                    ),
                    "image_size": list(current_size),
                }
            )
            print(
                f"REJECT {image_path.name}: "
                f"size={current_size}, expected={image_size}"
            )
            continue

        (
            charuco_corners,
            charuco_ids,
            marker_corners,
            marker_ids,
        ) = detector.detectBoard(grayscale)

        corner_count = (
            0
            if charuco_ids is None
            else int(len(charuco_ids))
        )

        marker_count = (
            0
            if marker_ids is None
            else int(len(marker_ids))
        )

        visualization = cv2.cvtColor(
            grayscale,
            cv2.COLOR_GRAY2BGR,
        )

        if marker_ids is not None and marker_count > 0:
            cv2.aruco.drawDetectedMarkers(
                visualization,
                marker_corners,
                marker_ids,
            )

        if charuco_ids is not None and corner_count > 0:
            cv2.aruco.drawDetectedCornersCharuco(
                visualization,
                charuco_corners,
                charuco_ids,
            )

        cv2.putText(
            visualization,
            f"corners={corner_count}, markers={marker_count}",
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.imwrite(
            str(debug_directory / image_path.name),
            visualization,
        )

        if (
            charuco_ids is None
            or charuco_corners is None
            or corner_count < MIN_CHARUCO_CORNERS
        ):
            all_records.append(
                {
                    "filename": image_path.name,
                    "accepted": False,
                    "reason": (
                        "insufficient_charuco_corners"
                    ),
                    "charuco_corner_count": corner_count,
                    "marker_count": marker_count,
                }
            )

            print(
                f"REJECT {image_path.name}: "
                f"corners={corner_count}"
            )
            continue

        frame_object_points, frame_image_points = (
            board.matchImagePoints(
                charuco_corners,
                charuco_ids,
            )
        )

        frame_object_points = np.asarray(
            frame_object_points,
            dtype=np.float32,
        ).reshape(-1, 1, 3)

        frame_image_points = np.asarray(
            frame_image_points,
            dtype=np.float32,
        ).reshape(-1, 1, 2)

        if (
            len(frame_object_points)
            != len(frame_image_points)
        ):
            raise RuntimeError(
                "Object/image point count mismatch for "
                f"{image_path.name}"
            )

        object_points.append(frame_object_points)
        image_points.append(frame_image_points)

        accepted_record = {
            "filename": image_path.name,
            "path": str(image_path),
            "accepted": True,
            "charuco_corner_count": corner_count,
            "marker_count": marker_count,
        }

        accepted_records.append(accepted_record)
        all_records.append(accepted_record)

        print(
            f"ACCEPT {image_path.name}: "
            f"corners={corner_count}, "
            f"markers={marker_count}"
        )

    if image_size is None:
        print("FAIL: no readable images")
        return 1

    if len(object_points) < MIN_VALID_IMAGES:
        print()
        print(
            "FAIL: only "
            f"{len(object_points)} valid calibration images; "
            f"at least {MIN_VALID_IMAGES} are required"
        )
        return 1

    print()
    print(
        f"Running calibration with "
        f"{len(object_points)} valid images..."
    )

    (
        rms_error,
        camera_matrix,
        distortion_coefficients,
        rotation_vectors,
        translation_vectors,
    ) = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
    )

    per_view_errors: list[float] = []

    total_squared_error = 0.0
    total_point_count = 0

    for index, record in enumerate(accepted_records):
        per_view_error = calculate_per_view_error(
            object_points[index],
            image_points[index],
            rotation_vectors[index],
            translation_vectors[index],
            camera_matrix,
            distortion_coefficients,
        )

        per_view_errors.append(per_view_error)
        record["reprojection_error_px"] = per_view_error

        projected_points, _ = cv2.projectPoints(
            object_points[index],
            rotation_vectors[index],
            translation_vectors[index],
            camera_matrix,
            distortion_coefficients,
        )

        differences = (
            image_points[index].reshape(-1, 2)
            - projected_points.reshape(-1, 2)
        )

        total_squared_error += float(
            np.sum(differences * differences)
        )
        total_point_count += len(differences)

    global_recomputed_error = math.sqrt(
        total_squared_error / total_point_count
    )

    mean_per_view_error = float(
        np.mean(per_view_errors)
    )

    median_per_view_error = float(
        np.median(per_view_errors)
    )

    maximum_per_view_error = float(
        np.max(per_view_errors)
    )

    minimum_per_view_error = float(
        np.min(per_view_errors)
    )

    new_camera_matrix, roi_array = (
        cv2.getOptimalNewCameraMatrix(
            camera_matrix,
            distortion_coefficients,
            image_size,
            1.0,
            image_size,
        )
    )

    roi = tuple(
        int(value)
        for value in roi_array
    )

    quality = classify_quality(
        float(rms_error),
        maximum_per_view_error,
    )

    canonical_yaml_path = (
        CALIBRATION_ROOT
        / "camera_intrinsics.yaml"
    )

    session_yaml_path = (
        output_directory
        / "camera_intrinsics.yaml"
    )

    write_intrinsics_yaml(
        session_yaml_path,
        image_width=image_size[0],
        image_height=image_size[1],
        camera_matrix=camera_matrix,
        distortion_coefficients=(
            distortion_coefficients
        ),
        new_camera_matrix=new_camera_matrix,
        roi=roi,
        rms_error=float(rms_error),
        mean_per_view_error=mean_per_view_error,
        maximum_per_view_error=maximum_per_view_error,
        valid_image_count=len(object_points),
        board_config=board_config,
        capture_session=session_path,
    )

    write_intrinsics_yaml(
        canonical_yaml_path,
        image_width=image_size[0],
        image_height=image_size[1],
        camera_matrix=camera_matrix,
        distortion_coefficients=(
            distortion_coefficients
        ),
        new_camera_matrix=new_camera_matrix,
        roi=roi,
        rms_error=float(rms_error),
        mean_per_view_error=mean_per_view_error,
        maximum_per_view_error=maximum_per_view_error,
        valid_image_count=len(object_points),
        board_config=board_config,
        capture_session=session_path,
    )

    report = {
        "quality": quality,
        "capture_session": str(session_path),
        "image_size": {
            "width": image_size[0],
            "height": image_size[1],
        },
        "board": board_config,
        "input_image_count": len(image_paths),
        "valid_image_count": len(object_points),
        "rejected_image_count": (
            len(image_paths) - len(object_points)
        ),
        "rms_reprojection_error_px": float(
            rms_error
        ),
        "global_recomputed_error_px": (
            global_recomputed_error
        ),
        "mean_per_view_error_px": (
            mean_per_view_error
        ),
        "median_per_view_error_px": (
            median_per_view_error
        ),
        "minimum_per_view_error_px": (
            minimum_per_view_error
        ),
        "maximum_per_view_error_px": (
            maximum_per_view_error
        ),
        "camera_matrix": (
            np.asarray(camera_matrix).tolist()
        ),
        "distortion_coefficients": (
            np.asarray(
                distortion_coefficients
            ).reshape(-1).tolist()
        ),
        "optimal_new_camera_matrix": (
            np.asarray(new_camera_matrix).tolist()
        ),
        "valid_roi": {
            "x": roi[0],
            "y": roi[1],
            "width": roi[2],
            "height": roi[3],
        },
        "images": all_records,
    }

    session_report_path = (
        output_directory
        / "calibration_report.json"
    )

    canonical_report_path = (
        CALIBRATION_ROOT
        / "camera_calibration_report.json"
    )

    for report_path in (
        session_report_path,
        canonical_report_path,
    ):
        with report_path.open(
            "w",
            encoding="utf-8",
        ) as file:
            json.dump(
                report,
                file,
                indent=2,
                ensure_ascii=False,
            )

    error_order = sorted(
        range(len(per_view_errors)),
        key=lambda index: per_view_errors[index],
    )

    selected_indices = {
        "best": error_order[0],
        "median": error_order[
            len(error_order) // 2
        ],
        "worst": error_order[-1],
    }

    for label, index in selected_indices.items():
        image_path = Path(
            accepted_records[index]["path"]
        )

        error = per_view_errors[index]

        save_undistortion_comparison(
            image_path,
            (
                comparison_directory
                / f"{label}_{image_path.name}"
            ),
            camera_matrix,
            distortion_coefficients,
            new_camera_matrix,
            (
                f"{label}: "
                f"reprojection error={error:.3f}px"
            ),
        )

    print()
    print("=== CALIBRATION SUMMARY ===")
    print(f"QUALITY={quality}")
    print(f"INPUT_IMAGES={len(image_paths)}")
    print(f"VALID_IMAGES={len(object_points)}")
    print(
        "REJECTED_IMAGES="
        f"{len(image_paths) - len(object_points)}"
    )
    print(f"IMAGE_SIZE={image_size[0]}x{image_size[1]}")
    print(f"RMS_ERROR_PX={float(rms_error):.6f}")
    print(
        "GLOBAL_RECOMPUTED_ERROR_PX="
        f"{global_recomputed_error:.6f}"
    )
    print(
        "MEAN_PER_VIEW_ERROR_PX="
        f"{mean_per_view_error:.6f}"
    )
    print(
        "MEDIAN_PER_VIEW_ERROR_PX="
        f"{median_per_view_error:.6f}"
    )
    print(
        "MAX_PER_VIEW_ERROR_PX="
        f"{maximum_per_view_error:.6f}"
    )
    print()
    print("CAMERA_MATRIX=")
    print(camera_matrix)
    print()
    print("DISTORTION_COEFFICIENTS=")
    print(
        np.asarray(
            distortion_coefficients
        ).reshape(-1)
    )
    print()
    print(f"INTRINSICS={canonical_yaml_path}")
    print(f"REPORT={canonical_report_path}")
    print(f"DEBUG_DETECTIONS={debug_directory}")
    print(
        f"UNDISTORTION_COMPARISONS="
        f"{comparison_directory}"
    )

    worst_records = sorted(
        accepted_records,
        key=lambda record: (
            record["reprojection_error_px"]
        ),
        reverse=True,
    )[:5]

    print()
    print("WORST_FIVE_IMAGES:")

    for record in worst_records:
        print(
            f"  {record['filename']}: "
            f"{record['reprojection_error_px']:.6f}px, "
            f"corners={record['charuco_corner_count']}"
        )

    if quality == "PASS":
        print()
        print("PASS: intrinsic calibration accepted")
        return 0

    if quality == "REVIEW":
        print()
        print(
            "REVIEW: calibration is usable for inspection, "
            "but high-error images should be reviewed"
        )
        return 0

    print()
    print(
        "FAIL: reprojection error is too high; "
        "review the detection images and worst views"
    )
    return 2


if __name__ == "__main__":
    raise SystemExit(main())