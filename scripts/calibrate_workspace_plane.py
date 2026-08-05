from __future__ import annotations

import json
import math
import time
from collections import defaultdict
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

INTRINSICS_REPORT_PATH = (
    PROJECT_ROOT
    / "data"
    / "calibration"
    / "camera_calibration_report.json"
)

OUTPUT_PATH = (
    PROJECT_ROOT
    / "data"
    / "calibration"
    / "workspace_plane_calibration.json"
)

DEBUG_IMAGE_PATH = (
    PROJECT_ROOT
    / "data"
    / "calibration"
    / "workspace_plane_calibration.png"
)

CAMERA_INDEX = 1
CAMERA_BACKEND = cv2.CAP_DSHOW

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
REQUESTED_FPS = 30.0

TARGET_VALID_FRAMES = 60
MIN_CORNERS_PER_FRAME = 20
MIN_STABLE_CORNER_IDS = 16

SAMPLE_PERIOD_S = 0.08


def load_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"File not found: {path}")

    with path.open("r", encoding="utf-8") as file:
        data = json.load(file)

    if not isinstance(data, dict):
        raise ValueError(f"Expected JSON object: {path}")

    return data


def create_board(
    board_config: dict[str, Any],
) -> tuple[
    cv2.aruco.CharucoBoard,
    cv2.aruco.CharucoDetector,
]:
    if board_config["dictionary"] != "DICT_4X4_50":
        raise ValueError("Expected DICT_4X4_50")

    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_4X4_50
    )

    board = cv2.aruco.CharucoBoard(
        (
            int(board_config["squares_x"]),
            int(board_config["squares_y"]),
        ),
        float(board_config["square_length_m"]),
        float(board_config["marker_length_m"]),
        dictionary,
    )

    detector = cv2.aruco.CharucoDetector(board)

    return board, detector


def load_intrinsics(
    report: dict[str, Any],
) -> tuple[np.ndarray, np.ndarray]:
    camera_matrix = np.asarray(
        report["camera_matrix"],
        dtype=np.float64,
    )

    distortion_coefficients = np.asarray(
        report["distortion_coefficients"],
        dtype=np.float64,
    ).reshape(1, -1)

    if camera_matrix.shape != (3, 3):
        raise ValueError(
            f"Invalid camera matrix shape: "
            f"{camera_matrix.shape}"
        )

    if not np.all(np.isfinite(camera_matrix)):
        raise ValueError(
            "Camera matrix contains NaN or Inf"
        )

    if not np.all(
        np.isfinite(distortion_coefficients)
    ):
        raise ValueError(
            "Distortion coefficients contain NaN or Inf"
        )

    return camera_matrix, distortion_coefficients


def transform_points(
    points: np.ndarray,
    homography: np.ndarray,
) -> np.ndarray:
    transformed = cv2.perspectiveTransform(
        np.asarray(
            points,
            dtype=np.float64,
        ).reshape(-1, 1, 2),
        homography,
    )

    return transformed.reshape(-1, 2)


def calculate_errors_mm(
    predicted: np.ndarray,
    expected: np.ndarray,
) -> tuple[float, float, float]:
    differences = predicted - expected

    distances = np.linalg.norm(
        differences,
        axis=1,
    )

    rms = math.sqrt(
        float(np.mean(distances * distances))
    )

    mean = float(np.mean(distances))
    maximum = float(np.max(distances))

    return rms, mean, maximum


def classify_quality(
    holdout_rms_mm: float,
    holdout_max_mm: float,
) -> str:
    if (
        holdout_rms_mm <= 1.5
        and holdout_max_mm <= 3.0
    ):
        return "PASS"

    if (
        holdout_rms_mm <= 3.0
        and holdout_max_mm <= 6.0
    ):
        return "REVIEW"

    return "FAIL"


def draw_axes(
    image: np.ndarray,
    plane_to_pixel: np.ndarray,
    axis_length_mm: float,
) -> None:
    axis_points_mm = np.asarray(
        [
            [0.0, 0.0],
            [axis_length_mm, 0.0],
            [0.0, axis_length_mm],
        ],
        dtype=np.float64,
    )

    axis_pixels = transform_points(
        axis_points_mm,
        plane_to_pixel,
    )

    origin = tuple(
        np.round(axis_pixels[0]).astype(int)
    )
    x_endpoint = tuple(
        np.round(axis_pixels[1]).astype(int)
    )
    y_endpoint = tuple(
        np.round(axis_pixels[2]).astype(int)
    )

    cv2.arrowedLine(
        image,
        origin,
        x_endpoint,
        (0, 0, 255),
        3,
        cv2.LINE_AA,
        tipLength=0.12,
    )

    cv2.arrowedLine(
        image,
        origin,
        y_endpoint,
        (0, 255, 0),
        3,
        cv2.LINE_AA,
        tipLength=0.12,
    )

    cv2.circle(
        image,
        origin,
        6,
        (255, 255, 0),
        -1,
    )

    cv2.putText(
        image,
        "O",
        (origin[0] + 8, origin[1] - 8),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 0),
        2,
        cv2.LINE_AA,
    )

    cv2.putText(
        image,
        "X",
        (x_endpoint[0] + 5, x_endpoint[1]),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 0, 255),
        2,
        cv2.LINE_AA,
    )

    cv2.putText(
        image,
        "Y",
        (y_endpoint[0] + 5, y_endpoint[1]),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )


def main() -> int:
    board_config = load_json(
        BOARD_CONFIG_PATH
    )

    intrinsics_report = load_json(
        INTRINSICS_REPORT_PATH
    )

    board, detector = create_board(
        board_config
    )

    camera_matrix, distortion_coefficients = (
        load_intrinsics(intrinsics_report)
    )

    chessboard_corners_m = np.asarray(
        board.getChessboardCorners(),
        dtype=np.float64,
    ).reshape(-1, 3)

    chessboard_corners_mm = (
        chessboard_corners_m[:, :2] * 1000.0
    )

    capture = cv2.VideoCapture(
        CAMERA_INDEX,
        CAMERA_BACKEND,
    )

    if not capture.isOpened():
        print(
            "FAIL: could not open "
            "camera index=1 with DSHOW"
        )
        return 1

    capture.set(
        cv2.CAP_PROP_FRAME_WIDTH,
        IMAGE_WIDTH,
    )
    capture.set(
        cv2.CAP_PROP_FRAME_HEIGHT,
        IMAGE_HEIGHT,
    )
    capture.set(
        cv2.CAP_PROP_FPS,
        REQUESTED_FPS,
    )

    actual_width = int(
        capture.get(cv2.CAP_PROP_FRAME_WIDTH)
    )
    actual_height = int(
        capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
    )

    if (
        actual_width != IMAGE_WIDTH
        or actual_height != IMAGE_HEIGHT
    ):
        capture.release()

        print(
            "FAIL: camera resolution mismatch: "
            f"{actual_width}x{actual_height}"
        )
        return 1

    observations_by_id: dict[
        int,
        list[np.ndarray],
    ] = defaultdict(list)

    valid_frame_count = 0
    last_sample_time = 0.0
    final_raw_frame: np.ndarray | None = None
    final_corners: np.ndarray | None = None
    final_ids: np.ndarray | None = None

    print("Workspace plane calibration")
    print(
        "Keep the camera and board completely still."
    )
    print(
        f"Collecting {TARGET_VALID_FRAMES} "
        "valid frames..."
    )
    print("Press Q to cancel.")
    print()

    try:
        for _ in range(20):
            capture.read()

        while valid_frame_count < TARGET_VALID_FRAMES:
            ok, frame = capture.read()

            if (
                not ok
                or frame is None
                or frame.size == 0
            ):
                continue

            (
                charuco_corners,
                charuco_ids,
                marker_corners,
                marker_ids,
            ) = detector.detectBoard(frame)

            corner_count = (
                0
                if charuco_ids is None
                else int(len(charuco_ids))
            )

            display = frame.copy()

            if (
                marker_ids is not None
                and len(marker_ids) > 0
            ):
                cv2.aruco.drawDetectedMarkers(
                    display,
                    marker_corners,
                    marker_ids,
                )

            if (
                charuco_ids is not None
                and corner_count > 0
            ):
                cv2.aruco.drawDetectedCornersCharuco(
                    display,
                    charuco_corners,
                    charuco_ids,
                )

            ready = (
                charuco_corners is not None
                and charuco_ids is not None
                and corner_count
                >= MIN_CORNERS_PER_FRAME
            )

            now = time.monotonic()

            if (
                ready
                and now - last_sample_time
                >= SAMPLE_PERIOD_S
            ):
                ids = np.asarray(
                    charuco_ids,
                    dtype=np.int32,
                ).reshape(-1)

                corners = np.asarray(
                    charuco_corners,
                    dtype=np.float64,
                ).reshape(-1, 2)

                for corner_id, pixel in zip(
                    ids,
                    corners,
                    strict=True,
                ):
                    observations_by_id[
                        int(corner_id)
                    ].append(pixel.copy())

                valid_frame_count += 1
                last_sample_time = now

                final_raw_frame = frame.copy()
                final_corners = corners.copy()
                final_ids = ids.copy()

            status_color = (
                (0, 255, 0)
                if ready
                else (0, 165, 255)
            )

            cv2.putText(
                display,
                f"Corners: {corner_count}",
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                status_color,
                2,
                cv2.LINE_AA,
            )

            cv2.putText(
                display,
                (
                    f"Frames: {valid_frame_count}/"
                    f"{TARGET_VALID_FRAMES}"
                ),
                (15, 62),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                status_color,
                2,
                cv2.LINE_AA,
            )

            cv2.putText(
                display,
                "Do not move camera or board",
                (15, 94),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                status_color,
                2,
                cv2.LINE_AA,
            )

            cv2.imshow(
                "Workspace Plane Calibration",
                display,
            )

            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), ord("Q"), 27):
                print("CANCELLED")
                return 1

    finally:
        capture.release()
        cv2.destroyAllWindows()

    if (
        final_raw_frame is None
        or final_corners is None
        or final_ids is None
    ):
        print("FAIL: no valid frames collected")
        return 1

    minimum_observation_count = int(
        TARGET_VALID_FRAMES * 0.8
    )

    stable_ids = sorted(
        corner_id
        for corner_id, observations
        in observations_by_id.items()
        if len(observations)
        >= minimum_observation_count
    )

    if len(stable_ids) < MIN_STABLE_CORNER_IDS:
        print(
            "FAIL: only "
            f"{len(stable_ids)} stable corner IDs"
        )
        return 1

    median_raw_pixels: list[np.ndarray] = []
    plane_points_mm: list[np.ndarray] = []

    for corner_id in stable_ids:
        observations = np.asarray(
            observations_by_id[corner_id],
            dtype=np.float64,
        )

        median_pixel = np.median(
            observations,
            axis=0,
        )

        median_raw_pixels.append(median_pixel)

        plane_points_mm.append(
            chessboard_corners_mm[corner_id]
        )

    median_raw_pixels_array = np.asarray(
        median_raw_pixels,
        dtype=np.float64,
    )

    plane_points_mm_array = np.asarray(
        plane_points_mm,
        dtype=np.float64,
    )

    undistorted_pixels = cv2.undistortPoints(
        median_raw_pixels_array.reshape(
            -1,
            1,
            2,
        ),
        camera_matrix,
        distortion_coefficients,
        P=camera_matrix,
    ).reshape(-1, 2)

    stable_ids_array = np.asarray(
        stable_ids,
        dtype=np.int32,
    )

    holdout_mask = (
        stable_ids_array % 3 == 0
    )
    fit_mask = ~holdout_mask

    if np.count_nonzero(fit_mask) < 8:
        print("FAIL: insufficient fit points")
        return 1

    if np.count_nonzero(holdout_mask) < 4:
        print("FAIL: insufficient holdout points")
        return 1

    fit_homography, _ = cv2.findHomography(
        undistorted_pixels[fit_mask],
        plane_points_mm_array[fit_mask],
        method=0,
    )

    if fit_homography is None:
        print("FAIL: could not estimate homography")
        return 1

    fit_predictions = transform_points(
        undistorted_pixels[fit_mask],
        fit_homography,
    )

    holdout_predictions = transform_points(
        undistorted_pixels[holdout_mask],
        fit_homography,
    )

    fit_rms_mm, fit_mean_mm, fit_max_mm = (
        calculate_errors_mm(
            fit_predictions,
            plane_points_mm_array[fit_mask],
        )
    )

    (
        holdout_rms_mm,
        holdout_mean_mm,
        holdout_max_mm,
    ) = calculate_errors_mm(
        holdout_predictions,
        plane_points_mm_array[holdout_mask],
    )

    final_homography, _ = cv2.findHomography(
        undistorted_pixels,
        plane_points_mm_array,
        method=0,
    )

    if final_homography is None:
        print(
            "FAIL: could not estimate final homography"
        )
        return 1

    plane_to_pixel = np.linalg.inv(
        final_homography
    )

    all_predictions = transform_points(
        undistorted_pixels,
        final_homography,
    )

    all_rms_mm, all_mean_mm, all_max_mm = (
        calculate_errors_mm(
            all_predictions,
            plane_points_mm_array,
        )
    )

    quality = classify_quality(
        holdout_rms_mm,
        holdout_max_mm,
    )

    undistorted_debug = cv2.undistort(
        final_raw_frame,
        camera_matrix,
        distortion_coefficients,
        None,
        camera_matrix,
    )

    for corner_id, pixel, expected_mm in zip(
        stable_ids,
        undistorted_pixels,
        plane_points_mm_array,
        strict=True,
    ):
        point = tuple(
            np.round(pixel).astype(int)
        )

        cv2.circle(
            undistorted_debug,
            point,
            4,
            (255, 0, 255),
            -1,
        )

        cv2.putText(
            undistorted_debug,
            str(corner_id),
            (point[0] + 4, point[1] - 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 0),
            1,
            cv2.LINE_AA,
        )

    axis_length_mm = (
        float(board_config["square_length_m"])
        * 1000.0
        * 2.0
    )

    draw_axes(
        undistorted_debug,
        plane_to_pixel,
        axis_length_mm,
    )

    cv2.putText(
        undistorted_debug,
        (
            f"{quality} | holdout RMS="
            f"{holdout_rms_mm:.3f} mm | "
            f"max={holdout_max_mm:.3f} mm"
        ),
        (15, undistorted_debug.shape[0] - 18),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        (0, 255, 0)
        if quality == "PASS"
        else (0, 165, 255),
        2,
        cv2.LINE_AA,
    )

    DEBUG_IMAGE_PATH.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    cv2.imwrite(
        str(DEBUG_IMAGE_PATH),
        undistorted_debug,
    )

    result = {
        "version": "1.0",
        "quality": quality,
        "coordinate_frame": (
            "opencv_charuco_board_plane"
        ),
        "coordinate_unit": "millimeter",
        "input_pixel_space": (
            "undistorted_pixels_using_original_camera_matrix"
        ),
        "camera": {
            "index": CAMERA_INDEX,
            "backend": "dshow",
            "width": IMAGE_WIDTH,
            "height": IMAGE_HEIGHT,
        },
        "board": board_config,
        "sampling": {
            "target_valid_frames": (
                TARGET_VALID_FRAMES
            ),
            "minimum_corners_per_frame": (
                MIN_CORNERS_PER_FRAME
            ),
            "stable_corner_ids": stable_ids,
            "stable_corner_count": len(stable_ids),
        },
        "quality_metrics_mm": {
            "fit_rms": fit_rms_mm,
            "fit_mean": fit_mean_mm,
            "fit_max": fit_max_mm,
            "holdout_rms": holdout_rms_mm,
            "holdout_mean": holdout_mean_mm,
            "holdout_max": holdout_max_mm,
            "all_point_rms": all_rms_mm,
            "all_point_mean": all_mean_mm,
            "all_point_max": all_max_mm,
        },
        "homography_undistorted_pixel_to_plane_mm": (
            final_homography.tolist()
        ),
        "homography_plane_mm_to_undistorted_pixel": (
            plane_to_pixel.tolist()
        ),
        "notes": [
            (
                "The camera and workspace plane must remain "
                "fixed after this calibration."
            ),
            (
                "This transform maps image points to the "
                "physical tabletop plane."
            ),
            (
                "Object height above the plane may introduce "
                "parallax error."
            ),
        ],
    }

    with OUTPUT_PATH.open(
        "w",
        encoding="utf-8",
    ) as file:
        json.dump(
            result,
            file,
            indent=2,
            ensure_ascii=False,
        )

    print()
    print("=== WORKSPACE PLANE SUMMARY ===")
    print(f"QUALITY={quality}")
    print(
        f"VALID_FRAMES={TARGET_VALID_FRAMES}"
    )
    print(
        f"STABLE_CORNER_IDS={len(stable_ids)}"
    )
    print(f"FIT_RMS_MM={fit_rms_mm:.6f}")
    print(f"FIT_MAX_MM={fit_max_mm:.6f}")
    print(
        f"HOLDOUT_RMS_MM={holdout_rms_mm:.6f}"
    )
    print(
        f"HOLDOUT_MAX_MM={holdout_max_mm:.6f}"
    )
    print(f"ALL_POINT_RMS_MM={all_rms_mm:.6f}")
    print(f"ALL_POINT_MAX_MM={all_max_mm:.6f}")
    print()
    print(f"CALIBRATION={OUTPUT_PATH}")
    print(f"DEBUG_IMAGE={DEBUG_IMAGE_PATH}")

    if quality == "PASS":
        print(
            "PASS: workspace plane calibration accepted"
        )
        return 0

    if quality == "REVIEW":
        print(
            "REVIEW: inspect the debug image before use"
        )
        return 0

    print(
        "FAIL: plane calibration error is too high"
    )
    return 2


if __name__ == "__main__":
    raise SystemExit(main())