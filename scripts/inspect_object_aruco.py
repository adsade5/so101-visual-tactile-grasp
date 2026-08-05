from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]

CONFIG_PATH = PROJECT_ROOT / "config" / "object_marker.json"
INTRINSICS_PATH = (
    PROJECT_ROOT
    / "data"
    / "calibration"
    / "camera_calibration_report.json"
)

DICTIONARIES = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
}


def load_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"File not found: {path}")

    with path.open("r", encoding="utf-8") as file:
        value = json.load(file)

    if not isinstance(value, dict):
        raise ValueError(f"Expected JSON object: {path}")

    return value


def load_intrinsics() -> tuple[np.ndarray, np.ndarray]:
    report = load_json(INTRINSICS_PATH)

    camera_matrix = np.asarray(
        report["camera_matrix"],
        dtype=np.float64,
    )

    distortion = np.asarray(
        report["distortion_coefficients"],
        dtype=np.float64,
    ).reshape(1, -1)

    if camera_matrix.shape != (3, 3):
        raise ValueError("Invalid camera matrix")

    if not np.all(np.isfinite(camera_matrix)):
        raise ValueError("Camera matrix contains NaN or Inf")

    if not np.all(np.isfinite(distortion)):
        raise ValueError(
            "Distortion coefficients contain NaN or Inf"
        )

    return camera_matrix, distortion


def create_marker_object_points(
    marker_size_m: float,
) -> np.ndarray:
    half_size = marker_size_m / 2.0

    # 顺序对应检测角点：
    # 左上、右上、右下、左下。
    return np.asarray(
        [
            [-half_size, half_size, 0.0],
            [half_size, half_size, 0.0],
            [half_size, -half_size, 0.0],
            [-half_size, -half_size, 0.0],
        ],
        dtype=np.float64,
    )


def select_target_index(
    detected_ids: np.ndarray,
    target_id: int,
) -> int | None:
    flat_ids = detected_ids.reshape(-1)

    if target_id >= 0:
        matches = np.where(flat_ids == target_id)[0]

        if len(matches) == 0:
            return None

        return int(matches[0])

    # target_id=-1 时，只在画面中恰好有一个 Marker
    # 的情况下使用它，避免选错目标。
    if len(flat_ids) != 1:
        return None

    return 0


def main() -> int:
    config = load_json(CONFIG_PATH)
    camera_matrix, distortion = load_intrinsics()

    dictionary_name = str(config["dictionary"])

    if dictionary_name not in DICTIONARIES:
        raise ValueError(
            f"Unsupported dictionary: {dictionary_name}"
        )

    target_id = int(config["target_id"])
    marker_size_m = float(config["marker_size_m"])

    if marker_size_m <= 0:
        raise ValueError("marker_size_m must be positive")

    camera_config = config["camera"]

    camera_index = int(camera_config["index"])
    width = int(camera_config["width"])
    height = int(camera_config["height"])

    dictionary = cv2.aruco.getPredefinedDictionary(
        DICTIONARIES[dictionary_name]
    )

    detector_parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(
        dictionary,
        detector_parameters,
    )

    marker_object_points = create_marker_object_points(
        marker_size_m
    )

    capture = cv2.VideoCapture(
        camera_index,
        cv2.CAP_DSHOW,
    )

    if not capture.isOpened():
        print("FAIL: could not open workspace camera")
        return 1

    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    capture.set(cv2.CAP_PROP_FPS, 30.0)

    print("ArUco object inspection")
    print(f"Dictionary: {dictionary_name}")
    print(f"Configured target ID: {target_id}")
    print(
        f"Marker size: {marker_size_m * 1000:.1f} mm"
    )
    print("Q or ESC: quit")
    print()

    last_log_time = 0.0

    try:
        for _ in range(20):
            capture.read()

        while True:
            ok, frame = capture.read()

            if not ok or frame is None or frame.size == 0:
                continue

            display = frame.copy()

            marker_corners, marker_ids, rejected = (
                detector.detectMarkers(frame)
            )

            del rejected

            detected_count = (
                0
                if marker_ids is None
                else len(marker_ids)
            )

            if marker_ids is not None:
                cv2.aruco.drawDetectedMarkers(
                    display,
                    marker_corners,
                    marker_ids,
                )

            target_index = None

            if marker_ids is not None:
                target_index = select_target_index(
                    marker_ids,
                    target_id,
                )

            if target_index is not None:
                corners = np.asarray(
                    marker_corners[target_index],
                    dtype=np.float64,
                ).reshape(4, 2)

                detected_id = int(
                    marker_ids[target_index][0]
                )

                center = np.mean(corners, axis=0)

                success, rotation_vector, translation_vector = (
                    cv2.solvePnP(
                        marker_object_points,
                        corners,
                        camera_matrix,
                        distortion,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE,
                    )
                )

                center_pixel = (
                    int(round(center[0])),
                    int(round(center[1])),
                )

                cv2.circle(
                    display,
                    center_pixel,
                    6,
                    (0, 0, 255),
                    -1,
                )

                cv2.putText(
                    display,
                    (
                        f"TARGET ID={detected_id} "
                        f"center={center_pixel}"
                    ),
                    (12, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.68,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

                if success:
                    cv2.drawFrameAxes(
                        display,
                        camera_matrix,
                        distortion,
                        rotation_vector,
                        translation_vector,
                        marker_size_m * 0.75,
                        2,
                    )

                    translation_mm = (
                        translation_vector.reshape(3)
                        * 1000.0
                    )

                    cv2.putText(
                        display,
                        (
                            "camera t="
                            f"({translation_mm[0]:.1f}, "
                            f"{translation_mm[1]:.1f}, "
                            f"{translation_mm[2]:.1f}) mm"
                        ),
                        (12, 62),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.58,
                        (0, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )

                    now = time.monotonic()

                    if now - last_log_time >= 1.0:
                        print(
                            f"ID={detected_id}, "
                            f"center=({center[0]:.2f}, "
                            f"{center[1]:.2f}), "
                            "camera_translation_mm="
                            f"({translation_mm[0]:.2f}, "
                            f"{translation_mm[1]:.2f}, "
                            f"{translation_mm[2]:.2f})"
                        )

                        last_log_time = now

            else:
                cv2.putText(
                    display,
                    (
                        f"Detected markers={detected_count}; "
                        "target is not uniquely selected"
                    ),
                    (12, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.62,
                    (0, 165, 255),
                    2,
                    cv2.LINE_AA,
                )

            cv2.imshow(
                "SO-101 ArUco Object Inspection",
                display,
            )

            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), ord("Q"), 27):
                break

    finally:
        capture.release()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())