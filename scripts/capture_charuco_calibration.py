from __future__ import annotations

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]

BOARD_CONFIG_PATH = (
    PROJECT_ROOT
    / "data"
    / "calibration"
    / "charuco_board_5x7_actual.json"
)

OUTPUT_ROOT = (
    PROJECT_ROOT
    / "data"
    / "calibration"
    / "captures"
)

CAMERA_INDEX = 1
CAMERA_BACKEND = cv2.CAP_DSHOW

REQUESTED_WIDTH = 640
REQUESTED_HEIGHT = 480
REQUESTED_FPS = 30.0

MIN_CHARUCO_CORNERS = 12
TARGET_IMAGE_COUNT = 25


def load_board_config() -> dict:
    if not BOARD_CONFIG_PATH.is_file():
        raise FileNotFoundError(
            f"Board config not found: {BOARD_CONFIG_PATH}"
        )

    with BOARD_CONFIG_PATH.open(
        "r",
        encoding="utf-8",
    ) as file:
        config = json.load(file)

    required_fields = {
        "dictionary",
        "squares_x",
        "squares_y",
        "square_length_m",
        "marker_length_m",
    }

    missing = required_fields - config.keys()

    if missing:
        raise ValueError(
            f"Board config missing fields: {sorted(missing)}"
        )

    return config


def create_board(
    config: dict,
) -> tuple[
    cv2.aruco.CharucoBoard,
    cv2.aruco.CharucoDetector,
]:
    dictionary_name = config["dictionary"]

    if dictionary_name != "DICT_4X4_50":
        raise ValueError(
            f"Unsupported dictionary: {dictionary_name}"
        )

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


def calculate_board_coverage(
    charuco_corners: np.ndarray | None,
    image_width: int,
    image_height: int,
) -> float:
    if charuco_corners is None or len(charuco_corners) < 4:
        return 0.0

    points = np.asarray(
        charuco_corners,
        dtype=np.float32,
    ).reshape(-1, 2)

    x, y, width, height = cv2.boundingRect(points)

    board_area = float(width * height)
    image_area = float(image_width * image_height)

    if image_area <= 0:
        return 0.0

    return board_area / image_area


def draw_status(
    image: np.ndarray,
    *,
    corner_count: int,
    coverage: float,
    saved_count: int,
    message: str,
    ready_to_save: bool,
) -> None:
    status_lines = [
        f"ChArUco corners: {corner_count}",
        f"Board coverage: {coverage * 100:.1f}%",
        f"Saved: {saved_count}/{TARGET_IMAGE_COUNT}",
        "SPACE: save | Q: quit",
        message,
    ]

    for index, text in enumerate(status_lines):
        y = 28 + index * 28

        cv2.putText(
            image,
            text,
            (15, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0) if ready_to_save else (0, 165, 255),
            2,
            cv2.LINE_AA,
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Capture ChArUco images for SO-101 workspace "
            "camera intrinsic calibration."
        )
    )

    parser.add_argument(
        "--camera-index",
        type=int,
        default=CAMERA_INDEX,
    )

    parser.add_argument(
        "--target-count",
        type=int,
        default=TARGET_IMAGE_COUNT,
    )

    args = parser.parse_args()

    if args.target_count < 15:
        parser.error("--target-count must be at least 15")

    board_config = load_board_config()
    _, detector = create_board(board_config)

    session_name = datetime.now().strftime(
        "session_%Y%m%d_%H%M%S"
    )

    output_directory = OUTPUT_ROOT / session_name
    output_directory.mkdir(
        parents=True,
        exist_ok=False,
    )

    capture = cv2.VideoCapture(
        args.camera_index,
        CAMERA_BACKEND,
    )

    if not capture.isOpened():
        print(
            "FAIL: could not open camera "
            f"index={args.camera_index} with DSHOW"
        )
        return 1

    capture.set(
        cv2.CAP_PROP_FRAME_WIDTH,
        REQUESTED_WIDTH,
    )
    capture.set(
        cv2.CAP_PROP_FRAME_HEIGHT,
        REQUESTED_HEIGHT,
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
    reported_fps = float(
        capture.get(cv2.CAP_PROP_FPS)
    )

    session_metadata = {
        "camera_index": args.camera_index,
        "camera_backend": "dshow",
        "width": actual_width,
        "height": actual_height,
        "reported_fps": reported_fps,
        "board_config": board_config,
        "minimum_charuco_corners": MIN_CHARUCO_CORNERS,
        "target_image_count": args.target_count,
        "images": [],
    }

    saved_count = 0
    last_save_time = 0.0
    status_message = "Move board into view"

    print("ChArUco capture started")
    print(f"Camera: index={args.camera_index}, backend=dshow")
    print(f"Resolution: {actual_width}x{actual_height}")
    print(f"Output: {output_directory}")
    print("SPACE saves a valid raw frame")
    print("Q exits")
    print()

    try:
        # 给自动曝光一点稳定时间。
        for _ in range(20):
            capture.read()

        while True:
            ok, frame = capture.read()

            if not ok or frame is None or frame.size == 0:
                print("WARNING: invalid camera frame")
                continue

            raw_frame = frame.copy()
            display_frame = frame.copy()

            (
                charuco_corners,
                charuco_ids,
                marker_corners,
                marker_ids,
            ) = detector.detectBoard(frame)

            marker_count = (
                0
                if marker_ids is None
                else len(marker_ids)
            )

            corner_count = (
                0
                if charuco_ids is None
                else len(charuco_ids)
            )

            if (
                marker_ids is not None
                and len(marker_ids) > 0
            ):
                cv2.aruco.drawDetectedMarkers(
                    display_frame,
                    marker_corners,
                    marker_ids,
                )

            if (
                charuco_ids is not None
                and len(charuco_ids) > 0
            ):
                cv2.aruco.drawDetectedCornersCharuco(
                    display_frame,
                    charuco_corners,
                    charuco_ids,
                    (255, 0, 0),
                )

            coverage = calculate_board_coverage(
                charuco_corners,
                actual_width,
                actual_height,
            )

            ready_to_save = (
                corner_count >= MIN_CHARUCO_CORNERS
                and marker_count >= 4
                and coverage >= 0.03
            )

            if ready_to_save:
                status_message = "READY: press SPACE"
            else:
                status_message = (
                    "Need >=12 corners, >=4 markers, "
                    "and visible board area"
                )

            draw_status(
                display_frame,
                corner_count=corner_count,
                coverage=coverage,
                saved_count=saved_count,
                message=status_message,
                ready_to_save=ready_to_save,
            )

            cv2.imshow(
                "SO-101 ChArUco Calibration Capture",
                display_frame,
            )

            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), ord("Q"), 27):
                break

            if key == ord(" "):
                now = time.monotonic()

                if not ready_to_save:
                    print(
                        "REJECTED: board detection is not "
                        "good enough"
                    )
                    continue

                if now - last_save_time < 0.5:
                    print(
                        "REJECTED: wait before saving "
                        "another frame"
                    )
                    continue

                image_name = (
                    f"charuco_{saved_count + 1:03d}.png"
                )

                image_path = output_directory / image_name

                saved_ok = cv2.imwrite(
                    str(image_path),
                    raw_frame,
                )

                if not saved_ok:
                    print(
                        f"FAIL: could not save {image_path}"
                    )
                    continue

                session_metadata["images"].append(
                    {
                        "filename": image_name,
                        "charuco_corner_count": (
                            int(corner_count)
                        ),
                        "marker_count": int(marker_count),
                        "coverage": float(coverage),
                    }
                )

                saved_count += 1
                last_save_time = now

                print(
                    f"SAVED {saved_count}/"
                    f"{args.target_count}: "
                    f"{image_name}, "
                    f"corners={corner_count}, "
                    f"markers={marker_count}, "
                    f"coverage={coverage:.3f}"
                )

                if saved_count >= args.target_count:
                    print(
                        "Target image count reached"
                    )
                    break

    finally:
        capture.release()
        cv2.destroyAllWindows()

    metadata_path = (
        output_directory
        / "capture_metadata.json"
    )

    with metadata_path.open(
        "w",
        encoding="utf-8",
    ) as file:
        json.dump(
            session_metadata,
            file,
            indent=2,
            ensure_ascii=False,
        )

    print()
    print("=== CAPTURE SUMMARY ===")
    print(f"SAVED_IMAGES={saved_count}")
    print(f"OUTPUT_DIRECTORY={output_directory}")
    print(f"METADATA={metadata_path}")

    if saved_count < 15:
        print(
            "FAIL: fewer than 15 valid images were captured"
        )
        return 1

    if saved_count < args.target_count:
        print(
            "WARNING: target count was not reached"
        )
        return 0

    print("PASS: ChArUco capture completed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())