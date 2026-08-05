from __future__ import annotations

import json
import math
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]

INTRINSICS_PATH = (
    PROJECT_ROOT
    / "data"
    / "calibration"
    / "camera_calibration_report.json"
)

PLANE_CALIBRATION_PATH = (
    PROJECT_ROOT
    / "data"
    / "calibration"
    / "workspace_plane_calibration.json"
)

OUTPUT_DIRECTORY = (
    PROJECT_ROOT
    / "data"
    / "calibration"
    / "workspace_validation"
)

CAMERA_INDEX = 1
CAMERA_BACKEND = cv2.CAP_DSHOW

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
REQUESTED_FPS = 30.0

GRID_SPACING_MM = 25.0
MAX_CLICK_POINTS = 2


def load_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"File not found: {path}")

    with path.open("r", encoding="utf-8") as file:
        data = json.load(file)

    if not isinstance(data, dict):
        raise ValueError(f"Expected JSON object: {path}")

    return data


def load_calibration() -> tuple[
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
]:
    intrinsics = load_json(INTRINSICS_PATH)
    plane_calibration = load_json(
        PLANE_CALIBRATION_PATH
    )

    if plane_calibration.get("quality") != "PASS":
        raise ValueError(
            "Workspace plane calibration is not marked PASS"
        )

    camera_matrix = np.asarray(
        intrinsics["camera_matrix"],
        dtype=np.float64,
    )

    distortion_coefficients = np.asarray(
        intrinsics["distortion_coefficients"],
        dtype=np.float64,
    ).reshape(1, -1)

    pixel_to_plane = np.asarray(
        plane_calibration[
            "homography_undistorted_pixel_to_plane_mm"
        ],
        dtype=np.float64,
    )

    plane_to_pixel = np.asarray(
        plane_calibration[
            "homography_plane_mm_to_undistorted_pixel"
        ],
        dtype=np.float64,
    )

    if camera_matrix.shape != (3, 3):
        raise ValueError(
            f"Invalid camera matrix: {camera_matrix.shape}"
        )

    if pixel_to_plane.shape != (3, 3):
        raise ValueError(
            f"Invalid pixel-to-plane homography: "
            f"{pixel_to_plane.shape}"
        )

    if plane_to_pixel.shape != (3, 3):
        raise ValueError(
            f"Invalid plane-to-pixel homography: "
            f"{plane_to_pixel.shape}"
        )

    for name, matrix in (
        ("camera_matrix", camera_matrix),
        (
            "distortion_coefficients",
            distortion_coefficients,
        ),
        ("pixel_to_plane", pixel_to_plane),
        ("plane_to_pixel", plane_to_pixel),
    ):
        if not np.all(np.isfinite(matrix)):
            raise ValueError(
                f"{name} contains NaN or Inf"
            )

    return (
        camera_matrix,
        distortion_coefficients,
        pixel_to_plane,
        plane_to_pixel,
    )


def transform_points(
    points: np.ndarray,
    homography: np.ndarray,
) -> np.ndarray:
    points_array = np.asarray(
        points,
        dtype=np.float64,
    ).reshape(-1, 1, 2)

    transformed = cv2.perspectiveTransform(
        points_array,
        homography,
    )

    return transformed.reshape(-1, 2)


def pixel_to_plane_point(
    pixel: tuple[int, int],
    pixel_to_plane: np.ndarray,
) -> np.ndarray:
    result = transform_points(
        np.asarray(
            [[float(pixel[0]), float(pixel[1])]],
            dtype=np.float64,
        ),
        pixel_to_plane,
    )

    return result[0]


def safe_integer_point(
    point: np.ndarray,
) -> tuple[int, int] | None:
    if point.shape != (2,):
        return None

    if not np.all(np.isfinite(point)):
        return None

    if np.max(np.abs(point)) > 100_000:
        return None

    rounded = np.round(point).astype(int)

    return int(rounded[0]), int(rounded[1])


def draw_grid(
    image: np.ndarray,
    pixel_to_plane: np.ndarray,
    plane_to_pixel: np.ndarray,
) -> None:
    height, width = image.shape[:2]

    image_corners = np.asarray(
        [
            [0.0, 0.0],
            [float(width - 1), 0.0],
            [float(width - 1), float(height - 1)],
            [0.0, float(height - 1)],
        ],
        dtype=np.float64,
    )

    plane_corners = transform_points(
        image_corners,
        pixel_to_plane,
    )

    if not np.all(np.isfinite(plane_corners)):
        return

    minimum_x = (
        math.floor(
            float(np.min(plane_corners[:, 0]))
            / GRID_SPACING_MM
        )
        * GRID_SPACING_MM
    )

    maximum_x = (
        math.ceil(
            float(np.max(plane_corners[:, 0]))
            / GRID_SPACING_MM
        )
        * GRID_SPACING_MM
    )

    minimum_y = (
        math.floor(
            float(np.min(plane_corners[:, 1]))
            / GRID_SPACING_MM
        )
        * GRID_SPACING_MM
    )

    maximum_y = (
        math.ceil(
            float(np.max(plane_corners[:, 1]))
            / GRID_SPACING_MM
        )
        * GRID_SPACING_MM
    )

    # 防止异常标定导致生成过多网格线。
    if maximum_x - minimum_x > 2000:
        return

    if maximum_y - minimum_y > 2000:
        return

    x_values = np.arange(
        minimum_x,
        maximum_x + GRID_SPACING_MM,
        GRID_SPACING_MM,
    )

    y_values = np.arange(
        minimum_y,
        maximum_y + GRID_SPACING_MM,
        GRID_SPACING_MM,
    )

    for x_value in x_values:
        plane_line = np.asarray(
            [
                [x_value, minimum_y],
                [x_value, maximum_y],
            ],
            dtype=np.float64,
        )

        pixel_line = transform_points(
            plane_line,
            plane_to_pixel,
        )

        point_1 = safe_integer_point(pixel_line[0])
        point_2 = safe_integer_point(pixel_line[1])

        if point_1 is None or point_2 is None:
            continue

        is_axis = abs(x_value) < 1e-6

        cv2.line(
            image,
            point_1,
            point_2,
            (0, 0, 255) if is_axis else (90, 90, 90),
            2 if is_axis else 1,
            cv2.LINE_AA,
        )

    for y_value in y_values:
        plane_line = np.asarray(
            [
                [minimum_x, y_value],
                [maximum_x, y_value],
            ],
            dtype=np.float64,
        )

        pixel_line = transform_points(
            plane_line,
            plane_to_pixel,
        )

        point_1 = safe_integer_point(pixel_line[0])
        point_2 = safe_integer_point(pixel_line[1])

        if point_1 is None or point_2 is None:
            continue

        is_axis = abs(y_value) < 1e-6

        cv2.line(
            image,
            point_1,
            point_2,
            (0, 255, 0) if is_axis else (90, 90, 90),
            2 if is_axis else 1,
            cv2.LINE_AA,
        )


class WorkspaceValidator:
    def __init__(
        self,
        pixel_to_plane: np.ndarray,
    ) -> None:
        self.pixel_to_plane = pixel_to_plane

        self.mouse_pixel: tuple[int, int] | None = None
        self.mouse_plane: np.ndarray | None = None

        self.clicked_pixels: list[tuple[int, int]] = []
        self.clicked_plane_points: list[np.ndarray] = []

        self.last_display_frame: np.ndarray | None = None

    def clear_points(self) -> None:
        self.clicked_pixels.clear()
        self.clicked_plane_points.clear()

        print("CLEARED: measurement points")

    def handle_mouse(
        self,
        event: int,
        x: int,
        y: int,
        flags: int,
        parameter: Any,
    ) -> None:
        del flags
        del parameter

        self.mouse_pixel = (x, y)
        self.mouse_plane = pixel_to_plane_point(
            self.mouse_pixel,
            self.pixel_to_plane,
        )

        if event == cv2.EVENT_LBUTTONDOWN:
            if (
                len(self.clicked_pixels)
                >= MAX_CLICK_POINTS
            ):
                self.clear_points()

            plane_point = pixel_to_plane_point(
                (x, y),
                self.pixel_to_plane,
            )

            self.clicked_pixels.append((x, y))
            self.clicked_plane_points.append(
                plane_point
            )

            point_number = len(self.clicked_pixels)

            print(
                f"P{point_number}: "
                f"pixel=({x}, {y}), "
                f"plane=({plane_point[0]:.3f}, "
                f"{plane_point[1]:.3f}) mm"
            )

            if len(self.clicked_plane_points) == 2:
                difference = (
                    self.clicked_plane_points[1]
                    - self.clicked_plane_points[0]
                )

                distance = float(
                    np.linalg.norm(difference)
                )

                print(
                    "MEASUREMENT: "
                    f"dX={difference[0]:.3f} mm, "
                    f"dY={difference[1]:.3f} mm, "
                    f"distance={distance:.3f} mm"
                )

        elif event == cv2.EVENT_RBUTTONDOWN:
            self.clear_points()

    def draw_measurement(
        self,
        image: np.ndarray,
    ) -> None:
        for index, pixel in enumerate(
            self.clicked_pixels
        ):
            cv2.circle(
                image,
                pixel,
                6,
                (255, 0, 255),
                -1,
            )

            cv2.putText(
                image,
                f"P{index + 1}",
                (pixel[0] + 8, pixel[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 0, 255),
                2,
                cv2.LINE_AA,
            )

        if len(self.clicked_pixels) == 2:
            cv2.line(
                image,
                self.clicked_pixels[0],
                self.clicked_pixels[1],
                (255, 255, 0),
                2,
                cv2.LINE_AA,
            )

            difference = (
                self.clicked_plane_points[1]
                - self.clicked_plane_points[0]
            )

            distance = float(
                np.linalg.norm(difference)
            )

            midpoint = (
                (
                    self.clicked_pixels[0][0]
                    + self.clicked_pixels[1][0]
                )
                // 2,
                (
                    self.clicked_pixels[0][1]
                    + self.clicked_pixels[1][1]
                )
                // 2,
            )

            cv2.putText(
                image,
                f"{distance:.2f} mm",
                (midpoint[0] + 8, midpoint[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 0),
                2,
                cv2.LINE_AA,
            )

    def draw_mouse_coordinates(
        self,
        image: np.ndarray,
    ) -> None:
        if (
            self.mouse_pixel is None
            or self.mouse_plane is None
        ):
            return

        text = (
            f"pixel=({self.mouse_pixel[0]},"
            f"{self.mouse_pixel[1]}) | "
            f"X={self.mouse_plane[0]:.1f} mm, "
            f"Y={self.mouse_plane[1]:.1f} mm"
        )

        cv2.rectangle(
            image,
            (0, image.shape[0] - 42),
            (image.shape[1], image.shape[0]),
            (0, 0, 0),
            -1,
        )

        cv2.putText(
            image,
            text,
            (10, image.shape[0] - 14),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )


def main() -> int:
    (
        camera_matrix,
        distortion_coefficients,
        pixel_to_plane,
        plane_to_pixel,
    ) = load_calibration()

    capture = cv2.VideoCapture(
        CAMERA_INDEX,
        CAMERA_BACKEND,
    )

    if not capture.isOpened():
        print(
            "FAIL: could not open camera "
            "index=1 using DSHOW"
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

    validator = WorkspaceValidator(
        pixel_to_plane
    )

    window_name = (
        "SO-101 Workspace Coordinate Validation"
    )

    cv2.namedWindow(
        window_name,
        cv2.WINDOW_AUTOSIZE,
    )

    cv2.setMouseCallback(
        window_name,
        validator.handle_mouse,
    )

    print("Workspace coordinate validation")
    print("Left click: select P1 and P2")
    print("Right click or C: clear points")
    print("S: save current validation image")
    print("Q or ESC: quit")
    print()

    try:
        for _ in range(20):
            capture.read()

        while True:
            ok, raw_frame = capture.read()

            if (
                not ok
                or raw_frame is None
                or raw_frame.size == 0
            ):
                continue

            undistorted_frame = cv2.undistort(
                raw_frame,
                camera_matrix,
                distortion_coefficients,
                None,
                camera_matrix,
            )

            display = undistorted_frame.copy()

            draw_grid(
                display,
                pixel_to_plane,
                plane_to_pixel,
            )

            validator.draw_measurement(display)
            validator.draw_mouse_coordinates(display)

            cv2.putText(
                display,
                "Left click: P1/P2 | Right/C: clear | S: save | Q: quit",
                (10, 26),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

            validator.last_display_frame = (
                display.copy()
            )

            cv2.imshow(window_name, display)

            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), ord("Q"), 27):
                break

            if key in (ord("c"), ord("C")):
                validator.clear_points()

            if key in (ord("s"), ord("S")):
                if validator.last_display_frame is None:
                    continue

                OUTPUT_DIRECTORY.mkdir(
                    parents=True,
                    exist_ok=True,
                )

                timestamp = datetime.now().strftime(
                    "%Y%m%d_%H%M%S"
                )

                output_path = (
                    OUTPUT_DIRECTORY
                    / f"workspace_validation_{timestamp}.png"
                )

                saved = cv2.imwrite(
                    str(output_path),
                    validator.last_display_frame,
                )

                if saved:
                    print(f"SAVED: {output_path}")
                else:
                    print(
                        f"FAIL: could not save {output_path}"
                    )

    finally:
        capture.release()
        cv2.destroyAllWindows()

    print("Workspace validator stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())