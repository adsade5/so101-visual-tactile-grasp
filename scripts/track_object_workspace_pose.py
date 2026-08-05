from __future__ import annotations

import json
import math
import time
from collections import deque
from pathlib import Path
from typing import Any

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]

MARKER_CONFIG_PATH = (
    PROJECT_ROOT
    / "config"
    / "object_marker.json"
)

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

DICTIONARIES = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
}

SMOOTHING_WINDOW = 15
LOG_PERIOD_S = 0.5
LOST_TIMEOUT_S = 0.3

MAX_XY_STD_MM = 1.5
MAX_YAW_STD_DEG = 2.0
MAX_MARKER_SIZE_ERROR_MM = 2.5


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
        raise ValueError(
            f"Invalid camera matrix shape: {camera_matrix.shape}"
        )

    if not np.all(np.isfinite(camera_matrix)):
        raise ValueError(
            "Camera matrix contains NaN or Inf"
        )

    if not np.all(np.isfinite(distortion)):
        raise ValueError(
            "Distortion coefficients contain NaN or Inf"
        )

    return camera_matrix, distortion


def rigid_transform_inverse(
    transform: np.ndarray,
) -> np.ndarray:
    rotation = transform[:3, :3]
    translation = transform[:3, 3]

    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation

    return inverse


def recover_camera_from_plane_transform(
    camera_matrix: np.ndarray,
    plane_to_pixel_homography: np.ndarray,
) -> np.ndarray:
    """
    恢复桌面坐标系到相机坐标系的刚体变换。

    保存的单应矩阵满足：

        undistorted_pixel ~ K [r1 r2 t] [X Y 1]^T

    桌面坐标和translation的单位均为毫米。
    """

    normalized = (
        np.linalg.inv(camera_matrix)
        @ plane_to_pixel_homography
    )

    column_1 = normalized[:, 0]
    column_2 = normalized[:, 1]
    column_3 = normalized[:, 2]

    scale = 2.0 / (
        np.linalg.norm(column_1)
        + np.linalg.norm(column_2)
    )

    rotation_1 = scale * column_1
    rotation_2 = scale * column_2
    translation = scale * column_3

    # 可见桌面应位于相机正Z方向。
    if translation[2] < 0:
        rotation_1 = -rotation_1
        rotation_2 = -rotation_2
        translation = -translation

    rotation_3 = np.cross(
        rotation_1,
        rotation_2,
    )

    approximate_rotation = np.column_stack(
        [
            rotation_1,
            rotation_2,
            rotation_3,
        ]
    )

    # 恢复严格正交的旋转矩阵。
    u_matrix, _, vt_matrix = np.linalg.svd(
        approximate_rotation
    )

    rotation = u_matrix @ vt_matrix

    if np.linalg.det(rotation) < 0:
        u_matrix[:, -1] *= -1
        rotation = u_matrix @ vt_matrix

    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation

    return transform


def validate_recovered_plane_pose(
    camera_matrix: np.ndarray,
    plane_to_pixel_homography: np.ndarray,
    camera_from_plane: np.ndarray,
) -> float:
    test_points_mm = np.asarray(
        [
            [0.0, 0.0],
            [50.0, 0.0],
            [0.0, 50.0],
            [100.0, 50.0],
            [150.0, 100.0],
        ],
        dtype=np.float64,
    )

    homography_pixels = cv2.perspectiveTransform(
        test_points_mm.reshape(-1, 1, 2),
        plane_to_pixel_homography,
    ).reshape(-1, 2)

    object_points = np.column_stack(
        [
            test_points_mm,
            np.zeros(len(test_points_mm)),
        ]
    )

    rotation_vector, _ = cv2.Rodrigues(
        camera_from_plane[:3, :3]
    )

    projected_pixels, _ = cv2.projectPoints(
        object_points,
        rotation_vector,
        camera_from_plane[:3, 3],
        camera_matrix,
        np.zeros((1, 5), dtype=np.float64),
    )

    projected_pixels = projected_pixels.reshape(-1, 2)

    differences = (
        homography_pixels
        - projected_pixels
    )

    return float(
        math.sqrt(
            np.mean(
                np.sum(
                    differences * differences,
                    axis=1,
                )
            )
        )
    )


def pixels_to_camera_rays(
    pixels: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> np.ndarray:
    """
    将原始畸变像素转换为相机坐标系中的射线方向。

    返回：
        N x 3，每行形如[x, y, 1]。
    """

    pixel_array = np.asarray(
        pixels,
        dtype=np.float64,
    ).reshape(-1, 1, 2)

    normalized = cv2.undistortPoints(
        pixel_array,
        camera_matrix,
        distortion,
    ).reshape(-1, 2)

    rays = np.column_stack(
        [
            normalized[:, 0],
            normalized[:, 1],
            np.ones(len(normalized)),
        ]
    )

    ray_norms = np.linalg.norm(
        rays,
        axis=1,
        keepdims=True,
    )

    if np.any(ray_norms <= 1e-12):
        raise ValueError(
            "Invalid zero-length camera ray"
        )

    return rays / ray_norms


def intersect_rays_with_parallel_plane(
    rays_camera: np.ndarray,
    plane_from_camera: np.ndarray,
    target_plane_z_mm: float,
) -> np.ndarray:
    """
    将相机射线转换到桌面坐标系，并与Z固定的平面求交。

    target_plane_z_mm是物块顶面在桌面坐标系中的Z。
    """

    camera_origin_plane = (
        plane_from_camera[:3, 3]
    )

    plane_rotation_from_camera = (
        plane_from_camera[:3, :3]
    )

    directions_plane = (
        plane_rotation_from_camera
        @ rays_camera.T
    ).T

    denominators = directions_plane[:, 2]

    if np.any(np.abs(denominators) < 1e-9):
        raise ValueError(
            "Camera ray is parallel to the target plane"
        )

    distances = (
        target_plane_z_mm
        - camera_origin_plane[2]
    ) / denominators

    if np.any(distances <= 0):
        raise ValueError(
            "Target plane intersection is behind the camera"
        )

    intersections = (
        camera_origin_plane.reshape(1, 3)
        + distances.reshape(-1, 1)
        * directions_plane
    )

    return intersections


def calculate_marker_geometry(
    marker_corners_plane: np.ndarray,
) -> tuple[
    np.ndarray,
    float,
    float,
]:
    """
    输入角点顺序：
        0 左上
        1 右上
        2 右下
        3 左下

    返回：
        Marker中心三维坐标
        yaw角
        四边平均长度
    """

    if marker_corners_plane.shape != (4, 3):
        raise ValueError(
            "Expected four 3D marker corners"
        )

    center = np.mean(
        marker_corners_plane,
        axis=0,
    )

    left_midpoint = 0.5 * (
        marker_corners_plane[0]
        + marker_corners_plane[3]
    )

    right_midpoint = 0.5 * (
        marker_corners_plane[1]
        + marker_corners_plane[2]
    )

    marker_x_axis = (
        right_midpoint
        - left_midpoint
    )

    xy_norm = math.hypot(
        float(marker_x_axis[0]),
        float(marker_x_axis[1]),
    )

    if xy_norm <= 1e-9:
        raise ValueError(
            "Marker X axis is degenerate"
        )

    yaw_degrees = math.degrees(
        math.atan2(
            float(marker_x_axis[1]),
            float(marker_x_axis[0]),
        )
    )

    side_lengths = []

    for index in range(4):
        point_a = marker_corners_plane[index]
        point_b = marker_corners_plane[
            (index + 1) % 4
        ]

        side_lengths.append(
            float(
                np.linalg.norm(
                    point_b[:2] - point_a[:2]
                )
            )
        )

    mean_marker_size_mm = float(
        np.mean(side_lengths)
    )

    return (
        center,
        wrap_degrees(yaw_degrees),
        mean_marker_size_mm,
    )


def apply_marker_center_offset(
    marker_center_xy: np.ndarray,
    yaw_degrees: float,
    offset_x_mm: float,
    offset_y_mm: float,
) -> np.ndarray:
    """
    将Marker局部坐标系中的固定偏移旋转到桌面坐标系。

    偏移为0时，物块中心等于Marker中心。
    """

    yaw_radians = math.radians(
        yaw_degrees
    )

    rotation_2d = np.asarray(
        [
            [
                math.cos(yaw_radians),
                -math.sin(yaw_radians),
            ],
            [
                math.sin(yaw_radians),
                math.cos(yaw_radians),
            ],
        ],
        dtype=np.float64,
    )

    local_offset = np.asarray(
        [offset_x_mm, offset_y_mm],
        dtype=np.float64,
    )

    return (
        marker_center_xy
        + rotation_2d @ local_offset
    )


def wrap_degrees(
    angle_degrees: float,
) -> float:
    return (
        angle_degrees + 180.0
    ) % 360.0 - 180.0


def gripper_symmetric_yaw(
    yaw_degrees: float,
) -> float:
    return (
        yaw_degrees + 90.0
    ) % 180.0 - 90.0


def circular_mean_degrees(
    values: list[float],
) -> float:
    radians = np.radians(
        np.asarray(values, dtype=np.float64)
    )

    mean_sine = float(
        np.mean(np.sin(radians))
    )

    mean_cosine = float(
        np.mean(np.cos(radians))
    )

    return wrap_degrees(
        math.degrees(
            math.atan2(
                mean_sine,
                mean_cosine,
            )
        )
    )


def calculate_yaw_std_degrees(
    values: list[float],
    mean_degrees: float,
) -> float:
    differences = [
        wrap_degrees(
            value - mean_degrees
        )
        for value in values
    ]

    return float(
        np.std(
            np.asarray(
                differences,
                dtype=np.float64,
            )
        )
    )


def find_target_index(
    marker_ids: np.ndarray,
    target_id: int,
) -> int | None:
    flat_ids = marker_ids.reshape(-1)

    matches = np.where(
        flat_ids == target_id
    )[0]

    if len(matches) == 0:
        return None

    return int(matches[0])


def draw_text(
    image: np.ndarray,
    text: str,
    line: int,
    color: tuple[int, int, int],
) -> None:
    cv2.putText(
        image,
        text,
        (12, 28 + line * 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.60,
        color,
        2,
        cv2.LINE_AA,
    )


def main() -> int:
    marker_config = load_json(
        MARKER_CONFIG_PATH
    )

    plane_calibration = load_json(
        PLANE_CALIBRATION_PATH
    )

    if plane_calibration.get("quality") != "PASS":
        raise ValueError(
            "Workspace plane calibration is not PASS"
        )

    camera_matrix, distortion = (
        load_intrinsics()
    )

    dictionary_name = str(
        marker_config["dictionary"]
    )

    if dictionary_name not in DICTIONARIES:
        raise ValueError(
            f"Unsupported dictionary: {dictionary_name}"
        )

    target_id = int(
        marker_config["target_id"]
    )

    if target_id < 0:
        raise ValueError(
            "target_id must be fixed before tracking"
        )

    marker_size_mm = (
        float(marker_config["marker_size_m"])
        * 1000.0
    )

    object_top_height_mm = float(
        marker_config["object_top_height_mm"]
    )

    if marker_size_mm <= 0:
        raise ValueError(
            "marker_size_m must be positive"
        )

    if object_top_height_mm <= 0:
        raise ValueError(
            "object_top_height_mm must be positive"
        )

    center_offset = marker_config.get(
        "marker_center_offset_mm",
        {},
    )

    center_offset_x_mm = float(
        center_offset.get("x", 0.0)
    )

    center_offset_y_mm = float(
        center_offset.get("y", 0.0)
    )

    camera_config = marker_config["camera"]

    camera_index = int(
        camera_config["index"]
    )

    image_width = int(
        camera_config["width"]
    )

    image_height = int(
        camera_config["height"]
    )

    plane_to_pixel = np.asarray(
        plane_calibration[
            "homography_plane_mm_to_undistorted_pixel"
        ],
        dtype=np.float64,
    )

    if plane_to_pixel.shape != (3, 3):
        raise ValueError(
            "Invalid plane-to-pixel homography"
        )

    camera_from_plane = (
        recover_camera_from_plane_transform(
            camera_matrix,
            plane_to_pixel,
        )
    )

    plane_from_camera = rigid_transform_inverse(
        camera_from_plane
    )

    camera_position_in_plane = (
        plane_from_camera[:3, 3]
    )

    if abs(camera_position_in_plane[2]) < 1e-6:
        raise ValueError(
            "Recovered camera is on the workspace plane"
        )

    # 桌面坐标系Z轴的方向可能朝向相机，也可能背向相机。
    # 物块顶面必须位于桌面朝向相机的一侧。
    top_plane_z_mm = math.copysign(
        object_top_height_mm,
        float(camera_position_in_plane[2]),
    )

    recovery_error_px = (
        validate_recovered_plane_pose(
            camera_matrix,
            plane_to_pixel,
            camera_from_plane,
        )
    )

    if recovery_error_px > 2.0:
        raise ValueError(
            "Recovered plane pose is inconsistent with "
            f"the saved homography: {recovery_error_px:.3f}px"
        )

    dictionary = cv2.aruco.getPredefinedDictionary(
        DICTIONARIES[dictionary_name]
    )

    detector_parameters = (
        cv2.aruco.DetectorParameters()
    )

    detector_parameters.cornerRefinementMethod = (
        cv2.aruco.CORNER_REFINE_SUBPIX
    )

    detector = cv2.aruco.ArucoDetector(
        dictionary,
        detector_parameters,
    )

    capture = cv2.VideoCapture(
        camera_index,
        cv2.CAP_DSHOW,
    )

    if not capture.isOpened():
        print(
            "FAIL: could not open workspace camera"
        )
        return 1

    capture.set(
        cv2.CAP_PROP_FRAME_WIDTH,
        image_width,
    )

    capture.set(
        cv2.CAP_PROP_FRAME_HEIGHT,
        image_height,
    )

    capture.set(
        cv2.CAP_PROP_FPS,
        30.0,
    )

    pose_samples: deque[
        tuple[float, float, float, float]
    ] = deque(
        maxlen=SMOOTHING_WINDOW
    )

    last_detection_time = 0.0
    last_log_time = 0.0

    print(
        "SO-101 known-height workspace object tracking"
    )
    print(f"Target marker ID: {target_id}")
    print(
        f"Configured marker size: "
        f"{marker_size_mm:.1f} mm"
    )
    print(
        f"Known object top height: "
        f"{object_top_height_mm:.1f} mm"
    )
    print(
        "Homography pose recovery RMS: "
        f"{recovery_error_px:.4f} px"
    )
    print(
        "Camera position in plane frame: "
        f"({camera_position_in_plane[0]:.1f}, "
        f"{camera_position_in_plane[1]:.1f}, "
        f"{camera_position_in_plane[2]:.1f}) mm"
    )
    print(
        "PnP depth is not used for the final pose."
    )
    print("Q or ESC: quit")
    print()

    try:
        for _ in range(20):
            capture.read()

        while True:
            ok, frame = capture.read()

            if (
                not ok
                or frame is None
                or frame.size == 0
            ):
                continue

            display = frame.copy()

            marker_corners, marker_ids, _ = (
                detector.detectMarkers(frame)
            )

            target_index = None

            if marker_ids is not None:
                cv2.aruco.drawDetectedMarkers(
                    display,
                    marker_corners,
                    marker_ids,
                )

                target_index = find_target_index(
                    marker_ids,
                    target_id,
                )

            now = time.monotonic()

            if target_index is None:
                if (
                    now - last_detection_time
                    > LOST_TIMEOUT_S
                ):
                    pose_samples.clear()

                draw_text(
                    display,
                    (
                        f"TARGET ID={target_id}: "
                        "NOT DETECTED"
                    ),
                    0,
                    (0, 0, 255),
                )

                draw_text(
                    display,
                    "No stale pose is retained",
                    1,
                    (0, 165, 255),
                )

            else:
                raw_corners = np.asarray(
                    marker_corners[target_index],
                    dtype=np.float64,
                ).reshape(4, 2)

                try:
                    rays_camera = (
                        pixels_to_camera_rays(
                            raw_corners,
                            camera_matrix,
                            distortion,
                        )
                    )

                    corners_plane = (
                        intersect_rays_with_parallel_plane(
                            rays_camera,
                            plane_from_camera,
                            top_plane_z_mm,
                        )
                    )

                    (
                        marker_center_plane,
                        yaw_degrees,
                        measured_marker_size_mm,
                    ) = calculate_marker_geometry(
                        corners_plane
                    )

                    object_center_xy = (
                        apply_marker_center_offset(
                            marker_center_plane[:2],
                            yaw_degrees,
                            center_offset_x_mm,
                            center_offset_y_mm,
                        )
                    )

                    x_mm = float(
                        object_center_xy[0]
                    )

                    y_mm = float(
                        object_center_xy[1]
                    )

                    pose_samples.append(
                        (
                            x_mm,
                            y_mm,
                            yaw_degrees,
                            measured_marker_size_mm,
                        )
                    )

                    last_detection_time = now

                except ValueError as exc:
                    pose_samples.clear()

                    draw_text(
                        display,
                        f"GEOMETRY ERROR: {exc}",
                        0,
                        (0, 0, 255),
                    )

                    cv2.imshow(
                        "SO-101 Known-Height Object Pose",
                        display,
                    )

                    key = cv2.waitKey(1) & 0xFF

                    if key in (
                        ord("q"),
                        ord("Q"),
                        27,
                    ):
                        break

                    continue

                sample_array = np.asarray(
                    pose_samples,
                    dtype=np.float64,
                )

                smooth_x = float(
                    np.median(
                        sample_array[:, 0]
                    )
                )

                smooth_y = float(
                    np.median(
                        sample_array[:, 1]
                    )
                )

                yaw_values = (
                    sample_array[:, 2].tolist()
                )

                smooth_yaw = (
                    circular_mean_degrees(
                        yaw_values
                    )
                )

                smooth_marker_size = float(
                    np.median(
                        sample_array[:, 3]
                    )
                )

                x_std = float(
                    np.std(
                        sample_array[:, 0]
                    )
                )

                y_std = float(
                    np.std(
                        sample_array[:, 1]
                    )
                )

                yaw_std = (
                    calculate_yaw_std_degrees(
                        yaw_values,
                        smooth_yaw,
                    )
                )

                marker_size_std = float(
                    np.std(
                        sample_array[:, 3]
                    )
                )

                marker_size_error = abs(
                    smooth_marker_size
                    - marker_size_mm
                )

                stable = (
                    len(pose_samples) >= 10
                    and x_std <= MAX_XY_STD_MM
                    and y_std <= MAX_XY_STD_MM
                    and yaw_std <= MAX_YAW_STD_DEG
                    and marker_size_error
                    <= MAX_MARKER_SIZE_ERROR_MM
                )

                gripper_yaw = (
                    gripper_symmetric_yaw(
                        smooth_yaw
                    )
                )

                center_pixel_float = np.mean(
                    raw_corners,
                    axis=0,
                )

                center_pixel = (
                    int(round(center_pixel_float[0])),
                    int(round(center_pixel_float[1])),
                )

                left_midpoint_pixel = 0.5 * (
                    raw_corners[0]
                    + raw_corners[3]
                )

                right_midpoint_pixel = 0.5 * (
                    raw_corners[1]
                    + raw_corners[2]
                )

                left_pixel = tuple(
                    np.round(
                        left_midpoint_pixel
                    ).astype(int)
                )

                right_pixel = tuple(
                    np.round(
                        right_midpoint_pixel
                    ).astype(int)
                )

                cv2.circle(
                    display,
                    center_pixel,
                    6,
                    (0, 0, 255),
                    -1,
                )

                cv2.arrowedLine(
                    display,
                    left_pixel,
                    right_pixel,
                    (255, 0, 255),
                    3,
                    cv2.LINE_AA,
                    tipLength=0.18,
                )

                status_color = (
                    (0, 255, 0)
                    if stable
                    else (0, 165, 255)
                )

                draw_text(
                    display,
                    (
                        f"ID={target_id} | "
                        f"X={smooth_x:.1f} mm | "
                        f"Y={smooth_y:.1f} mm"
                    ),
                    0,
                    status_color,
                )

                draw_text(
                    display,
                    (
                        f"height={object_top_height_mm:.1f} mm "
                        "(KNOWN)"
                    ),
                    1,
                    status_color,
                )

                draw_text(
                    display,
                    (
                        f"yaw={smooth_yaw:.1f} deg | "
                        f"gripper yaw={gripper_yaw:.1f} deg"
                    ),
                    2,
                    status_color,
                )

                draw_text(
                    display,
                    (
                        f"marker size="
                        f"{smooth_marker_size:.2f} mm | "
                        f"expected={marker_size_mm:.2f} mm"
                    ),
                    3,
                    status_color,
                )

                draw_text(
                    display,
                    (
                        f"samples={len(pose_samples)}/"
                        f"{SMOOTHING_WINDOW} | "
                        f"stable={'YES' if stable else 'NO'}"
                    ),
                    4,
                    status_color,
                )

                draw_text(
                    display,
                    (
                        f"std: X={x_std:.2f}, "
                        f"Y={y_std:.2f} mm, "
                        f"yaw={yaw_std:.2f} deg, "
                        f"size={marker_size_std:.2f} mm"
                    ),
                    5,
                    status_color,
                )

                if now - last_log_time >= LOG_PERIOD_S:
                    print(
                        f"ID={target_id} | "
                        f"X={smooth_x:.2f} mm, "
                        f"Y={smooth_y:.2f} mm, "
                        f"height={object_top_height_mm:.2f} mm "
                        "(known), "
                        f"yaw={smooth_yaw:.2f} deg, "
                        f"gripper_yaw={gripper_yaw:.2f} deg, "
                        f"marker_size="
                        f"{smooth_marker_size:.2f} mm, "
                        f"stable={stable}"
                    )

                    last_log_time = now

            cv2.imshow(
                "SO-101 Known-Height Object Pose",
                display,
            )

            key = cv2.waitKey(1) & 0xFF

            if key in (
                ord("q"),
                ord("Q"),
                27,
            ):
                break

    finally:
        capture.release()
        cv2.destroyAllWindows()

    print("Object pose tracker stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())